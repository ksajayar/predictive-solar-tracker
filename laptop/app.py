"""
app.py — Streamlit dashboard. Display and operator input ONLY.

app.py never touches the serial port / socket directly (that's link.py's job)
and never runs weather logic itself (that's weather.py's job). It just reads
thread-safe snapshots from both and renders them, plus forwards a handful of
operator actions (scenario choice, AUTO/HOLD, "simulate API failure") back
into those services.

Run:
    streamlit run app.py
"""
from __future__ import annotations

import streamlit as st
from plotly.subplots import make_subplots

from link import Link, FLAG_DARK, FLAG_LIMIT
from weather import WeatherService, SCENARIOS, DEFAULT_CFG

# Keep in sync with the tracker's own E_START (fake_esp32.py / real firmware).
# Only used here to label the graph's deadband and the "CENTERED" cutoff.
E_START_DISPLAY = 0.06

STATE_LABEL = {"TRK": "TRACKING", "STW": "SAFE MODE", "RPN": "REOPENING", "HLD": "HOLD"}

st.set_page_config(page_title="Solar Tracker", page_icon="☀️", layout="wide")


@st.cache_resource
def services():
    """Background threads start exactly once per Streamlit server process,
    never on a script rerun — this is what makes reconnection automatic."""
    link = Link().start()
    wx = WeatherService(link).start()
    return link, wx


link, wxs = services()


# --------------------------------------------------------------------------
# Small pure helpers (display logic only — never feed back into control)
# --------------------------------------------------------------------------


def light_balance(t: dict):
    """(1 - |error|) * 100, clamped. A control metric, not an energy measurement."""
    if t["flags"] & FLAG_DARK:
        return None
    return max(0.0, min(100.0, (1.0 - abs(t["err"])) * 100.0))


def direction_label(t: dict) -> str:
    if t["flags"] & FLAG_DARK:
        return "NO LIGHT"
    if abs(t["err"]) < E_START_DISPLAY:
        return "CENTERED"
    return "◀ LIGHT LEFT" if t["err"] > 0 else "LIGHT RIGHT ▶"


def banner(link_snap: dict, wx_snap: dict):
    t = link_snap["latest"]
    if not link_snap["connected"] or t is None:
        return "#6b7280", ("⚪ ESP32 DISCONNECTED — reconnecting automatically. "
                            "The tracker keeps running on its own; nothing here controls it.")
    state = t["state"]
    if state == "STW":
        if wx_snap["verdict"] == "UNKNOWN":
            why = "weather data unavailable — the SAFE latch holds regardless"
        else:
            why = wx_snap["reason"]
        return "#dc2626", f"\U0001f534 SAFE MODE — {why} — stowed at {t['target']:+.0f}° (configured stow position)"
    if state == "RPN":
        return "#d97706", "\U0001f7e0 REOPENING — conditions clear, slowly returning to tracking"
    if state == "HLD":
        return "#2563eb", f"\U0001f535 HOLD — commanded to {t['target']:+.0f}° (debug/demo mode)"
    if wx_snap["verdict"] == "UNKNOWN":
        return "#6b7280", "⚪ WEATHER UNAVAILABLE — local light tracking continues"
    return "#16a34a", f"\U0001f7e2 TRACKING — {wx_snap['reason']}"


def trend_figure(history: list):
    if not history:
        return None
    now_ms = history[-1]["ms"]
    window = [h for h in history if now_ms - h["ms"] <= 60_000]
    xs = [(h["ms"] - now_ms) / 1000.0 for h in window]
    raw = [light_balance(h) for h in window]
    # display-only smoothing (5-sample rolling mean) — never fed back to the controller
    smooth = []
    for i in range(len(raw)):
        chunk = [v for v in raw[max(0, i - 4):i + 1] if v is not None]
        smooth.append(sum(chunk) / len(chunk) if chunk else None)

    fig = make_subplots(rows=2, cols=1, shared_xaxes=True, vertical_spacing=0.10,
                         subplot_titles=("Light balance % (green band = tracker deadband)",
                                          "Panel angle ° (commanded, solid) vs target (dashed)"))
    fig.add_hrect(y0=100 * (1 - E_START_DISPLAY), y1=100, fillcolor="green", opacity=0.12,
                  line_width=0, row=1, col=1)
    fig.add_scatter(x=xs, y=raw, line=dict(width=1, color="#9ca3af"), name="raw", row=1, col=1)
    fig.add_scatter(x=xs, y=smooth, line=dict(width=3, color="#16a34a"), name="smoothed", row=1, col=1)
    fig.add_scatter(x=xs, y=[h["angle"] for h in window], line=dict(width=3, color="#2563eb"),
                     name="angle", row=2, col=1)
    fig.add_scatter(x=xs, y=[h["target"] for h in window], line=dict(width=1, dash="dash", color="#dc2626"),
                     name="target", row=2, col=1)
    fig.update_yaxes(range=[0, 100], row=1, col=1)
    fig.update_yaxes(range=[-60, 60], row=2, col=1)
    fig.update_xaxes(range=[-60, 0], title_text="seconds ago", row=2, col=1)
    fig.update_layout(height=420, showlegend=False, margin=dict(l=10, r=10, t=30, b=10))
    return fig


def last_correction(history: list):
    """Length + size of the most recently completed MOVING episode, purely
    from telemetry flags — a legitimate measured metric, not a fake one."""
    from link import FLAG_MOVING
    episode_start = None
    result = None
    for h in history:
        moving = bool(h["flags"] & FLAG_MOVING)
        if moving and episode_start is None:
            episode_start = h
        elif not moving and episode_start is not None:
            result = (episode_start, h)
            episode_start = None
    if result is None:
        return None
    start, end = result
    return (end["ms"] - start["ms"]) / 1000.0, end["angle"] - start["angle"]


# --------------------------------------------------------------------------
# Layout
# --------------------------------------------------------------------------

st.title("☀️ Weather-Aware Solar Tracker")

scenario = st.radio("Weather input", ["LIVE", *SCENARIOS.keys()], horizontal=True,
                     help="LIVE polls Open-Meteo. The others inject a fixed simulated "
                          "reading through the exact same decision function as live data.")
wxs.set_scenario(scenario)


@st.fragment(run_every=0.5)
def live_view():
    link_snap = link.snapshot()
    wx_snap = wxs.snapshot()
    t = link_snap["latest"]

    status_cols = st.columns(2)
    status_cols[0].markdown(
        ("\U0001f7e2 **ESP32 CONNECTED**" if link_snap["connected"] else "\U0001f534 **ESP32 DISCONNECTED**")
        + f"  · `{link_snap['port'] or '—'}`"
    )
    wx_status = ("SIMULATED" if wx_snap["scenario"] != "LIVE"
                 else ("LIVE" if wx_snap["fetch_ok"] else "UNAVAILABLE"))
    status_cols[1].markdown(f"Weather: **{wx_status}**")

    if wx_snap["scenario"] != "LIVE":
        st.markdown(
            f"<div style='background:#7c3aed;color:#fff;padding:6px 14px;border-radius:6px;"
            f"font-weight:600;margin-bottom:8px'>SIMULATED WEATHER — {wx_snap['scenario']} "
            f"(presenter input, not live data)</div>", unsafe_allow_html=True)

    if t is None:
        st.info("Waiting for ESP32 telemetry... (is fake_esp32.py / the real board running?)")
        return

    color, text = banner(link_snap, wx_snap)
    st.markdown(f"<div style='background:{color};color:#fff;padding:16px 20px;border-radius:8px;"
                f"font-size:24px;font-weight:600;margin-bottom:10px'>{text}</div>", unsafe_allow_html=True)

    c1, c2, c3 = st.columns(3)

    with c1:
        st.subheader("☀️ Light sensors")
        st.write(f"Left **{t['L']:.0f}**  ·  Right **{t['R']:.0f}**  ·  error **{t['err']:+.3f}**")
        bal = light_balance(t)
        st.metric("Light Balance", "—" if bal is None else f"{bal:.0f}%")
        st.caption("How evenly the two LDR sensors are illuminated. "
                   "A control metric, not an energy measurement.")
        paused = t["state"] in ("STW", "HLD")
        label = direction_label(t)
        st.write(f"**{label}**" + ("  · *tracking paused*" if paused else ""))

    with c2:
        st.subheader("\U0001f3af Panel")
        st.metric("Commanded angle", f"{t['angle']:+.1f}°")
        st.caption("Commanded, not measured — this servo has no position feedback.")
        st.write(f"Target **{t['target']:+.1f}°**  ·  State **{STATE_LABEL[t['state']]}**"
                 + ("  · at limit" if t["flags"] & FLAG_LIMIT else ""))
        lc = last_correction(link_snap["history"])
        if lc:
            st.caption(f"Last correction: {lc[0]:.1f}s, {lc[1]:+.0f}°")

    with c3:
        wx = wx_snap["wx"]
        st.subheader(f"\U0001f324 Weather ({'SIMULATED' if wx_snap['scenario'] != 'LIVE' else 'LIVE'})")
        if wx_snap["verdict"] == "UNKNOWN" or wx is None:
            st.warning("WEATHER DATA UNAVAILABLE")
            if wx_snap["last_error"]:
                st.caption(wx_snap["last_error"])
        if wx is not None:
            from weather import describe_code
            st.write(describe_code(wx.code))
            st.write(f"Gust **{wx.gust_kmh:.0f}** km/h · next 1h **{wx.gust_next_kmh:.0f}** km/h "
                     f"(SAFE ≥ {DEFAULT_CFG.safe_gust_kmh:.0f}, demo threshold)")
            st.write(f"Wind {wx.wind_kmh:.0f} km/h · Cloud {wx.cloud_pct:.0f}% (context only)")
        st.write(f"Decision: **{wx_snap['verdict']}** — {wx_snap['reason']}")

    fig = trend_figure(link_snap["history"])
    if fig is not None:
        st.plotly_chart(fig, use_container_width=True, key="trend")

    with st.expander("Diagnostics"):
        st.write(f"Sent to ESP32: `C,{','.join(str(x) for x in link_snap['cmd'])}`")
        st.write(f"Bad/malformed lines: {link_snap['bad_lines']}  ·  "
                 f"ESP32 boots/reboots seen: {link_snap['boots']}")
        st.write(f"Telemetry log: `{link_snap['log_path']}`")
        st.write(f"Weather fetch: {'OK' if wx_snap['fetch_ok'] else 'FAILED'}"
                 + (f" — {wx_snap['last_error']}" if wx_snap['last_error'] else ""))
        st.code("\n".join(link_snap["events"][-10:]) or "no events yet")


live_view()

with st.sidebar:
    st.header("Operator / debug")
    st.caption("Secondary controls — not part of the judge-facing demo story.")
    hold_angle = st.slider("HOLD angle", -55, 55, 0)
    b1, b2 = st.columns(2)
    if b1.button("HOLD", use_container_width=True):
        link.set_mode("HOLD", hold_angle)
    if b2.button("AUTO", use_container_width=True):
        link.set_mode("AUTO")

    st.divider()
    offline = st.toggle("Simulate weather API failure", value=wxs.force_offline)
    if offline != wxs.force_offline:
        wxs.force_offline = offline
        wxs.request_refresh()
    if st.button("Refresh weather now"):
        wxs.request_refresh()
