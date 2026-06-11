"""
Stage 2 — Proportional-Derivative (PD) Controller
===================================================
Adding a derivative term damps the overshoot seen in Stage 1.
The drone now approaches the target smoothly — derivative action
acts like predictive braking: the faster we approach, the harder
we brake.

What to observe:
  - No overshoot; the drone settles cleanly at ~500 m
  - Residual steady-state error is tiny (~0.2 m) thanks to drag equilibrium
  - PD cannot fight a constant external disturbance (shown in Stage 3)

Physics model:  m * a = Thrust - m*g - c_drag * v
Control law:    u = Kp * e + Kd * ė + m*g
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
MASS       = 1.0
GRAVITY    = 9.81
TARGET_ALT = 500.0

Kp     = 0.05    # lower than Stage 1 — derivative now handles the heavy lifting
Kd     = 0.10    # derivative gain — damps the approach velocity
C_DRAG = 0.25

DT    = 0.05
T_END = 60.0


# ---------------------------------------------------------------------------
# Pre-simulate
# ---------------------------------------------------------------------------
t = np.arange(0, T_END + DT, DT)
N = len(t)

altitude = np.zeros(N)
velocity = np.zeros(N)
thrust   = np.zeros(N)
error    = np.zeros(N)

e_prev = TARGET_ALT - altitude[0]

for i in range(1, N):
    error[i-1] = TARGET_ALT - altitude[i-1]
    e_dot      = (error[i-1] - e_prev) / DT

    # PD + gravity feedforward
    thrust[i-1] = Kp * error[i-1] + Kd * e_dot + MASS * GRAVITY

    accel = (thrust[i-1] - MASS * GRAVITY - C_DRAG * velocity[i-1]) / MASS

    velocity[i] = velocity[i-1] + accel * DT
    altitude[i] = altitude[i-1] + velocity[i] * DT

    if altitude[i] < 0:
        altitude[i] = 0.0
        velocity[i] = 0.0

    e_prev = error[i-1]

error[-1]  = TARGET_ALT - altitude[-1]
thrust[-1] = thrust[-2]


# ---------------------------------------------------------------------------
# Figure setup  (same dark theme as Stage 1)
# ---------------------------------------------------------------------------
BG_DARK  = (0.13, 0.13, 0.15)
BG_PANEL = (0.10, 0.12, 0.18)
COLOR_TXT  = (0.75, 0.75, 0.75)
COLOR_GRID = (0.28, 0.28, 0.28)

plt.ion()
fig = plt.figure(figsize=(10, 5.8), facecolor=BG_DARK)
fig.canvas.manager.set_window_title("Stage 2 — PD Controller (500 m)")


def make_side_axes(rect, ylim, title, ylabel):
    ax = fig.add_axes(rect, facecolor=BG_PANEL)
    ax.tick_params(colors=COLOR_TXT, labelsize=8)
    ax.grid(color=COLOR_GRID, alpha=0.5)
    ax.set_xlim([0, T_END])
    ax.set_ylim(ylim)
    ax.set_ylabel(ylabel, color=COLOR_TXT, fontsize=9)
    ax.set_title(title, color=(0.9, 0.9, 0.9), fontsize=10)
    for spine in ax.spines.values():
        spine.set_edgecolor(COLOR_TXT)
    return ax


ax_scene = fig.add_axes([0.05, 0.08, 0.38, 0.85], facecolor=BG_PANEL)
ax_scene.tick_params(colors=COLOR_TXT)
ax_scene.set_xlim([-25, 25])
ax_scene.set_ylim([0, 650])
ax_scene.set_xticks([])
ax_scene.set_yticks(np.arange(0, 700, 50))
ax_scene.set_yticklabels([f"{v} m" for v in np.arange(0, 700, 50)], fontsize=9)
ax_scene.grid(color=COLOR_GRID, alpha=0.4)
ax_scene.set_ylabel("Altitude [m]", color=COLOR_TXT, fontsize=10)
ax_scene.set_title("Stage 2 — PD Control Simulation", color=(0.9, 0.9, 0.9), fontsize=11)

ax_scene.add_patch(
    patches.Rectangle((-25, 0), 50, 10, facecolor=(0.22, 0.18, 0.12), edgecolor="none")
)
ax_scene.plot([-25, 25], [TARGET_ALT, TARGET_ALT], "--", color=(0.9, 0.3, 0.3), lw=1.4)
ax_scene.text(2, TARGET_ALT + 10, "target", color=(0.9, 0.3, 0.3), fontsize=9)

h_body     = patches.Polygon(np.zeros((4, 2)), facecolor=(0.25, 0.55, 0.95), edgecolor="none")
ax_scene.add_patch(h_body)
h_arm_l,   = ax_scene.plot([], [], color=(0.5, 0.75, 1.0), lw=2.5)
h_arm_r,   = ax_scene.plot([], [], color=(0.5, 0.75, 1.0), lw=2.5)
h_rotor_l, = ax_scene.plot([], [], color=(0.8, 0.9, 1.0), lw=3)
h_rotor_r, = ax_scene.plot([], [], color=(0.8, 0.9, 1.0), lw=3)
h_flame,   = ax_scene.plot([], [], color=(0.3, 0.8, 1.0), lw=3)

h_alt_lbl = ax_scene.text(10.0, 0, "", color=(0.85, 0.85, 0.85), fontsize=9)
h_time    = ax_scene.text(-23, 620, "t = 0.00 s", color=(0.7, 0.7, 0.7), fontsize=9)
h_err_lbl = ax_scene.text(-23, 590, "error = 0.00 m", color=(1.0, 0.5, 0.5), fontsize=9)

h_ss_line, = ax_scene.plot([], [], "m-", lw=1.5, visible=False)
h_ss_text  = ax_scene.text(-23, 0, "", color=(1, 0.5, 1), fontsize=9, visible=False)

ax_alt = make_side_axes([0.52, 0.71, 0.43, 0.22], [0, 650], "Altitude", "m")
ax_alt.plot([0, T_END], [TARGET_ALT, TARGET_ALT], "--", color=(0.85, 0.3, 0.3), lw=1)
h_line_alt, = ax_alt.plot([], [], color=(0.3, 0.65, 1.0), lw=1.5)

ax_err = make_side_axes([0.52, 0.40, 0.43, 0.22], [-200, 550], "Error", "m")
ax_err.plot([0, T_END], [0, 0], "--", color=(0.5, 0.5, 0.5), lw=1)
h_line_err, = ax_err.plot([], [], color=(1.0, 0.4, 0.4), lw=1.5)

ax_thr = make_side_axes([0.52, 0.08, 0.43, 0.22], [-5, np.max(thrust) * 1.25], "Thrust", "N")
ax_thr.set_xlabel("Time [s]", color=COLOR_TXT, fontsize=9)
ax_thr.plot([0, T_END], [MASS * GRAVITY, MASS * GRAVITY], "--", color=(0.5, 0.5, 0.5), lw=1)
h_line_thr, = ax_thr.plot([], [], color=(0.25, 0.85, 0.55), lw=1.5)


# ---------------------------------------------------------------------------
# Draw drone helper
# ---------------------------------------------------------------------------
def draw_drone(z, u):
    bw, bh   = 3.5, 5.0
    arm_span = 8.5

    h_body.set_xy([[-bw, z-bh], [bw, z-bh], [bw, z+bh], [-bw, z+bh]])
    h_arm_l.set_data([-bw, -arm_span], [z, z])
    h_arm_r.set_data([ bw,  arm_span], [z, z])
    h_rotor_l.set_data([-arm_span-2, -arm_span+2], [z+bh, z+bh])
    h_rotor_r.set_data([ arm_span-2,  arm_span+2], [z+bh, z+bh])

    flame_len = min(abs(u) / 15.0, 1.0) * 20.0
    if u > 0.05:
        h_flame.set_data([0, 0], [z-bh, z-bh-flame_len])
        h_flame.set_color((0.3, 0.8, 1.0))
    elif u < -0.05:
        h_flame.set_data([0, 0], [z+bh, z+bh+flame_len])
        h_flame.set_color((1.0, 0.5, 0.3))
    else:
        h_flame.set_data([0, 0], [z-bh, z-bh])

    h_alt_lbl.set_position((10.0, z+3.0))
    h_alt_lbl.set_text(f"{z:.1f} m")


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------
SKIP = 2

for i in range(0, N, SKIP):
    if not plt.fignum_exists(fig.number):
        break

    draw_drone(altitude[i], thrust[i])
    h_time.set_text(f"t = {t[i]:.2f} s")
    h_err_lbl.set_text(f"error = {error[i]:.1f} m")

    h_line_alt.set_data(t[:i+1], altitude[:i+1])
    h_line_err.set_data(t[:i+1], error[:i+1])
    h_line_thr.set_data(t[:i+1], thrust[:i+1])

    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(DT * SKIP * 0.8)


# ---------------------------------------------------------------------------
# Post-simulation annotation
# ---------------------------------------------------------------------------
ss_alt = np.mean(altitude[-100:])
ss_err = TARGET_ALT - ss_alt

h_ss_line.set_data([-22, -6], [ss_alt, ss_alt])
h_ss_line.set_visible(True)
h_ss_text.set_position((-23, (ss_alt + TARGET_ALT) / 2))
h_ss_text.set_text(f"SS error\n= {ss_err:.1f} m")
h_ss_text.set_visible(True)

print("\n=== Stage 2: PD Controller ===")
print(f"  Kp                  = {Kp}")
print(f"  Kd                  = {Kd}")
print(f"  Steady-state alt    = {ss_alt:.1f} m")
print(f"  Steady-state error  = {ss_err:.1f} m  ({100*ss_err/TARGET_ALT:.1f}% of target)")

plt.ioff()
plt.show()