"""
Stage 1 — Proportional (P) Controller
======================================
A drone climbs from ground level to a 500m target altitude using only
a P controller with gravity feedforward.

What to observe:
  - The drone overshoots the target because P-only has no braking term
  - A small steady-state error remains (drag creates an equilibrium before the target)
  - The thrust spike at launch is large; it decays as the error shrinks

Physics model:  m * a = Thrust - m*g - c_drag * v
Control law:    u = Kp * e + m*g   (feedforward cancels gravity at hover)
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
MASS       = 1.0      # kg
GRAVITY    = 9.81     # m/s²
TARGET_ALT = 500.0    # m  — desired altitude
Kp         = 0.12     # proportional gain  (tuned for a strong initial climb)
C_DRAG     = 0.25     # aerodynamic drag coefficient

DT    = 0.05          # simulation time-step (s)
T_END = 60.0          # total run time (s)


# ---------------------------------------------------------------------------
# Pre-simulate the full trajectory before animating
# ---------------------------------------------------------------------------
t = np.arange(0, T_END + DT, DT)
N = len(t)

altitude = np.zeros(N)
velocity = np.zeros(N)
thrust   = np.zeros(N)
error    = np.zeros(N)

for i in range(1, N):
    error[i-1] = TARGET_ALT - altitude[i-1]

    # P control + gravity feedforward
    thrust[i-1] = Kp * error[i-1] + MASS * GRAVITY

    # Newton's second law
    accel = (thrust[i-1] - MASS * GRAVITY - C_DRAG * velocity[i-1]) / MASS

    # Euler integration
    velocity[i] = velocity[i-1] + accel * DT
    altitude[i] = altitude[i-1] + velocity[i] * DT

    # Floor — drone cannot go underground
    if altitude[i] < 0:
        altitude[i] = 0.0
        velocity[i] = 0.0

# Fill the last sample
error[-1]  = TARGET_ALT - altitude[-1]
thrust[-1] = thrust[-2]


# ---------------------------------------------------------------------------
# Figure / axes setup
# ---------------------------------------------------------------------------
BG_DARK  = (0.13, 0.13, 0.15)
BG_PANEL = (0.10, 0.12, 0.18)
COLOR_TXT  = (0.75, 0.75, 0.75)
COLOR_GRID = (0.28, 0.28, 0.28)

plt.ion()
fig = plt.figure(figsize=(10, 5.8), facecolor=BG_DARK)
fig.canvas.manager.set_window_title("Stage 1 — P Controller (500 m)")


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


# --- Left panel: animated drone scene ---
ax_scene = fig.add_axes([0.05, 0.08, 0.38, 0.85], facecolor=BG_PANEL)
ax_scene.tick_params(colors=COLOR_TXT)
ax_scene.set_xlim([-25, 25])
ax_scene.set_ylim([0, 650])
ax_scene.set_xticks([])
ax_scene.set_yticks(np.arange(0, 700, 50))
ax_scene.set_yticklabels([f"{v} m" for v in np.arange(0, 700, 50)], fontsize=9)
ax_scene.grid(color=COLOR_GRID, alpha=0.4)
ax_scene.set_ylabel("Altitude [m]", color=COLOR_TXT, fontsize=10)
ax_scene.set_title("Stage 1 — P Controller", color=(0.9, 0.9, 0.9), fontsize=11)

ax_scene.add_patch(
    patches.Rectangle((-25, 0), 50, 10, facecolor=(0.22, 0.18, 0.12), edgecolor="none")
)
ax_scene.plot([-25, 25], [TARGET_ALT, TARGET_ALT], "--", color=(0.9, 0.3, 0.3), lw=1.4)
ax_scene.text(2, TARGET_ALT + 10, "target", color=(0.9, 0.3, 0.3), fontsize=9)

# Drone geometry handles
h_body    = patches.Polygon(np.zeros((4, 2)), facecolor=(0.25, 0.55, 0.95), edgecolor="none")
ax_scene.add_patch(h_body)
h_arm_l,  = ax_scene.plot([], [], color=(0.5, 0.75, 1.0), lw=2.5)
h_arm_r,  = ax_scene.plot([], [], color=(0.5, 0.75, 1.0), lw=2.5)
h_rotor_l,= ax_scene.plot([], [], color=(0.8, 0.9, 1.0), lw=3)
h_rotor_r,= ax_scene.plot([], [], color=(0.8, 0.9, 1.0), lw=3)
h_flame,  = ax_scene.plot([], [], color=(0.3, 0.8, 1.0), lw=3)

h_alt_lbl = ax_scene.text(10.0, 0, "", color=(0.85, 0.85, 0.85), fontsize=9)
h_time    = ax_scene.text(-23, 580, "t = 0.00 s", color=(0.7, 0.7, 0.7), fontsize=9)
h_err_lbl = ax_scene.text(-23, 550, "error = 0.00 m", color=(1.0, 0.5, 0.5), fontsize=9)

h_ss_line, = ax_scene.plot([], [], "m-", lw=1.5, visible=False)
h_ss_text  = ax_scene.text(-23, 0, "", color=(1, 0.5, 1), fontsize=9, visible=False)

# --- Right panel: time-series charts ---
ax_alt = make_side_axes([0.52, 0.71, 0.43, 0.22], [0, 650], "Altitude", "m")
ax_alt.plot([0, T_END], [TARGET_ALT, TARGET_ALT], "--", color=(0.85, 0.3, 0.3), lw=1)
h_line_alt, = ax_alt.plot([], [], color=(0.3, 0.65, 1.0), lw=1.5)

ax_err = make_side_axes([0.52, 0.40, 0.43, 0.22], [-200, 550], "Error", "m")
ax_err.plot([0, T_END], [0, 0], "--", color=(0.5, 0.5, 0.5), lw=1)
h_line_err, = ax_err.plot([], [], color=(1.0, 0.4, 0.4), lw=1.5)

ax_thr = make_side_axes([0.52, 0.08, 0.43, 0.22], [-10, np.max(thrust) * 1.25], "Thrust", "N")
ax_thr.set_xlabel("Time [s]", color=COLOR_TXT, fontsize=9)
ax_thr.plot([0, T_END], [MASS * GRAVITY, MASS * GRAVITY], "--", color=(0.5, 0.5, 0.5), lw=1)
h_line_thr, = ax_thr.plot([], [], color=(0.25, 0.85, 0.55), lw=1.5)


# ---------------------------------------------------------------------------
# Helper — redraw drone body at a given altitude
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
# Animation loop
# ---------------------------------------------------------------------------
SKIP = 2   # render every 2nd sample to keep animation smooth

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
# Post-simulation: annotate steady-state error
# ---------------------------------------------------------------------------
ss_alt = np.mean(altitude[-100:])
ss_err = TARGET_ALT - ss_alt

h_ss_line.set_data([-22, -6], [ss_alt, ss_alt])
h_ss_line.set_visible(True)
h_ss_text.set_position((-23, (ss_alt + TARGET_ALT) / 2))
h_ss_text.set_text(f"SS error\n= {ss_err:.1f} m")
h_ss_text.set_visible(True)

print("\n=== Stage 1: P-Only Controller ===")
print(f"  Kp                  = {Kp}")
print(f"  Steady-state alt    = {ss_alt:.1f} m")
print(f"  Steady-state error  = {ss_err:.1f} m  ({100*ss_err/TARGET_ALT:.1f}% of target)")

plt.ioff()
plt.show()