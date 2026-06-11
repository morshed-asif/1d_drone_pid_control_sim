"""
Stage 4 — Proportional-Integral-Derivative (PID) Controller
===========================================================
Adding an integral term allows the controller to eliminate steady-state
error caused by constant external disturbances (like wind). 

What to observe:
  - The drone fights a constant downward wind force (-3.0 N)
  - Proportional and Derivative terms alone leave a steady-state gap
  - The Integral state accumulates over time, driving the steady-state error to zero
  - A dedicated 4th telemetry graph tracks this accumulated error state

Physics model:  m * a = Thrust - m*g - c_drag * v + Disturbance
Control law:    u = Kp * e + Ki * ∫e dt + Kd * ė + m*g
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ---------------------------------------------------------------------------
# Simulation parameters
# ---------------------------------------------------------------------------
MASS        = 1.0
GRAVITY     = 9.81
TARGET_ALT  = 500.0
C_DRAG      = 0.25

# --- PID Controller Gains ---
Kp = 0.05
Kd = 0.05
Ki = 0.0008  # Integral gain to clear steady-state error

# --- Environment Disturbance ---
DISTURBANCE = -3.0  # [N] Constant downward wind force

DT    = 0.05
T_END = 80.0

# ---------------------------------------------------------------------------
# Pre-simulate
# ---------------------------------------------------------------------------
t = np.arange(0, T_END + DT, DT)
N = len(t)

altitude = np.zeros(N)
velocity = np.zeros(N)
thrust   = np.zeros(N)
error    = np.zeros(N)
int_e    = np.zeros(N)  # Array to track accumulated error over time

e_prev = TARGET_ALT - altitude[0]
cumulative_error = 0.0

for i in range(1, N):
    error[i-1] = TARGET_ALT - altitude[i-1]
    
    # 1. Proportional term
    P_out = Kp * error[i-1]
    
    # 2. Integral term (Accumulating error over time)
    cumulative_error += error[i-1] * DT
    int_e[i-1] = cumulative_error
    I_out = Ki * cumulative_error
    
    # 3. Derivative term (Rate of change of error)
    e_dot = (error[i-1] - e_prev) / DT
    D_out = Kd * e_dot
    
    # Control Law: PID-control + Gravity Feedforward
    thrust[i-1] = P_out + I_out + D_out + (MASS * GRAVITY)
    
    # Physics Plant: m*a = Thrust - Gravity - Drag + Disturbance
    accel = (thrust[i-1] - (MASS * GRAVITY) - (C_DRAG * velocity[i-1]) + DISTURBANCE) / MASS 
    
    # Euler Integration
    velocity[i] = velocity[i-1] + accel * DT
    altitude[i] = altitude[i-1] + velocity[i] * DT
    
    # Ground collision logic
    if altitude[i] < 0:
        altitude[i] = 0.0
        velocity[i] = 0.0
        cumulative_error = 0.0
        
    e_prev = error[i-1]

# Final step calculations
error[-1] = TARGET_ALT - altitude[-1]
int_e[-1] = int_e[-2]
thrust[-1] = thrust[-2]

# ---------------------------------------------------------------------------
# Figure setup (Clean dark theme)
# ---------------------------------------------------------------------------
BG_DARK    = (0.13, 0.13, 0.15)
BG_PANEL   = (0.10, 0.12, 0.18)
COLOR_TXT  = (0.75, 0.75, 0.75)
COLOR_GRID = (0.28, 0.28, 0.28)

plt.ion()  
fig = plt.figure(figsize=(12, 6.5), facecolor=BG_DARK)
fig.canvas.manager.set_window_title('Stage 3 — PID Control')


def make_side_axes(rect, ylim, title, ylabel):
    ax = fig.add_axes(rect, facecolor=BG_PANEL)
    ax.tick_params(colors=COLOR_TXT, labelsize=8)
    ax.grid(color=COLOR_GRID, alpha=0.5)
    ax.set_xlim([0, T_END])
    ax.set_ylim(ylim)
    ax.set_ylabel(ylabel, color=COLOR_TXT, fontsize=9)
    ax.set_title(title, color=(0.9, 0.9, 0.9), fontsize=9, pad=3)
    for spine in ax.spines.values():
        spine.set_edgecolor(COLOR_TXT)
    return ax


# Main scene axes (left side)
ax_scene = fig.add_axes([0.05, 0.08, 0.38, 0.85], facecolor=BG_PANEL)
ax_scene.tick_params(colors=COLOR_TXT)
ax_scene.set_xlim([-25, 25])
ax_scene.set_ylim([0, 650])
ax_scene.set_xticks([])
ax_scene.set_yticks(np.arange(0, 700, 50))
ax_scene.set_yticklabels([f"{val} m" for val in np.arange(0, 700, 50)], fontsize=9)
ax_scene.grid(color=COLOR_GRID, alpha=0.4)
ax_scene.set_ylabel('Altitude [m]', color=COLOR_TXT, fontsize=10)
ax_scene.set_title('Stage 3 — PID Control Simulation', color=(0.9, 0.9, 0.9), fontsize=11)

# Ground patch
ax_scene.add_patch(
    patches.Rectangle((-25, 0), 50, 10, facecolor=(0.22, 0.18, 0.12), edgecolor='none')
)

# Target line
ax_scene.plot([-25, 25], [TARGET_ALT, TARGET_ALT], '--', color=(0.9, 0.3, 0.3), lw=1.4)
ax_scene.text(2, TARGET_ALT + 10, 'target (500m)', color=(0.9, 0.3, 0.3), fontsize=9)

# Drone body parts
h_body    = patches.Polygon(np.zeros((4, 2)), facecolor=(0.25, 0.75, 0.45), edgecolor='none')
ax_scene.add_patch(h_body)
h_arm_l,   = ax_scene.plot([], [], color=(0.5, 0.85, 0.6), lw=2.5)
h_arm_r,   = ax_scene.plot([], [], color=(0.5, 0.85, 0.6), lw=2.5)
h_rotor_l, = ax_scene.plot([], [], color=(0.8, 0.95, 0.85), lw=3)
h_rotor_r, = ax_scene.plot([], [], color=(0.8, 0.95, 0.85), lw=3)
h_flame,   = ax_scene.plot([], [], color=(0.3, 1.0, 0.6), lw=3)

# Environment text indicators
ax_scene.text(-23, 50, f'🌬️ Constant Wind: {DISTURBANCE:.1f} N', color=(0.6, 0.8, 1.0), fontsize=9, fontweight='bold')

h_alt_lbl = ax_scene.text(10.0, 0, '', color=(0.85, 0.85, 0.85), fontsize=9)
h_time    = ax_scene.text(-23, 620, 't = 0.00 s', color=(0.7, 0.7, 0.7), fontsize=9)
h_err_lbl = ax_scene.text(-23, 590, 'error = 0.00 m', color=(1.0, 0.5, 0.5), fontsize=9)

# --- Side chart axes setup (4 Stacked Plots) ---
ax_alt  = make_side_axes([0.52, 0.73, 0.43, 0.16], [0, 650], '1. Altitude', 'm')
ax_alt.plot([0, T_END], [TARGET_ALT, TARGET_ALT], '--', color=(0.85, 0.3, 0.3), lw=1)
h_line_alt, = ax_alt.plot([], [], color=(0.3, 0.85, 0.55), lw=1.5)

ax_err  = make_side_axes([0.52, 0.51, 0.43, 0.16], [-100, 550], '2. Error', 'm')
ax_err.plot([0, T_END], [0, 0], '--', color=(0.5, 0.5, 0.5), lw=1)
h_line_err, = ax_err.plot([], [], color=(1.0, 0.4, 0.4), lw=1.5)

# 4th Graph: Accumulated Error (Integral Tracking)
max_inte_val = max(1.0, np.max(int_e) * 1.1)
ax_inte = make_side_axes([0.52, 0.29, 0.43, 0.16], [-500, max_inte_val], '3. Accumulated Error (Integral State)', 'm·s')
h_line_inte, = ax_inte.plot([], [], color=(0.8, 0.4, 1.0), lw=1.5)

ax_thr  = make_side_axes([0.52, 0.08, 0.43, 0.16], [-5, np.max(thrust) * 1.25], '4. Thrust Command', 'N')
ax_thr.set_xlabel('Time [s]', color=COLOR_TXT, fontsize=9)
ax_thr.plot([0, T_END], [MASS * GRAVITY, MASS * GRAVITY], '--', color=(0.5, 0.5, 0.5), lw=1)
h_line_thr, = ax_thr.plot([], [], color=(0.25, 0.85, 0.55), lw=1.5)


# ---------------------------------------------------------------------------
# Draw drone helper
# ---------------------------------------------------------------------------
def draw_drone(z, u):
    bw, bh   = 3.5, 5.0 
    arm_span = 8.5
    
    h_body.set_xy([[-bw, z-bh], [bw, z-bh], [bw, z+bh], [-bw, z+bh]])
    
    h_arm_l.set_data([-bw, -arm_span], [z, z])
    h_arm_r.set_data([bw, arm_span], [z, z])
    
    h_rotor_l.set_data([-arm_span-2.0, -arm_span+2.0], [z+bh, z+bh])
    h_rotor_r.set_data([arm_span-2.0, arm_span+2.0], [z+bh, z+bh])
    
    flame_len = min(abs(u) / 15.0, 1.0) * 20.0
    if u > 0.05:
        h_flame.set_data([0, 0], [z-bh, z-bh-flame_len])
        h_flame.set_color((0.3, 0.8, 1.0))
    else:
        h_flame.set_data([0, 0], [z-bh, z-bh])
        
    h_alt_lbl.set_position((10.0, z+3.0))
    h_alt_lbl.set_text(f"{z:.1f} m")


# ---------------------------------------------------------------------------
# Animation loop
# ---------------------------------------------------------------------------
SKIP = 3 
for i in range(0, N, SKIP):
    if not plt.fignum_exists(fig.number):
        break
        
    draw_drone(altitude[i], thrust[i])
    
    h_time.set_text(f"t = {t[i]:.2f} s")
    h_err_lbl.set_text(f"error = {error[i]:.1f} m")
    
    h_line_alt.set_data(t[:i+1], altitude[:i+1])
    h_line_err.set_data(t[:i+1], error[:i+1])
    h_line_inte.set_data(t[:i+1], int_e[:i+1])
    h_line_thr.set_data(t[:i+1], thrust[:i+1])
    
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(DT * SKIP * 0.4)


# ---------------------------------------------------------------------------
# Post-simulation metrics output
# ---------------------------------------------------------------------------
ss_alt = np.mean(altitude[-100:])
ss_err = TARGET_ALT - ss_alt

print('\n=== Stage 3: PID Controller Analysis ===')
print(f'  Kp                    = {Kp:.2f}')
print(f'  Ki (Integral Gain)    = {Ki:.4f}')
print(f'  Kd (Derivative Gain)  = {Kd:.2f}')
print(f'  Final Accumulated Error= {int_e[-1]:.1f} m·s')
print(f'  Steady-state alt      = {ss_alt:.1f} m')
print(f'  Steady-state error    = {ss_err:.1f} m')

plt.ioff()
plt.show()