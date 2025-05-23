import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Simulation parameters
dt = 0.1             # time step (s)
total_time = 20.0    # total duration (s)
times = np.arange(0, total_time + dt, dt)

# Intersection point at (0, 0)
intersection = np.array([0.0, 0.0])

# --- Ego vehicle: starts on negative X-axis, moves along +X at constant speed ---
v_ego = 5.0                           # m/s
ego_start = np.array([-100.0, 0.0])
ego_velocity = np.array([v_ego, 0.0])

# --- Adversary vehicle: starts on negative Y-axis, moves along +Y at constant speed ---
v_adv = 7.0                           # m/s
adv_start = np.array([0.0, -50.0])
adv_velocity = np.array([0.0, v_adv])

# Compute trajectories
ego_positions = ego_start + np.outer(times, ego_velocity)
adv_positions = adv_start + np.outer(times, adv_velocity)

ego_x, ego_y = ego_positions.T
adv_x, adv_y = adv_positions.T

# Prepare the plot
fig, ax = plt.subplots()
ax.set_aspect('equal')
margin = 10
xmin = min(ego_x.min(), adv_x.min()) - margin
xmax = max(ego_x.max(), adv_x.max()) + margin
ymin = min(ego_y.min(), adv_y.min()) - margin
ymax = max(ego_y.max(), adv_y.max()) + margin

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_title('Ego vs. Adversary Trajectories')
ax.grid(True)

ego_line, = ax.plot([], [], label='Ego Path')
ego_point, = ax.plot([], [], marker='o', linestyle='None')
adv_line, = ax.plot([], [], label='Adversary Path')
adv_point, = ax.plot([], [], marker='s', linestyle='None')
ax.legend()

def init():
    ego_line.set_data([], [])
    ego_point.set_data([], [])
    adv_line.set_data([], [])
    adv_point.set_data([], [])
    return ego_line, ego_point, adv_line, adv_point

def update(frame):
    # Update path lines
    ego_line.set_data(ego_x[:frame], ego_y[:frame])
    adv_line.set_data(adv_x[:frame], adv_y[:frame])
    # Update current points (wrap scalars in sequences)
    ego_point.set_data([ego_x[frame]], [ego_y[frame]])
    adv_point.set_data([adv_x[frame]], [adv_y[frame]])
    return ego_line, ego_point, adv_line, adv_point

ani = animation.FuncAnimation(
    fig, update, frames=len(times), init_func=init,
    blit=True, interval=dt * 1000
)

plt.show()

# Example of how you would package adversary messages for ROS:
adversary_messages = [
    {'time': float(t), 'x': float(adv_x[i]), 'y': float(adv_y[i]), 'velocity': float(v_adv)}
    for i, t in enumerate(times)
]

# Print first few for illustration
print("Sample adversary messages:")
for msg in adversary_messages[:5]:
    print(msg)
