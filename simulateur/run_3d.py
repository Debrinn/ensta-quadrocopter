import ctypes
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D 
from simulator import Quadcopter
import matplotlib.gridspec as gridspec

# Load C controller
lib = ctypes.CDLL("./controller.so")
lib.compute_control.argtypes = (
    np.ctypeslib.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
    np.ctypeslib.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
    np.ctypeslib.ndpointer(dtype=np.double, ndim=1, flags="C_CONTIGUOUS"),
)
lib.compute_control.restype = None

# Simulation setup
dt = 0.02
sim = Quadcopter(m=2.0, I_body=[0.02,0.02,0.04], dt=dt)
state = np.zeros(10)
cmd = np.zeros(3)         # vx, vy, vz
output = np.zeros(4)      # tau_x, tau_y, tau_z, thrust

# History buffers
times = []
vx_cmd_h, vy_cmd_h, vz_cmd_h = [], [], []
vx_h, vy_h, vz_h = [], [], []

# Create figure and grid layout
fig = plt.figure(figsize=(10, 8))
gs = gridspec.GridSpec(2, 3, height_ratios=[2, 1])

# 3D view subplot
ax3d = fig.add_subplot(gs[0, :], projection='3d')
ax3d.set_xlim(-5, 5); ax3d.set_ylim(-5, 5); ax3d.set_zlim(0, 5)
ax3d.set_xlabel('X'); ax3d.set_ylabel('Y'); ax3d.set_zlabel('Z')
scatter = ax3d.scatter([], [], [], s=50, color='cyan')

# Velocity plots subplots
axes = []
labels = ['X Velocity', 'Y Velocity', 'Z Velocity']
for i, lbl in enumerate(labels):
    ax = fig.add_subplot(gs[1, i])
    ax.set_title(lbl)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('m/s')
    ax.grid(True)
    axes.append(ax)
line_cmd = [ax.plot([], [], '--', label='cmd')[0] for ax in axes]
line_act = [ax.plot([], [], label='act')[0] for ax in axes]
for ax in axes: ax.legend()

# Keyboard control handlers
vel_step = 1.0

def on_key_press(event):
    if event.key == 'left':    cmd[0] = -vel_step
    elif event.key == 'right': cmd[0] = vel_step
    elif event.key == 'up':    cmd[1] = vel_step
    elif event.key == 'down':  cmd[1] = -vel_step
    elif event.key == 'w':     cmd[2] = vel_step
    elif event.key == 's':     cmd[2] = -vel_step

def on_key_release(event):
    if event.key in ('left', 'right'): cmd[0] = 0.0
    if event.key in ('up', 'down'):    cmd[1] = 0.0
    if event.key in ('w', 's'):        cmd[2] = 0.0

fig.canvas.mpl_connect('key_press_event',  on_key_press)
fig.canvas.mpl_connect('key_release_event', on_key_release)

# Animation update function
def update(frame):
    # pack state
    state[:3]    = sim.v
    state[3:7]   = sim.q
    state[7:10]  = sim.omega

    # prepare full cmd array for C controller
    cmd_full = np.zeros(6)
    cmd_full[:3] = cmd
    lib.compute_control(state, cmd_full, output)

    # step dynamics
    p, v, q, omega = sim.step(output[:3], output[3])

    # record history
    t = frame * dt
    times.append(t)
    vx_cmd_h.append(cmd[0]); vy_cmd_h.append(cmd[1]); vz_cmd_h.append(cmd[2])
    vx_h.append(v[0]); vy_h.append(v[1]); vz_h.append(v[2])

    # update 3D scatter
    scatter._offsets3d = ([p[0]], [p[1]], [p[2]])

    # update velocity plots
    for idx, (lc, la, cmd_buf, act_buf, ax) in enumerate(
            zip(line_cmd, line_act, (vx_cmd_h, vy_cmd_h, vz_cmd_h),
                                    (vx_h, vy_h, vz_h), axes)):
        lc.set_data(times, cmd_buf)
        la.set_data(times, act_buf)
        ax.set_xlim(0, t if t>0 else dt)
        ymin = min(min(cmd_buf), min(act_buf)) - 0.5
        ymax = max(max(cmd_buf), max(act_buf)) + 0.5
        ax.set_ylim(ymin, ymax)

    return (scatter, *line_cmd, *line_act)

ani = FuncAnimation(fig, update, interval=dt*1000, blit=False)
plt.tight_layout()
plt.show()