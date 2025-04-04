import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# === CONFIGURE AQUI ===
PORT = 'COM4'  # Substitua pela sua porta serial (ex: '/dev/ttyUSB0' no Linux)
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

scale = np.array([20, 30, 6]) / 100.0

vertices = np.array([
    [-1, -1, -1],
    [+1, -1, -1],
    [+1, +1, -1],
    [-1, +1, -1],
    [-1, -1, +1],
    [+1, -1, +1],
    [+1, +1, +1],
    [-1, +1, +1],
]) * 0.5 * scale


faces = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [0, 3, 7, 4]
]

def rotation_matrix(roll, pitch, yaw):
    # Conversão para radianos
    r, p, y = np.radians([roll, pitch, yaw])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(r), -np.sin(r)],
        [0, np.sin(r),  np.cos(r)]
    ])

    Ry = np.array([
        [ np.cos(p), 0, np.sin(p)],
        [0, 1, 0],
        [-np.sin(p), 0, np.cos(p)]
    ])

    Rz = np.array([
        [np.cos(y), -np.sin(y), 0],
        [np.sin(y),  np.cos(y), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx  # Ordem: ZYX

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def update(frame):
    ax.cla()
    try:
        line = ser.readline().decode().strip()
        print(f"[Serial] {line}")
        if not line.startswith("Roll"):
            return
        parts = line.replace("Roll:", "").replace("Pitch:", "").replace("Yaw:", "").split("|")
        roll = float(parts[0].strip())
        pitch = float(parts[1].strip())
        yaw = float(parts[2].strip())

        R = rotation_matrix(roll, pitch, yaw)
        rotated = vertices @ R.T

        for face in faces:
            square = rotated[face]
            ax.plot_trisurf(square[:, 0], square[:, 1], square[:, 2], color='cyan', alpha=0.5, edgecolor='k')

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_title(f"Roll: {roll:.1f} | Pitch: {pitch:.1f} | Yaw: {yaw:.1f}")
    except:
        pass

ani = FuncAnimation(fig, update, interval=50)
plt.show()
