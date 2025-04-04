import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp

# Parâmetros
g = -9.81
M = 1.0      # massa do corpo
l = 1.0      # distância do centro de massa à origem
Ixx = 0.1
Iyy = 0.1
Izz = 0.1
Iw = 0.01

# Torque nos volantes
def torques(t):
    return [0.0, 0.0, 0.0]  # torque constante nulo

# Equações diferenciais
def dynamics(t, state):
    theta_x, theta_y, theta_z, psi1, psi2, psi3, dtheta_x, dtheta_y, dtheta_z, dpsi1, dpsi2, dpsi3 = state
    tau1, tau2, tau3 = torques(t)

    ddtheta_x = (-M * g * l * np.cos(theta_y) * np.sin(theta_x)) / Ixx
    ddtheta_y = (-M * g * l * np.cos(theta_x) * np.sin(theta_y)) / Iyy
    ddtheta_z = 0  # sem torque externo
    ddpsi1 = tau1 / Iw
    ddpsi2 = tau2 / Iw
    ddpsi3 = tau3 / Iw

    return [dtheta_x, dtheta_y, dtheta_z, dpsi1, dpsi2, dpsi3,
            ddtheta_x, ddtheta_y, ddtheta_z, ddpsi1, ddpsi2, ddpsi3]

# Estado inicial: [theta_x, theta_y, theta_z, psi1, psi2, psi3, dtheta_x, dtheta_y, dtheta_z, dpsi1, dpsi2, dpsi3]
y0 = [0.5, 0.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
t_span = (0, 10)
t_eval = np.linspace(*t_span, 300)

sol = solve_ivp(dynamics, t_span, y0, t_eval=t_eval)

# Função para calcular a orientação do corpo
def get_body_position(theta_x, theta_y, theta_z):
    # Matriz de rotação composta
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(theta_x), -np.sin(theta_x)],
                   [0, np.sin(theta_x), np.cos(theta_x)]])
    Ry = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                   [0, 1, 0],
                   [-np.sin(theta_y), 0, np.cos(theta_y)]])
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0],
                   [np.sin(theta_z), np.cos(theta_z), 0],
                   [0, 0, 1]])

    R = Rz @ Ry @ Rx
    end = R @ np.array([0, 0, l])  # ponta do pêndulo no espaço
    return np.array([0, 0, 0]), end

# Animação
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], 'o-', lw=3)

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_zlim(-1.5, 0.5)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Pêndulo Invertido 3D")

def update(frame):
    theta_x = sol.y[0, frame]
    theta_y = sol.y[1, frame]
    theta_z = sol.y[2, frame]
    origin, end = get_body_position(theta_x, theta_y, theta_z)
    line.set_data([origin[0], end[0]], [origin[1], end[1]])
    line.set_3d_properties([origin[2], end[2]])
    return line,

ani = FuncAnimation(fig, update, frames=len(t_eval), blit=False)
plt.show()
