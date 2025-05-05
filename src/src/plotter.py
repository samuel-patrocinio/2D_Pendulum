import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from collections import defaultdict
import time

# === CONFIGURAÇÕES ===
PORT = 'COM3'       # Altere para sua porta serial
BAUD = 115200       # Altere se necessário
MAX_POINTS = 200    # Número máximo de pontos no gráfico

# === INICIALIZAÇÃO ===
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Aguarda a inicialização da porta serial

# === ARMAZENAMENTO DINÂMICO ===
data = defaultdict(list)

# === CONFIGURAÇÃO DO GRÁFICO ===
fig, ax = plt.subplots()

def update(frame):
    line = ser.readline().decode(errors='ignore').strip()
    #print(f"[Serial] {line}")

    # Extrai variáveis no formato "nome: valor"
    matches = re.findall(r'(\w+):\s*(-?\d+\.?\d*)', line)
    if not matches:
        return

    for var, val in matches:
        try:
            val = float(val)
            data[var].append(val)
            if len(data[var]) > MAX_POINTS:
                data[var] = data[var][-MAX_POINTS:]
        except ValueError:
            continue

    ax.clear()
    for var, values in data.items():
        ax.plot(values, label=var)

    ax.legend()
    ax.set_title("Leitura Serial em Tempo Real")
    ax.set_xlabel("Amostras")
    ax.set_ylabel("Valor")

ani = FuncAnimation(fig, update, interval=10)
plt.tight_layout()
plt.show()
