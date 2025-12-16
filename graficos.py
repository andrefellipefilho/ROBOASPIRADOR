import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

with open("resultados_simulacao.json", "r") as f:
    dados = json.load(f)

# ---------- ERRO MÉDIO DO EFETUADOR ----------
erro_ee = [d["erro_medio_ee"] for d in dados]

plt.figure()
plt.plot(erro_ee)
plt.xlabel("Ciclo")
plt.ylabel("Erro médio do efetuador (m)")
plt.title("Erro médio do efetuador por ciclo")
plt.grid(True)
plt.show()

# ---------- TEMPO DE ESTABILIZAÇÃO ----------
tempo_est = [
    d["tempo_estabilizacao"]
    for d in dados
    if d["tempo_estabilizacao"] is not None
]

plt.figure()
plt.plot(tempo_est)
plt.xlabel("Ciclo")
plt.ylabel("Tempo até estabilizar (s)")
plt.title("Tempo de estabilização por ciclo")
plt.grid(True)
plt.show()

# ---------- TRAJETÓRIA 3D ----------
traj = np.array(dados[-1]["trajetoria"])

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")
ax.plot(traj[:, 0], traj[:, 1], traj[:, 2])
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("Trajetória do efetuador (ciclo exemplo)")

plt.show()
