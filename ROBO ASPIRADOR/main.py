import pybullet as p
import pybullet_data
import time
import random
import math
import json
import os
import requests


# ===========================
#      MAPA PERSISTENTE
# ===========================

MAP_FILE = "mapa_memoria.json"

def pos_to_cell(x, y, resolucao=0.5):
    cx = round(x / resolucao)
    cy = round(y / resolucao)
    return f"{cx},{cy}"

def load_map():
    if os.path.exists(MAP_FILE):
        with open(MAP_FILE, "r") as f:
            return json.load(f)
    return {}

def save_map(mapa):
    with open(MAP_FILE, "w") as f:
        json.dump(mapa, f, indent=4)

mapa = load_map()


# ===========================
#   ENVIAR LOG PARA NODE-RED
# ===========================

def enviar_log_node_red(dados):
    try:
        requests.post(
            "http://127.0.0.1:1880/robo",
            json=dados,
            timeout=0.5
        )
    except:
        pass  # ignora erros caso Node-RED esteja fechado


# ===========================
#  CONFIGURAÇÃO DO CENÁRIO
# ===========================

MAP_SIZE = 6
SUJEIRA_QTD = 25
SUJEIRA_RAIO = 0.06

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf")

def criar_parede(pos, half_extents):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.7, 0.7, 0.7, 1])
    return p.createMultiBody(
        baseCollisionShapeIndex=col,
        baseVisualShapeIndex=vis,
        basePosition=pos
    )

s = MAP_SIZE
altura = 0.4
espessura = 0.1

criar_parede([0, -s, altura/2], [s, espessura, altura/2])
criar_parede([0,  s, altura/2], [s, espessura, altura/2])
criar_parede([-s, 0, altura/2], [espessura, s, altura/2])
criar_parede([ s, 0, altura/2], [espessura, s, altura/2])


# ===========================
#        SUJEIRAS
# ===========================

sujeiras = []

for i in range(SUJEIRA_QTD):
    for _ in range(50):
        x = random.uniform(-MAP_SIZE + 0.3, MAP_SIZE - 0.3)
        y = random.uniform(-MAP_SIZE + 0.3, MAP_SIZE - 0.3)
        z = SUJEIRA_RAIO + 0.01

        cell = pos_to_cell(x, y)

        # evitar spawn repetitivo
        if mapa.get(cell) == "teve_sujeira" and random.random() < 0.8:
            continue

        col = p.createCollisionShape(p.GEOM_SPHERE, radius=SUJEIRA_RAIO)
        vis = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=SUJEIRA_RAIO,
            rgbaColor=[0.8, 0.4, 0.1, 1]
        )
        sujeira = p.createMultiBody(
            baseMass=0.01,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[x, y, z]
        )
        sujeiras.append(sujeira)
        break

print("Sujeiras criadas:", len(sujeiras))


# ===========================
#       ROBÔ ASPIRADOR
# ===========================

ROBO_RAIO = 0.35
ROBO_ALTURA = 0.15

robo_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=ROBO_RAIO, height=ROBO_ALTURA)
robo_vis = p.createVisualShape(
    p.GEOM_CYLINDER,
    radius=ROBO_RAIO,
    length=ROBO_ALTURA,
    rgbaColor=[0.2, 0.4, 1, 1]
)

robo = p.createMultiBody(
    baseMass=1,
    baseCollisionShapeIndex=robo_col,
    baseVisualShapeIndex=robo_vis,
    basePosition=[0, 0, ROBO_ALTURA/2]
)

velocidade = 70.0
direcao = random.uniform(0, 2 * math.pi)

distancia_total = 0.0
tempo_inicio = time.time()
ult_pos = p.getBasePositionAndOrientation(robo)[0]


# ===========================
#   FUNÇÃO: SUJEIRA MAIS PRÓXIMA
# ===========================

def encontrar_sujeira_mais_proxima(robo_pos, sujeiras_list):
    if not sujeiras_list:
        return None

    rx, ry = robo_pos[0], robo_pos[1]
    menor = float("inf")
    alvo = None

    for sujeira in sujeiras_list:
        pos_s = p.getBasePositionAndOrientation(sujeira)[0]
        sx, sy = pos_s[0], pos_s[1]
        dist = math.hypot(rx - sx, ry - sy)
        if dist < menor:
            menor = dist
            alvo = (sujeira, sx, sy)

    return alvo


# ===========================
#      FIX CRÍTICO
# ===========================
# roda algumas iterações antes do loop para garantir física inicializada
for _ in range(10):
    p.stepSimulation()
    time.sleep(1./240.)


# ===========================
#   LOOP PRINCIPAL
# ===========================

try:
    while True:

        pos, orn = p.getBasePositionAndOrientation(robo)
        x, y = pos[0], pos[1]

        # distância percorrida
        dx = x - ult_pos[0]
        dy = y - ult_pos[1]
        distancia_total += math.hypot(dx, dy)
        ult_pos = pos

        # objetivo: sujeira mais próxima
        alvo = encontrar_sujeira_mais_proxima(pos, sujeiras)

        if alvo:
            sujeira_obj, sx, sy = alvo
            direcao = math.atan2(sy - y, sx - x)
        else:
            # aleatório se acabou sujeira
            if random.random() < 0.02:
                direcao += random.uniform(-math.pi/2, math.pi/2)

        dt = 1/240
        nx = x + math.cos(direcao) * velocidade * dt
        ny = y + math.sin(direcao) * velocidade * dt
        nova_pos = [nx, ny, pos[2]]

        p.resetBasePositionAndOrientation(robo, nova_pos, orn)

        # colisão com paredes
        limite = MAP_SIZE - ROBO_RAIO - 0.1
        if abs(nx) > limite or abs(ny) > limite:
            direcao += random.uniform(math.pi/2, math.pi)

        # verificar sujeira aspirada
        aspiradas = []
        for sujeira in list(sujeiras):
            if p.getContactPoints(robo, sujeira):
                aspiradas.append(sujeira)

        for sujeira in aspiradas:
            pos_s = p.getBasePositionAndOrientation(sujeira)[0]
            cell = pos_to_cell(pos_s[0], pos_s[1])
            mapa[cell] = "teve_sujeira"
            p.removeBody(sujeira)
            sujeiras.remove(sujeira)
            print("🟠 Sujeira aspirada!")

        # enviar dados ao Node-RED
        dados = {
            "x": x,
            "y": y,
            "z": pos[2],
            "sujeiras_restantes": len(sujeiras),
            "sujeiras_iniciais": SUJEIRA_QTD,
            "distancia_total": distancia_total,
            "tempo": time.time() - tempo_inicio,
            "estado": "limpando" if len(sujeiras) > 0 else "finalizado"
        }

        enviar_log_node_red(dados)

        if len(sujeiras) == 0:
            print("\n🎉 MAPA LIMPO! Salvando memória...")
            save_map(mapa)
            break

        p.stepSimulation()
        time.sleep(dt)

finally:
    save_map(mapa)
    time.sleep(1.0)
    p.disconnect()
