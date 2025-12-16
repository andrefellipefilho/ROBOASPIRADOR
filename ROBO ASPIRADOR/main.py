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
    return f"{round(x / resolucao)},{round(y / resolucao)}"

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
        requests.post("http://127.0.0.1:1880/robo", json=dados, timeout=0.5)
    except:
        pass

# ===========================
#  CONFIGURAÇÃO DO CENÁRIO
# ===========================

MAP_SIZE = 6
SUJEIRA_QTD = 25
SUJEIRA_RAIO = 0.06
DIST_CAPTURA = 0.45   # <<<<<< ZONA DE ASPIRAÇÃO

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# ===========================
#        PAREDES
# ===========================

def criar_parede(pos, half_extents):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.7,0.7,0.7,1])
    p.createMultiBody(0, col, vis, pos)

s, h, e = MAP_SIZE, 0.4, 0.1
criar_parede([0,-s,h/2],[s,e,h/2])
criar_parede([0, s,h/2],[s,e,h/2])
criar_parede([-s,0,h/2],[e,s,h/2])
criar_parede([ s,0,h/2],[e,s,h/2])

# ===========================
#      OBSTÁCULOS FIXOS
# ===========================

def criar_obstaculo(pos, half_extents):
    col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=[0.4,0.4,0.4,1])
    p.createMultiBody(0, col, vis, pos)

criar_obstaculo([1.5, 1.5, 0.2], [0.6, 0.3, 0.2])
criar_obstaculo([-2, -1, 0.2], [0.4, 0.6, 0.2])
criar_obstaculo([0, -2, 0.2], [0.7, 0.3, 0.2])

# ===========================
#      SUJEIRAS SEQUENCIAIS
# ===========================

sujeiras = []
sujeiras_criadas = 0

def criar_sujeira():
    global sujeiras_criadas
    if sujeiras_criadas >= SUJEIRA_QTD:
        return
    for _ in range(50):
        x = random.uniform(-MAP_SIZE+0.5, MAP_SIZE-0.5)
        y = random.uniform(-MAP_SIZE+0.5, MAP_SIZE-0.5)
        cell = pos_to_cell(x, y)
        if mapa.get(cell) == "teve_sujeira" and random.random() < 0.8:
            continue
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=SUJEIRA_RAIO)
        vis = p.createVisualShape(p.GEOM_SPHERE, radius=SUJEIRA_RAIO, rgbaColor=[0.9,0.5,0.1,1])
        sujeira = p.createMultiBody(0.01, col, vis, [x,y,SUJEIRA_RAIO+0.01])
        sujeiras.append(sujeira)
        sujeiras_criadas += 1
        print(f"🟠 Sujeira criada {sujeiras_criadas}/{SUJEIRA_QTD}")
        break

# ===========================
#       ROBÔ ASPIRADOR
# ===========================

ROBO_RAIO = 0.35
ROBO_ALTURA = 0.15

robo_col = p.createCollisionShape(p.GEOM_CYLINDER, radius=ROBO_RAIO, height=ROBO_ALTURA)
robo_vis = p.createVisualShape(p.GEOM_CYLINDER, radius=ROBO_RAIO, length=ROBO_ALTURA, rgbaColor=[0.2,0.4,1,1])
robo = p.createMultiBody(1, robo_col, robo_vis, [0,0,ROBO_ALTURA/2])

velocidade = 80.0
direcao = random.uniform(0, 2*math.pi)
modo = "alvo"
direcao_desvio = 0
tempo_desvio = 0

# ===========================
#   SENSOR (IGNORA SUJEIRAS)
# ===========================

def ray_sensor(pos, direcao, offset=0.0, alcance=0.7):
    x,y,z = pos
    ang = direcao + offset
    tx = x + math.cos(ang)*alcance
    ty = y + math.sin(ang)*alcance
    hit = p.rayTest([x,y,z+0.05],[tx,ty,z+0.05])[0][0]
    if hit in sujeiras:
        return False
    return hit != -1

# ===========================
#   SUJEIRA MAIS PRÓXIMA
# ===========================

def encontrar_sujeira_mais_proxima(pos):
    if not sujeiras:
        return None
    rx, ry = pos[0], pos[1]
    return min(
        sujeiras,
        key=lambda s: math.hypot(
            rx - p.getBasePositionAndOrientation(s)[0][0],
            ry - p.getBasePositionAndOrientation(s)[0][1]
        )
    )

# ===========================
#      INICIALIZAÇÃO
# ===========================

for _ in range(10):
    p.stepSimulation()
    time.sleep(1/240)

criar_sujeira()
distancia_total = 0
tempo_inicio = time.time()
ult_pos = p.getBasePositionAndOrientation(robo)[0]

# ===========================
#        LOOP PRINCIPAL
# ===========================

try:
    while True:
        pos, orn = p.getBasePositionAndOrientation(robo)
        x,y = pos[0], pos[1]

        distancia_total += math.hypot(x-ult_pos[0], y-ult_pos[1])
        ult_pos = pos

        alvo = encontrar_sujeira_mais_proxima(pos)
        if alvo:
            sx, sy = p.getBasePositionAndOrientation(alvo)[0][:2]
            dist = math.hypot(sx-x, sy-y)
            direcao = math.atan2(sy-y, sx-x)

        if alvo and dist < DIST_CAPTURA:
            nx = x + math.cos(direcao)*velocidade*(1/240)
            ny = y + math.sin(direcao)*velocidade*(1/240)
            p.resetBasePositionAndOrientation(robo,[nx,ny,pos[2]],orn)
        else:
            frente = ray_sensor(pos, direcao)
            esq = ray_sensor(pos, direcao, math.pi/4)
            dirr = ray_sensor(pos, direcao, -math.pi/4)

            if modo == "alvo":
                if frente:
                    modo = "desvio"
                    tempo_desvio = time.time()
                    direcao_desvio = direcao + (math.pi/2 if not esq else -math.pi/2)
                else:
                    nx = x + math.cos(direcao)*velocidade*(1/240)
                    ny = y + math.sin(direcao)*velocidade*(1/240)
                    p.resetBasePositionAndOrientation(robo,[nx,ny,pos[2]],orn)
            else:
                nx = x + math.cos(direcao_desvio)*velocidade*(1/240)
                ny = y + math.sin(direcao_desvio)*velocidade*(1/240)
                p.resetBasePositionAndOrientation(robo,[nx,ny,pos[2]],orn)
                if time.time() - tempo_desvio > 0.5:
                    modo = "alvo"

        for sujeira in list(sujeiras):
            if p.getContactPoints(robo, sujeira):
                px,py,_ = p.getBasePositionAndOrientation(sujeira)[0]
                mapa[pos_to_cell(px,py)] = "teve_sujeira"
                p.removeBody(sujeira)
                sujeiras.remove(sujeira)
                print("🟠 Sujeira aspirada!")
                criar_sujeira()

        enviar_log_node_red({
            "x": x, "y": y, "modo": modo,
            "sujeiras_restantes": SUJEIRA_QTD - sujeiras_criadas + len(sujeiras),
            "distancia_total": distancia_total,
            "tempo": time.time()-tempo_inicio
        })

        if sujeiras_criadas >= SUJEIRA_QTD and not sujeiras:
            print("\n🎉 MAPA LIMPO!")
            break

        p.stepSimulation()
        time.sleep(1/240)

finally:
    save_map(mapa)
    p.disconnect()
