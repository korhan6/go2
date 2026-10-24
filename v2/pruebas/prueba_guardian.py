"""El guardian: ordenes frescas -> Move; orden vencida -> StopMove; acostado/bloqueado/pausa no
caminan; combinaciones del PS4. Sin ROS: python3 v2/pruebas/prueba_guardian.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.guardian_logica import (API_FREEWALK, API_MOVE, API_RECOVERYSTAND,  # noqa: E402
                                    API_STANDDOWN, API_STANDUP, API_STOPMOVE, BOTON, Guardian)


def apis(lista):
    return [a for a, _ in lista]


def correr(g, t0, t1, dt=0.02, terreno=True):
    out = []
    t = t0
    while t < t1 - 1e-9:
        if terreno:
            g.terreno(t)               # el mapa de terreno llega normal
        out += g.paso(t)
        t += dt
    return out


def main():
    g = Guardian(vx_max=1.0, vy_max=0.6, wz_max=1.5, vigencia=0.25)
    # 0) sin mapa de terreno todavia: no camina
    g.orden(0.0, 0.5, 0.0, 0.0)
    assert API_MOVE not in apis(g.paso(0.0))
    # 1) orden fresca -> Move a 50 Hz, recortada
    g.terreno(0.0)
    g.orden(0.0, 2.0, -0.3, 0.2)
    o = g.paso(0.0)
    # 2.0 m/s > 1.0: se escala TODO a la mitad (la diagonal mantiene su direccion)
    assert apis(o) == [API_MOVE] and o[0][1] == {"x": 1.0, "y": -0.15, "z": 0.2}, o
    # 2) el planificador se calla: a los 0.25 s StopMove (y uno de respaldo), despues silencio
    o = correr(g, 0.02, 1.0)
    assert apis(o).count(API_MOVE) == 12, apis(o)            # 0.02 .. 0.24
    assert apis(o).count(API_STOPMOVE) == 2, apis(o)
    o = correr(g, 1.0, 3.0)
    assert o == [], o
    # 3) orden cero -> un StopMove (no spam)
    g.orden(3.0, 0.5, 0.0, 0.0)
    g.paso(3.0)
    g.orden(3.02, 0.0, 0.0, 0.0)
    o = g.paso(3.02) + g.paso(3.04) + correr(g, 3.06, 3.5)
    assert apis(o).count(API_STOPMOVE) == 2 and API_MOVE not in apis(o), apis(o)
    # 4) acostado: StopMove, a los 0.5 s StandDown, y despues ignora ordenes
    o = g.acostar(4.0)
    o += correr(g, 4.0, 5.0)
    assert API_STANDDOWN in apis(o) and API_STOPMOVE in apis(o)
    g.orden(5.0, 0.5, 0, 0)
    assert API_MOVE not in apis(correr(g, 5.0, 5.2))
    # levantar: StandUp, BalanceStand, FreeWalk SIN StopMove en el medio, y no camina mientras tanto
    o = g.levantar(6.0)
    t = 6.0
    while t < 9.5:
        g.terreno(t)
        g.orden(t, 0.4, 0.0, 0.0)             # el planificador ya tiene una orden lista
        o += g.paso(t)
        t += 0.02
    assert [a for a in apis(o) if a != API_MOVE] == [API_STANDUP, 1002, API_FREEWALK], apis(o)
    primer_move = [i for i, a in enumerate(apis(o)) if a == API_MOVE][0]
    assert apis(o).index(API_FREEWALK) < primer_move          # camina recien despues de pararse
    g.orden(9.0, 0.3, 0, 0)
    assert apis(g.paso(9.0)) == [API_MOVE]
    # 5) PS4: CIRCULO = pausa (corta), otra vez = sigue
    b = [0] * 13
    b[BOTON["circulo"]] = 1
    o = g.joystick(9.1, b)
    assert g.pausa and API_STOPMOVE in apis(o)
    g.orden(9.2, 0.3, 0, 0)
    assert API_MOVE not in apis(g.paso(9.2))
    g.joystick(9.3, [0] * 13)
    g.joystick(9.4, b)
    assert not g.pausa
    # SHARE + CUADRADO = bloquear; otra vez = desbloquear. L1 solo (borra el grafo de FAR) no hace nada aca
    b = [0] * 13
    b[BOTON["l1"]] = 1
    b[BOTON["cuadrado"]] = 1
    g.joystick(9.9, b)
    assert not g.bloqueado
    g.joystick(9.95, [0] * 13)
    b = [0] * 13
    b[BOTON["share"]] = 1
    g.joystick(10.0, b)
    b2 = list(b)
    b2[BOTON["cuadrado"]] = 1
    g.joystick(10.1, b2)
    assert g.bloqueado
    g.joystick(10.2, b)
    g.joystick(10.3, b2)
    assert not g.bloqueado
    # SHARE + CRUZ = acostar (sostener no repite)
    b3 = list(b)
    b3[BOTON["cruz"]] = 1
    g.joystick(11.0, b3)
    assert g.acostado
    n = len(g.agenda)
    g.joystick(11.1, b3)
    assert len(g.agenda) == n
    # SHARE + CIRCULO = recuperar (no es la pausa)
    g.joystick(11.5, b)
    b4 = list(b)
    b4[BOTON["circulo"]] = 1
    o = g.joystick(11.6, b4) + correr(g, 11.6, 16.0)
    assert API_RECOVERYSTAND in apis(o) and not g.pausa and not g.acostado
    # 6) odometria vieja (Point-LIO caido): para, avisa UNA vez, y sigue cuando vuelve
    g.joystick(16.0, [0] * 13)
    g.orden(16.0, 0.5, 0, 0, edad_odom=0.1)
    assert apis(g.paso(16.0)) == [API_MOVE]
    g.avisos = []
    o = []
    for k in range(10):
        g.terreno(16.02 + k * 0.02)
        g.orden(16.02 + k * 0.02, 0.5, 0, 0, edad_odom=1.0 + k * 0.02)
        o += g.paso(16.02 + k * 0.02)
    assert API_MOVE not in apis(o) and API_STOPMOVE in apis(o)
    assert sum(a.startswith("PARO") for a in g.avisos) == 1, g.avisos
    g.orden(16.3, 0.5, 0, 0, edad_odom=0.1)
    g.terreno(16.3)
    assert apis(g.paso(16.3)) == [API_MOVE]
    # ...pero en MANUAL (R2) si camina aunque la odometria este vieja (salida de emergencia)
    ejes = [0.0] * 8
    ejes[5] = -1.0
    g.joystick(16.4, [0] * 13, ejes)
    g.orden(16.4, 0.5, 0, 0, edad_odom=5.0)
    assert apis(g.paso(16.4)) == [API_MOVE]
    g.joystick(16.5, [0] * 13, [0.0] * 8)
    # 7) mapa de terreno que deja de llegar: para
    g.orden(16.6, 0.5, 0, 0, edad_odom=0.1)
    o = correr(g, 16.6, 19.0, terreno=False)
    assert API_STOPMOVE in apis(o)
    # 8) velocidad minima (si se activa): 0.1 m/s -> 0.2; 0.03 (casi quieto) no se toca
    g2 = Guardian(v_min_caminar=0.2)
    g2.terreno(0)
    g2.orden(0, 0.1, 0.0, 0.0)
    assert abs(g2.paso(0)[0][1]["x"] - 0.2) < 1e-9
    g2.orden(0.1, 0.03, 0.0, 0.0)
    assert abs(g2.paso(0.1)[0][1]["x"] - 0.03) < 1e-9
    print("OK guardian")


if __name__ == "__main__":
    main()
