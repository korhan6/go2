"""El guardian: ordenes frescas -> Move; orden vencida -> StopMove; acostado/bloqueado/pausa no
caminan; combinaciones del PS4. Sin ROS: python3 v2/pruebas/prueba_guardian.py"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "ros_ws", "src", "go2_v2"))
from go2_v2.guardian_logica import (API_FREEWALK, API_MOVE, API_STANDDOWN, API_STANDUP,  # noqa: E402
                                    API_STOPMOVE, BOTON, Guardian)


def apis(lista):
    return [a for a, _ in lista]


def correr(g, t0, t1, dt=0.02):
    out = []
    t = t0
    while t < t1 - 1e-9:
        out += g.paso(t)
        t += dt
    return out


def main():
    g = Guardian(vx_max=1.0, vy_max=0.6, wz_max=1.5, vigencia=0.25)
    # 1) orden fresca -> Move a 50 Hz, recortada
    g.orden(0.0, 2.0, -0.3, 0.2)
    o = g.paso(0.0)
    assert apis(o) == [API_MOVE] and o[0][1] == {"x": 1.0, "y": -0.3, "z": 0.2}, o
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
    # levantar: StandUp, BalanceStand, FreeWalk y vuelve a aceptar ordenes
    g.levantar(6.0)
    o = correr(g, 6.0, 9.0)
    assert apis(o) == [API_STANDUP, 1002, API_FREEWALK], apis(o)
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
    # L1 + SHARE = bloquear; otra vez = desbloquear
    b = [0] * 13
    b[BOTON["l1"]] = 1
    g.joystick(10.0, b)
    b2 = list(b)
    b2[BOTON["share"]] = 1
    g.joystick(10.1, b2)
    assert g.bloqueado
    g.joystick(10.2, b)
    g.joystick(10.3, b2)
    assert not g.bloqueado
    # L1 + CRUZ = acostar (sostener no repite)
    b3 = list(b)
    b3[BOTON["cruz"]] = 1
    g.joystick(11.0, b3)
    assert g.acostado
    n = len(g.agenda)
    g.joystick(11.1, b3)
    assert len(g.agenda) == n
    # 6) velocidad minima (si se activa): 0.1 m/s -> 0.2; 0.03 (casi quieto) no se toca
    g2 = Guardian(v_min_caminar=0.2)
    g2.orden(0, 0.1, 0.0, 0.0)
    assert abs(g2.paso(0)[0][1]["x"] - 0.2) < 1e-9
    g2.orden(0.1, 0.03, 0.0, 0.0)
    assert abs(g2.paso(0.1)[0][1]["x"] - 0.03) < 1e-9
    print("OK guardian")


if __name__ == "__main__":
    main()
