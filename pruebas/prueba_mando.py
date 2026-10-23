"""Control PS4 SIN ROS: orden de botones (kernel / SDL / medido), hombre muerto, frenos y acciones.

    python3 pruebas/prueba_mando.py
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "go2_nav"))
from go2_nav.nucleo.mando import FRENO, LAYOUTS, Mando, elegir_layout   # noqa: E402


def joy(layout, n_botones, apretados=(), ejes=None, n_ejes=8):
    b = [0] * n_botones
    for nombre in apretados:
        b[LAYOUTS[layout]["botones"][nombre]] = 1
    e = [0.0] * n_ejes
    for k, v in (ejes or {}).items():
        e[LAYOUTS[layout]["ejes"][k]] = v
    return e, b


def main():
    # 1) el orden se elige por el numero de botones; el medido manda
    assert elegir_layout(13)[0] == "kernel"
    assert elegir_layout(16)[0] == "sdl"
    assert elegir_layout(11)[0] is None
    propio = {"botones": {"l1": 7}, "ejes": {"vx": 4}}
    assert elegir_layout(13, propio) == ("propio", propio)

    for lay, nb in (("kernel", 13), ("sdl", 16)):
        m = Mando(LAYOUTS[lay])
        # 2) sin L1 no manda nada (Nav2 sigue), aunque mueva el stick
        o, a = m.paso(*joy(lay, nb, ejes={"vx": 1.0}))
        assert o is None and a == [], (lay, o, a)
        # 3) con L1 avanza; R1 = rapido
        o, _ = m.paso(*joy(lay, nb, ("l1",), {"vx": 1.0}))
        assert abs(o[0] - 0.40) < 1e-9 and o[1] == 0.0, (lay, o)
        o, _ = m.paso(*joy(lay, nb, ("l1", "r1"), {"vx": 1.0}))
        assert abs(o[0] - 0.60) < 1e-9, (lay, o)
        # 4) al soltar L1: UN cero y despues nada
        o, _ = m.paso(*joy(lay, nb, (), {"vx": 1.0}))
        assert o == (0.0, 0.0, 0.0), (lay, o)
        o, _ = m.paso(*joy(lay, nb, (), {"vx": 1.0}))
        assert o is None
        # 5) CIRCULO para todo, sin L1, una sola vez por apretada
        _, a = m.paso(*joy(lay, nb, ("circulo",)))
        assert a == ["parar"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("circulo",)))
        assert a == [], "repetido mientras lo mantiene"
        m.paso(*joy(lay, nb))
        # 6) L1 + PS = bajar; PS solo no hace nada (no se baja una escalera por error)
        _, a = m.paso(*joy(lay, nb, ("ps",)))
        assert a == []
        m.paso(*joy(lay, nb))
        _, a = m.paso(*joy(lay, nb, ("l1",)))
        _, a = m.paso(*joy(lay, nb, ("l1", "ps")))
        assert a == ["bajar"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("l1", "options")))
        assert a == ["subir"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("l1", "triangulo")))
        assert a == ["levantar"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("l1", "cruz")))
        assert a == ["acostar"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("l1", "share")))
        assert a == ["bloqueo"], (lay, a)
        m.paso(*joy(lay, nb))
        _, a = m.paso(*joy(lay, nb, ("share",)))
        assert a == ["pausa_seguir"], (lay, a)
        _, a = m.paso(*joy(lay, nb, ("cruz",)))
        assert a == [], "CRUZ sin L1 no acuesta"
        m.paso(*joy(lay, nb))
        # 7) frena como 'go2 manejar': algo delante a menos de FRENO -> no avanza, pero gira
        o, _ = m.paso(*joy(lay, nb, ("l1",), {"vx": 1.0, "wz": 1.0}), holguras={"adelante": FRENO - 0.05})
        assert o[0] == 0.0 and o[2] > 0.5 and m.aviso, (lay, o)
        o, _ = m.paso(*joy(lay, nb, ("l1",), {"vx": -1.0}), holguras={"adelante": 0.05, "atras": 1.0})
        assert o[0] < 0, "retroceder si hay algo DELANTE"
        # 8) zona muerta del stick: un stick que no vuelve del todo a cero no mueve al perro
        o, _ = m.paso(*joy(lay, nb, ("l1",), {"vx": 0.06, "vy": -0.05, "wz": 0.08}))
        assert o == (0.0, 0.0, 0.0), (lay, o)
        print("   %s (%d botones): OK" % (lay, nb))
    # 9) signos medidos con 'go2 control' (un control con el eje invertido)
    m = Mando(LAYOUTS["sdl"], {"vx": -1.0})
    o, _ = m.paso(*joy("sdl", 16, ("l1",), {"vx": 1.0}))
    assert o[0] < 0
    print("LISTO")


if __name__ == "__main__":
    main()
