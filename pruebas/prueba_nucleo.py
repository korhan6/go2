"""Pruebas del nucleo SIN ROS (solo numpy + scipy): deteccion y maniobra de escaleras contra el
perro simulado, y la exploracion por fronteras.

    python3 pruebas/prueba_nucleo.py            # todo (~2 min)
    python3 pruebas/prueba_nucleo.py bajar      # solo la bajada en reversa
    python3 pruebas/prueba_nucleo.py deteccion  # que ve el lidar segun la distancia al borde

Lo que comprueba:
  - una BAJADA solo se confirma cuando el lidar ve el primer escalon (de lejos queda en su
    sombra), y un balcon (desnivel sin escalones) NUNCA se confirma;
  - la bajada se hace SIEMPRE en reversa (gira 180 antes del borde) y la subida de frente;
  - la exploracion encuentra la puerta de un cuarto como frontera.
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "go2_nav"))
from go2_nav import mundo as M                                  # noqa: E402
from go2_nav.nucleo import config, exploracion as EX, terreno   # noqa: E402
from go2_nav.nucleo.control import Memoria, guardian            # noqa: E402
from go2_nav.nucleo.escaleras import Detector, Maniobra         # noqa: E402

BORDE_BAJADA = 10.92     # casa_demo: el entrepiso (z 1.53) termina en x = 10.92 (ultimo escalon)
PRIMER_ESCALON = 8.68    # casa_demo: la subida empieza en x = 8.68


class RobotMarcha(object):
    def __init__(self, r):
        self.r = r

    def modo_escaleras(self, activo):
        self.r.modo_escaleras(activo)
        return 0.8

    def balancear(self):
        pass


def balcon():
    """Entrepiso SIN escalera: un desnivel de 1.53 m a secas (nunca hay que bajarlo)."""
    m = M.Mundo(14.0, 10.0)
    m.rect_altura(11.2, 5.0, 14.0, 9.0, 1.53)
    m.rect_pared(11.2, 5.0, 14.0, 5.05)
    m.rect_pared(11.2, 9.0, 14.0, 9.05)
    return m


def que_ve(mundo, x, yaw, segundos=1.0):
    """Lo que el analizador de bajadas dice con el perro parado en (x, 6.6, yaw)."""
    r = M.RobotSim(mundo, x=x, y=6.6, yaw=yaw, ruido_real=True)
    mem = Memoria()
    r.dormir(segundos)
    for t, xyz in r.nubes_nuevas():
        px, py, pz, yw, _p, _r, _ = r.pose()
        limpio = terreno.corregir_cerca(terreno.quitar_propio(xyz, px, py, pz, yw, pz - 0.30), px, py, yw, (0.14, 0.19))
        mem.sumar(limpio, px, py, t)
    local = mem.puntos()
    px, py, pz, yw, _p, _r, _ = r.pose()
    suelo = terreno.suelo_abs(local, px, py, pz, yw)
    fx, fy = terreno.a_cuerpo(local, px, py, yw)
    return terreno.analizar_bajada(fx, fy, local[:, 2] - suelo)


def prueba_deteccion():
    print("== deteccion de la BAJADA segun la distancia del hocico al borde")
    vistas = {}
    for d in (1.0, 0.7, 0.5, 0.35, 0.2):
        det, centro, motivo = que_ve(M.casa_demo(), BORDE_BAJADA + terreno.LARGO2 + d, math.pi)
        vistas[d] = det is not None
        print("   hocico a %.2f m: %-6s %s" % (d, centro, motivo))
    assert not vistas[1.0], "a 1 m no deberia ver el primer escalon (sombra del lidar)"
    assert vistas[0.5] and vistas[0.35], "con el hocico a 0.35-0.5 m del borde tiene que confirmar la escalera"
    print("== un BALCON (desnivel sin escalones) nunca es escalera")
    for d in (0.5, 0.3, 0.2):
        det, centro, motivo = que_ve(balcon(), 11.2 + terreno.LARGO2 + d, math.pi)
        print("   hocico a %.2f m: %-6s %s" % (d, centro, motivo))
        assert det is None, "un balcon se tomo por escalera!"
    print("   OK")


def correr(tipo, d_nose=0.6, verbose=False):
    """La maniobra completa contra el simulador (como escaleras_nodo.lazo, a 10 Hz)."""
    m = M.casa_demo()
    if tipo == "bajar":
        x, yaw = BORDE_BAJADA + terreno.LARGO2 + d_nose, math.pi
    else:
        x, yaw = PRIMER_ESCALON - terreno.LARGO2 - d_nose, 0.0
    r = M.RobotSim(m, x=x, y=6.6, yaw=yaw, ruido_real=True)
    cfg = dict(config.DEFECTO)
    det, mem = Detector(), Memoria()
    man = Maniobra(RobotMarcha(r), cfg, (lambda t: print("   %6.1f  %s" % (r.t, t))) if verbose else (lambda t: None))
    suelo, pedido = None, False
    ordenes = {"BAJANDO": [], "SUBIENDO": []}
    for k in range(1200):
        r.dormir(0.1)
        ahora = r.t
        px, py, pz, yw, pitch, roll, _ = r.pose()
        nuevas = r.nubes_nuevas()
        for _t, xyz in nuevas:
            limpio = terreno.corregir_cerca(terreno.quitar_propio(xyz, px, py, pz, yw, suelo), px, py, yw, cfg["sesgo_cerca"])
            mem.sumar(limpio, px, py, ahora)
        if not nuevas:
            mem.sumar(np.zeros((0, 3), np.float32), px, py, ahora)
        local = mem.puntos()
        en_esc = man.estado in ("SUBIENDO", "BAJANDO", "SALIR", "DETENIDO")
        suelo = (pz - terreno.ALTURA_BASE) if en_esc else terreno.suelo_abs(local, px, py, pz, yw, previo=suelo)
        holg, _ = guardian(local, (px, py, pz, yw), suelo, cfg, rampa=abs(pitch) > math.radians(4))
        if nuevas and len(local):
            det.actualizar(local, (px, py, pz, yw), suelo, ahora, alrededor=(k % 10 == 0) and not en_esc)
        if not pedido:
            c = det.confirmada(tipo, ahora, (px, py))
            if c is not None and c.get("dist", 9) < 2.2:
                man.empezar(tipo, c, ahora)
                pedido = True
            elif ahora > 3.0:
                d_ob, d_ca, d_hu = holg["adelante"] if holg else (9, 9, 9)
                r.mover(0.07 if min(d_ob, d_ca, d_hu) > 0.25 and ahora < 25 else 0.0, 0.0, 0.0)
                assert ahora < 30, "no confirmo la escalera de %s: %s" % (tipo, det.motivo)
            continue
        if man.activa:
            vx, vy, wz = man.paso(ahora, (px, py, pz, yw, pitch, roll), det, holg)
            if man.estado in ordenes:
                ordenes[man.estado].append(vx)
            r.mover(vx, vy, wz)
            if man.resultado is not None:
                break
        else:
            raise AssertionError("DETENIDO en la escalera: %s" % man.m.get("motivo_det"))
    res = man.resultado
    print("   %s: %s | z %.2f -> %.2f | choques %d | %.0f s" % (tipo, res and res["motivo"], 0.3 if tipo == "subir" else 1.83,
                                                               r.z, r.choques, r.t))
    assert res and res["ok"], res
    assert r.choques == 0
    if tipo == "bajar":
        assert all(v <= 0 for v in ordenes["BAJANDO"]) and ordenes["BAJANDO"], "la bajada tiene que ser en REVERSA"
        assert abs(r.z - 0.30) < 0.05
    else:
        assert all(v >= 0 for v in ordenes["SUBIENDO"]) and ordenes["SUBIENDO"], "la subida tiene que ser de frente"
        assert abs(r.z - 1.83) < 0.05
    return res


def prueba_maniobras(cuales=("bajar", "subir")):
    print("== maniobras completas en el simulador (bajar = 180 y en reversa; subir = de frente)")
    for tipo in cuales:
        for d in (0.6, 1.2):
            correr(tipo, d)
    print("   OK")


def prueba_exploracion():
    print("== exploracion: un cuarto de 4 x 4 con una puerta de 0.8 m")
    v = EX.Vista()
    for x in np.arange(0.05, 4.0, 0.1):
        for y in np.arange(0.05, 4.0, 0.1):
            i, j = EX.celda(x, y)
            v.visto[i, j] = 3
    for y in np.arange(-0.05, 4.1, 0.1):
        for xx in (-0.05, 4.05):
            if xx > 0 and 1.6 <= y <= 2.4:
                continue                    # la puerta
            i, j = EX.celda(xx, y)
            v.ocup[i, j] = 10
    for x in np.arange(-0.05, 4.1, 0.1):
        for yy in (-0.05, 4.05):
            i, j = EX.celda(x, yy)
            v.ocup[i, j] = 10
    pose = (2.0, 2.0, 0.0)
    v.pisado(pose)
    fr, celdas = EX.fronteras(v, pose, None, [], [], 5, 0.5, 0.22, 0.7)
    print("   fronteras: %s" % [(round(f["x"], 1), round(f["y"], 1), f["n"]) for f in fr])
    assert fr and abs(fr[0]["x"] - 4.0) < 0.2 and abs(fr[0]["y"] - 2.0) < 0.5, "la puerta tiene que ser la frontera"
    assert v.zona_vista(2.0, 2.0) and not v.zona_vista(6.0, 2.0)
    assert abs(v.area() - 16.0) < 1.0
    print("   OK")


if __name__ == "__main__":
    que = sys.argv[1] if len(sys.argv) > 1 else "todo"
    if que in ("todo", "deteccion"):
        prueba_deteccion()
    if que in ("todo", "exploracion"):
        prueba_exploracion()
    if que in ("todo", "maniobras"):
        prueba_maniobras()
    if que in ("bajar", "subir"):
        correr(que, verbose=True)
    print("LISTO")
