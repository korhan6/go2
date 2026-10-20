"""Pruebas del nucleo SIN ROS (solo numpy + scipy): deteccion y maniobra de escaleras contra el
perro simulado, y la exploracion por fronteras.

    python3 pruebas/prueba_nucleo.py            # todo (~2 min)
    python3 pruebas/prueba_nucleo.py bajar      # solo la bajada en reversa
    python3 pruebas/prueba_nucleo.py deteccion  # que ve el lidar segun la distancia al borde
    python3 pruebas/prueba_nucleo.py posible    # caida vista de lejos -> ir a mirarla de cerca

Lo que comprueba:
  - una BAJADA solo se confirma cuando el lidar ve el primer escalon (de lejos queda en su
    sombra), y un balcon (desnivel sin escalones) NUNCA se confirma;
  - de lejos queda como POSIBLE bajada; acercandose despacio, la escalera se confirma y el
    balcon no (automatico: el gestor manda al perro a mirarla);
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
        vistas[d] = "escalera" if det is not None and det["huella"] else ("posible" if det is not None else "no")
        print("   hocico a %.2f m: %-6s %s" % (d, centro, motivo))
    assert vistas[1.0] == vistas[0.7] == "posible", "de lejos tiene que verla como POSIBLE bajada (sin escalon)"
    assert vistas[0.5] == vistas[0.35] == "escalera", "con el hocico a 0.35-0.5 m del borde tiene que confirmarla"
    print("== un BALCON (desnivel sin escalones) nunca es escalera")
    for d in (0.5, 0.3, 0.2):
        det, centro, motivo = que_ve(balcon(), 11.2 + terreno.LARGO2 + d, math.pi)
        print("   hocico a %.2f m: %-6s %s" % (d, centro, motivo))
        assert det is None or not det["huella"], "un balcon se tomo por escalera!"
    print("   OK")


def inspeccion(mundo, x_borde, d_nose=1.0):
    """Lo que hace el sistema con una caida vista de lejos: el Detector la registra como POSIBLE
    (el gestor la vuelve portal 'por confirmar'), y al ir a verla go2_escaleras se acerca
    despacio (como escaleras_nodo.acercarse: hasta 25 cm del borde, max 0.8 m) buscando el
    escalon. Devuelve (posible_vista, bajada_confirmada, hocico_final_al_borde)."""
    r = M.RobotSim(mundo, x=x_borde + terreno.LARGO2 + d_nose, y=6.6, yaw=math.pi, ruido_real=True)
    cfg = dict(config.DEFECTO)
    det, mem = Detector(), Memoria()
    suelo, posible, x0 = None, None, r.x
    for k in range(300):
        r.dormir(0.1)
        ahora = r.t
        px, py, pz, yw, pitch, _r, _ = r.pose()
        for _t, xyz in r.nubes_nuevas():
            limpio = terreno.corregir_cerca(terreno.quitar_propio(xyz, px, py, pz, yw, suelo), px, py, yw, cfg["sesgo_cerca"])
            mem.sumar(limpio, px, py, ahora)
        local = mem.puntos()
        suelo = terreno.suelo_abs(local, px, py, pz, yw, previo=suelo)
        holg, _ = guardian(local, (px, py, pz, yw), suelo, cfg)
        det.actualizar(local, (px, py, pz, yw), suelo, ahora, alrededor=(k % 10 == 0))
        if posible is None:
            posible = det.confirmada("posible", ahora, (px, py))
            continue                                     # primero: verla de lejos, quieto
        if det.confirmada("bajar", ahora, (px, py)) is not None:
            return True, True, r.x - terreno.LARGO2 - x_borde
        d_ob, d_ca, d_hu = holg["adelante"]
        r.mover(0.07 if min(d_ob, d_ca, d_hu) > 0.25 and x0 - r.x < 0.8 else 0.0, 0.0, 0.0)
    return posible is not None, False, r.x - terreno.LARGO2 - x_borde


def prueba_posible():
    print("== bajada vista de LEJOS -> 'posible' -> ir a mirarla de cerca")
    for nombre, mundo, borde in (("escalera", M.casa_demo(), BORDE_BAJADA), ("balcon", balcon(), 11.2)):
        vista, confirmada, d = inspeccion(mundo, borde)
        print("   %-8s: posible=%s  confirmada=%s  (hocico a %.2f m del borde al terminar)" % (nombre, vista, confirmada, d))
        assert vista, "de lejos tiene que registrarla como posible bajada"
        assert confirmada == (nombre == "escalera"), "escalera: de cerca SI; balcon: NUNCA"
        assert d > 0.15, "nunca tan cerca del borde"
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


def prueba_paredes():
    print("== paredes del SLAM para Nav2: el ruido no es pared, una pared con huecos se cierra")
    from types import SimpleNamespace
    H = W = 60
    g = np.zeros((H, W), np.int8)
    g[30, 5:55:2] = 100            # pared vista "punteada" (un punto si y uno no)
    g[10, 10] = 100                # ruido suelto
    g[30, 23:37] = 0               # una puerta de 70 cm (14 celdas)
    mapa = SimpleNamespace(data=g.ravel().tolist(), info=SimpleNamespace(height=H, width=W, resolution=0.05,
                           origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))))
    pared = EX.paredes_slam(mapa)
    assert not pared[10, 10], "un puntito de ruido no puede ser pared"
    assert pared[30, 6:22].all(), "la pared punteada tiene que quedar continua"
    assert not pared[30, 25:35].any(), "la puerta tiene que seguir abierta"
    print("   OK")
    print("== huecos mas angostos que el perro (40 cm) se cierran; las puertas no")
    g = np.zeros((80, 80), np.int8)
    g[40, 5:30] = 100; g[40, 36:75] = 100          # pared con una rendija de 30 cm (6 celdas)
    g[60, 5:20] = 100; g[60, 34:75] = 100          # pared con una puerta de 70 cm (14 celdas)
    for c in (10, 15, 20):                         # patas de mesa a 25 cm (5 celdas) entre si
        g[20, c] = 100; g[21, c] = 100
    mapa = SimpleNamespace(data=g.ravel().tolist(), info=SimpleNamespace(height=80, width=80, resolution=0.05,
                           origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0))))
    pared = EX.paredes_slam(mapa, largo_min=2)
    assert pared[40, 30:36].all(), "la rendija de 30 cm tiene que cerrarse"
    assert not pared[60, 21:33].any(), "la puerta de 70 cm tiene que seguir abierta"
    assert pared[20, 10:21].all(), "entre patas de mesa a 25 cm no se pasa: tiene que quedar cerrado"
    assert not pared[30, :].any() and not pared[50, :].any(), "el cierre no puede crecer hacia los costados"
    print("   OK")
    print("== meta de respaldo (sin camino por suelo visto): rodea la pared, no la cruza")
    v = EX.Vista()
    # isla vista de 2 x 2 m alrededor del perro, pared al norte de 6 m, nada mas visto
    for x in np.arange(-0.95, 1.0, 0.1):
        for y in np.arange(-0.95, 1.0, 0.1):
            i, j = EX.celda(x, y)
            v.visto[i, j] = 3
    for x in np.arange(-3.05, 3.1, 0.1):
        i, j = EX.celda(x, 1.05)
        v.ocup[i, j] = 10
    pose = (0.0, 0.0, math.pi / 2)            # mirando a la pared
    v.pisado(pose)
    fr, _ = EX.fronteras(v, pose, None, [], [], 5, 0.5, 5.0, 0.7)   # holgura imposible -> respaldo
    print("   metas de respaldo: %s" % [(round(f["x"], 1), round(f["y"], 1), round(f["d"], 1), f.get("directa"))
                                       for f in fr[:3]])
    assert fr and all(f.get("directa") for f in fr)
    assert all(f["y"] < 1.0 for f in fr), "ninguna meta detras de la pared"
    print("   OK")


def prueba_scan():
    print("== /scan para el SLAM con un lidar ralo como el L1 (10 % de los puntos de las paredes)")
    m = M.casa_demo()
    r = M.RobotSim(m, x=3.0, y=3.0, yaw=0.0, ruido_real=False)
    r.dormir(0.2)
    xyz = np.concatenate([c for _t, c in r.nubes_nuevas()])
    pared = xyz[(xyz[:, 2] > 0.10) & (xyz[:, 2] < 0.55)]
    rng = np.random.default_rng(0)
    rala = pared[rng.random(len(pared)) < 0.10]
    antes = terreno.rangos_scan(rala, 3.0, 3.0, 0.0, n=720, r_max=10.0, hueco_max=0)
    ahora = terreno.rangos_scan(rala, 3.0, 3.0, 0.0)
    fa, fn = np.isfinite(antes).mean(), np.isfinite(ahora).mean()
    print("   haces con lectura: antes %.0f %%  ahora %.0f %%" % (100 * fa, 100 * fn))
    assert fn > fa + 0.15, "el /scan nuevo tiene que llenar bastantes mas haces"
    # rellenar no puede inventar pared en una puerta: dos bordes a distinta distancia no se unen
    rg = np.full(360, np.inf, np.float32)
    rg[10], rg[13] = 2.0, 4.0
    assert not np.isfinite(terreno._rellenar_huecos(rg, 3)[11:13]).any()
    print("   OK")


def prueba_alinear():
    print("== alinearse sin pasarse (perro que responde 0.3 s tarde, como el Go2)")
    from go2_nav.nucleo import escaleras as ES
    man = Maniobra(None, dict(config.DEFECTO), lambda t: None)

    def girar(meta, nuevo, retraso):
        """retraso en pasos de 0.1 s. Devuelve (veces que se paso y volvio, error final, segundos)."""
        man.m, man.w_ult = {}, 0.0
        yaw, w_real, cola, cambios, w_prev = 0.0, 0.0, [0.0] * retraso, 0, 0.0
        for k in range(400):
            t = k * 0.1
            err = ES.ang(meta - yaw)
            if nuevo:
                listo, cmd = man._girar_a({"ahora": t, "yaw": yaw}, meta, math.radians(3))
                w = cmd[2]
            else:                                # lo de antes: gira hasta estar a < 3 grados
                listo = abs(err) < math.radians(3)
                w = 0.0 if listo else man._girar(err)[2]
            man.w_ult = w
            if w * w_prev < 0:
                cambios += 1                     # gira hacia el otro lado: se habia pasado
            if w:
                w_prev = w
            cola.append(w)
            w_real += (cola.pop(0) - w_real) * 0.6     # retraso + inercia
            yaw += w_real * 0.1
            if listo and abs(w_real) < 0.02 and k > 5:
                break
        return cambios, math.degrees(abs(ES.ang(meta - yaw))), t
    for retraso in (2, 3, 5):
        for meta in (math.pi, math.radians(30)):
            ca, ea, ta = girar(meta, False, retraso)
            cn, en, tn = girar(meta, True, retraso)
            print("   retraso %.1f s, giro de %3.0f: antes se paso %2d veces (%2.0f s) | ahora %d veces, "
                  "error %.1f grados, %2.0f s" % (retraso / 10, math.degrees(meta), ca, ta, cn, en, tn))
            assert cn == 0 and en < 3.5, "no se puede pasar y volver"
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
    if que in ("todo", "posible"):
        prueba_posible()
    if que in ("todo", "exploracion"):
        prueba_exploracion()
        prueba_paredes()
    if que in ("todo", "scan"):
        prueba_scan()
    if que in ("todo", "alinear"):
        prueba_alinear()
    if que in ("todo", "maniobras"):
        prueba_maniobras()
    if que in ("bajar", "subir"):
        correr(que, verbose=True)
    print("LISTO")
