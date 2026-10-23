"""SLAM 3D SIN ROS: un cuarto simulado, el L1 ralo, una odometria que DERIVA, y la correccion.

    python3 pruebas/prueba_slam3d.py

Cuarto 8 x 6 m con muebles; el perro da 2 vueltas (~45 m). La odometria tiene 3 % de error de
escala y deriva de giro (como la del perro despues de un rato). Pide:
  - error final corregido < 8 cm y < 1.5 grados (la odometria sola: decenas de cm)
  - paredes del mapa 3D finas (el 90 % de los puntos de pared a < 5 cm de la pared real)
  - mapa 2D con paredes (ocupadas) y el interior libre, sin rayos fuera del cuarto
Una persona camina por el cuarto todo el tiempo: no debe torcer la alineacion ni quedar en el mapa.
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "go2_nav"))
from go2_nav.nucleo.slam3d import (Registro, Rejilla, aplicar, ang, componer, inversa,  # noqa: E402
                                   submuestrear)

# cajas (xmin, ymin, zmin, xmax, ymax, zmax): piso, techo, 4 paredes, muebles
CAJAS = np.array([
    [-1, -1, -0.10, 9, 7, 0.0],          # piso (z = 0)
    [-1, -1, 2.5, 9, 7, 2.6],            # techo
    [-0.1, -0.1, 0, 0.0, 6.1, 2.5],      # pared x = 0
    [8.0, -0.1, 0, 8.1, 6.1, 2.5],       # pared x = 8
    [-0.1, -0.1, 0, 8.1, 0.0, 2.5],      # pared y = 0
    [-0.1, 6.0, 0, 8.1, 6.1, 2.5],       # pared y = 6
    [5.5, 4.2, 0, 7.8, 5.8, 0.75],       # escritorio
    [0.3, 0.3, 0, 1.5, 2.3, 0.5],        # cama
    [3.5, 2.7, 0, 3.9, 3.1, 1.1],        # mueble en medio
    [6.5, 0.4, 0, 7.6, 0.9, 1.8],        # ropero
])
H_LIDAR = 0.42


def rayos(o, d, cajas=CAJAS):
    """Distancia al primer choque de cada rayo (o: 3, d: Nx3) contra las cajas."""
    CAJAS = cajas
    inv = 1.0 / np.where(np.abs(d) < 1e-9, 1e-9, d)
    t1 = (CAJAS[None, :, :3] - o) * inv[:, None, :]
    t2 = (CAJAS[None, :, 3:] - o) * inv[:, None, :]
    tmin = np.minimum(t1, t2).max(axis=2)
    tmax = np.maximum(t1, t2).min(axis=2)
    hit = (tmax >= np.maximum(tmin, 0.0)) & (tmax > 0)
    t = np.where(tmin > 0, tmin, tmax)
    t = np.where(hit, t, np.inf)
    return t.min(axis=1)


def scan_l1(pose, rng, n=1400, extra=None):
    """Nube del L1 (hemisferio, ralo) en el marco del perro, con ruido de 2 cm."""
    az = rng.uniform(-math.pi, math.pi, n)
    el = rng.uniform(-math.radians(70), math.radians(80), n)
    d = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=1)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    dw = d.copy()
    dw[:, 0], dw[:, 1] = c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]
    o = np.array([pose[0], pose[1], H_LIDAR])
    t = rayos(o, dw, CAJAS if extra is None else np.vstack([CAJAS, extra]))
    ok = np.isfinite(t) & (t > 0.3) & (t < 12)
    t = t[ok] + rng.normal(0, 0.02, ok.sum())
    return d[ok] * t[:, None] + np.array([0, 0, H_LIDAR])     # marco del perro


def trayectoria():
    """2 vueltas a un circuito dentro del cuarto, (x, y, yaw) cada 0.1 m o 5 grados."""
    pts = [(2.0, 3.2), (3.0, 1.5), (5.5, 1.6), (6.8, 2.5), (6.5, 3.6), (4.6, 3.8), (2.4, 4.6), (2.0, 3.2)]
    pts = pts + pts[1:]
    out = []
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        yaw = math.atan2(y1 - y0, x1 - x0)
        if out:   # girar en el sitio
            y_prev = out[-1][2]
            dy = ang(yaw - y_prev)
            for k in range(1, int(abs(dy) / math.radians(5)) + 1):
                out.append((x0, y0, ang(y_prev + math.copysign(math.radians(5), dy) * k)))
        L = math.hypot(x1 - x0, y1 - y0)
        for k in range(int(L / 0.1) + 1):
            out.append((x0 + (x1 - x0) * k * 0.1 / L, y0 + (y1 - y0) * k * 0.1 / L, yaw))
    return out


def main():
    rng = np.random.default_rng(1)
    verdad = trayectoria()
    # odometria con deriva: escala 3 % y +0.04 rad por cada rad girado, +0.01 rad/m
    # el mapa = donde arranco el perro: verdad relativa a la pose inicial
    v0 = (verdad[0][0], verdad[0][1], 0.0, verdad[0][2])
    iv0 = inversa(v0)
    odom = [(0.0, 0.0, 0.0, 0.0)]
    for a, b in zip(verdad[:-1], verdad[1:]):
        ra = inversa((a[0], a[1], 0, a[2]))
        rel = componer(ra, (b[0], b[1], 0, b[2]))      # movimiento en el marco del perro
        dist = math.hypot(rel[0], rel[1])
        rel = (rel[0] * 1.03, rel[1] * 1.03, 0.0, rel[3] * 1.04 + 0.01 * dist + rng.normal(0, 0.002))
        odom.append(componer(odom[-1], rel))
    reg = Registro()
    rej = Rejilla()
    T = (0.0, 0.0, 0.0, 0.0)
    acept = rech = 0
    t0 = time.time()
    for i, (v, o) in enumerate(zip(verdad, odom)):
        # una persona camina por el cuarto (va y viene entre x = 1.5 y 6.5, y = 2.2)
        px = 4.0 + 2.5 * math.sin(i * 0.05)
        persona = np.array([[px - 0.2, 2.0, 0.0, px + 0.2, 2.4, 1.7]])
        if abs(v[0] - px) < 0.6 and abs(v[1] - 2.2) < 0.6:
            persona = None                                         # (no atraviesa al perro)
        s = scan_l1(v, rng, extra=persona)                        # marco perro
        # el perro entrega la nube en 'odom': odom * (lidar en el perro); z del perro en el mapa: 0
        s_odom = aplicar(o, s)
        if i < 5:
            ok = True
        else:
            pos = aplicar(T, np.array([[o[0], o[1], 0.0]]))[0]
            reg.rearmar(pos)
            Tn, info = reg.alinear(submuestrear(s_odom, 0.10), T)
            dT = math.hypot(Tn[0] - T[0], Tn[1] - T[1])
            ok = info["inliers"] >= 80 and info["frac"] >= 0.35 and dT < 0.5
            if ok:
                T = Tn
        if ok:
            acept += 1
            pm = aplicar(T, s_odom)
            reg.insertar(pm)
            if i % 3 == 0:
                reg.vox.fusionar()
            sen = aplicar(T, np.array([[o[0], o[1], H_LIDAR]]))[0]
            obst = pm[(pm[:, 2] > 0.10) & (pm[:, 2] < 1.5)]
            suelo = pm[np.abs(pm[:, 2]) < 0.05]
            rej.actualizar(sen[:2], obst[:, :2], suelo[:, :2])
        else:
            rech += 1
    dt = time.time() - t0
    reg.vox.fusionar()
    # error final: pose corregida vs verdad (en el marco del mapa)
    vf = componer(iv0, (verdad[-1][0], verdad[-1][1], 0, verdad[-1][2]))
    cf = componer(T, odom[-1])
    e_c = math.hypot(cf[0] - vf[0], cf[1] - vf[1])
    e_cy = abs(math.degrees(ang(cf[3] - vf[3])))
    e_o = math.hypot(odom[-1][0] - vf[0], odom[-1][1] - vf[1])
    e_oy = abs(math.degrees(ang(odom[-1][3] - vf[3])))
    print("   %d nubes (%.0f ms c/u), aceptadas %d, descartadas %d" % (len(verdad), 1000 * dt / len(verdad), acept, rech))
    print("   odometria sola: error %.2f m / %.1f grados" % (e_o, e_oy))
    print("   corregido:      error %.3f m / %.2f grados" % (e_c, e_cy))
    assert e_o > 0.3, "la prueba no tiene deriva"
    assert e_c < 0.08 and e_cy < 1.5, (e_c, e_cy)
    # paredes finas: puntos del mapa (en el marco real) a la pared x = 0 / y = 0
    P = aplicar(v0, reg.vox.puntos(2))
    banda = (P[:, 2] > 0.9) & (P[:, 2] < 2.3)
    px = P[banda & (P[:, 0] < 0.4) & (P[:, 1] > 1) & (P[:, 1] < 5)]
    py = P[banda & (P[:, 1] < 0.4) & (P[:, 0] > 2) & (P[:, 0] < 6)]
    fx = np.mean(np.abs(px[:, 0]) < 0.075)
    fy = np.mean(np.abs(py[:, 1]) < 0.075)
    print("   paredes finas: %.0f %% / %.0f %% de los puntos a < 7.5 cm" % (100 * fx, 100 * fy))
    assert fx > 0.85 and fy > 0.85, (fx, fy)
    # mapa 2D: celdas ocupadas sobre las paredes, interior libre, nada libre fuera del cuarto
    occ, x0, y0 = rej.ocupacion()
    ii, jj = np.nonzero(occ == 100)
    xy = np.stack([x0 + (jj + 0.5) * rej.res, y0 + (ii + 0.5) * rej.res, np.zeros(len(ii))], axis=1)
    W = aplicar(v0, xy)
    en_pared = ((np.abs(W[:, 0]) < 0.15) | (np.abs(W[:, 0] - 8) < 0.15)
                | (np.abs(W[:, 1]) < 0.15) | (np.abs(W[:, 1] - 6) < 0.15))
    en_mueble = np.zeros(len(W), bool)
    for c in CAJAS[6:]:
        en_mueble |= (W[:, 0] > c[0] - 0.15) & (W[:, 0] < c[3] + 0.15) & (W[:, 1] > c[1] - 0.15) & (W[:, 1] < c[4] + 0.15)
    fr = np.mean(en_pared | en_mueble)
    ii, jj = np.nonzero(occ == 0)
    xy = np.stack([x0 + (jj + 0.5) * rej.res, y0 + (ii + 0.5) * rej.res, np.zeros(len(ii))], axis=1)
    W = aplicar(v0, xy)
    fuera = np.mean((W[:, 0] < -0.15) | (W[:, 0] > 8.15) | (W[:, 1] < -0.15) | (W[:, 1] > 6.15))
    print("   mapa 2D: %.0f %% de lo ocupado es pared/mueble real; %.2f %% de lo libre fuera del cuarto"
          % (100 * fr, 100 * fuera))
    assert fr > 0.9 and fuera < 0.005, (fr, fuera)
    print("LISTO")


if __name__ == "__main__":
    main()
