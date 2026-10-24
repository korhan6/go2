"""Mapa 3D SIN ROS: voxeles, filtro de vistas, colores y PCD ida y vuelta.

    python3 pruebas/prueba_mapa3d.py
"""
import os
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src", "go2_nav"))
from go2_nav.nucleo.mapa3d import Voxeles, centros, claves, colores, guardar_pcd, leer_pcd  # noqa: E402


def main():
    rng = np.random.default_rng(0)
    # 1) claves <-> centros (negativos incluidos)
    p = np.array([[0.01, -0.01, 1.23], [-5.0, 7.5, -0.3]])
    c = centros(claves(p, 0.05), 0.05)
    assert np.all(np.abs(c - p) <= 0.025 + 1e-6), c
    # 2) una pared vista en 5 nubes + ruido que aparece una sola vez
    pared = np.stack([np.full(2000, 2.0), rng.uniform(-2, 2, 2000), rng.uniform(0, 2, 2000)], axis=1)
    v = Voxeles(0.05)
    for _ in range(5):
        v.agregar(pared + rng.normal(0, 0.005, pared.shape))
        v.agregar(rng.uniform(-10, 10, (50, 3)))          # rebotes sueltos
        v.fusionar()
    pts = v.puntos(3)
    assert len(pts) > 500 and np.all(np.abs(pts[:, 0] - 2.0) < 0.1), "solo la pared queda"
    assert v.cambio
    # 3) muchos puntos en el mismo voxel de UNA nube cuentan 1
    w = Voxeles(0.05)
    w.agregar(np.zeros((100, 3)) + 0.01)
    w.fusionar()
    assert list(w.c) == [1]
    # 4) colores: bajo azul, alto rojo
    col = colores(np.array([0.0, 1.0, 2.0]), 0.0, 2.0)
    assert col[0, 2] > 200 and col[2, 0] > 200 and col[0, 0] < 60
    # 5) PCD ida y vuelta
    with tempfile.TemporaryDirectory() as d:
        r = os.path.join(d, "m.pcd")
        guardar_pcd(r, pts)
        q = leer_pcd(r)
        assert q.shape == pts.shape and np.allclose(q, pts)
        v2 = Voxeles(0.05)
        v2.cargar_puntos(q)
        assert len(v2.puntos(3)) == len(pts)
    # 6) rapido: una casa (500k voxeles) + 10 nubes de 10k puntos por fusion
    big = Voxeles(0.05)
    big.agregar(rng.uniform(-15, 15, (500000, 3)))
    big.fusionar()
    t = time.time()
    for _ in range(10):
        big.agregar(rng.uniform(-15, 15, (10000, 3)))
    big.fusionar()
    dt = time.time() - t
    assert dt < 1.0, dt
    print("   fusion de 10 nubes sobre %d voxeles: %.0f ms" % (len(big.k), dt * 1000))
    print("LISTO")


if __name__ == "__main__":
    main()
