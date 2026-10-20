"""Obstaculos confirmados con varias nubes, para un lidar RALO como el L1 del Go2.

El L1 da ~2000 puntos por nube repartidos en media esfera: un marco de puerta de 5-10 cm
recibe 5-7 puntos por nube, cada vez en otro lugar. El filtro anterior ("el mismo voxel de
12 cm en 2 de las ultimas 3 nubes") dejaba pasar solo la mitad, y en el mapa de costos el
marco aparecia y desaparecia: Nav2 veia un hueco y el perro se trababa contra el marco
(reproducido con Nav2 real y lidar ralo: pruebas/prueba_puerta.py).

Ahora: los obstaculos de las ultimas N nubes se juntan en voxeles (marco odom, fijo al mundo:
juntar nubes no se corre aunque el perro camine). Un voxel es obstaculo si se vio en al menos
'min_nubes' nubes DISTINTAS de la ventana; se publica aunque en la nube actual no le haya
tocado ningun punto. El ruido de una sola nube (un rebote, un punto suelto) sigue fuera.
"""
import collections

import numpy as np

_OFF = 1 << 20


class VoxelesRecientes(object):
    def __init__(self, vox=0.10, nubes=6, min_nubes=2):
        self.vox, self.min_nubes = vox, min_nubes
        self.hist = collections.deque(maxlen=nubes)     # (claves unicas, un punto por clave)

    def _claves(self, xyz):
        k = np.floor(xyz / self.vox).astype(np.int64) + _OFF
        return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]

    def limpiar(self):
        self.hist.clear()

    def sumar(self, xyz):
        """Agrega los candidatos a obstaculo de una nube (Nx3, odom).

        Devuelve (obstaculos Mx3: un punto por voxel confirmado en la ventana,
                  confirmado: bool por punto de xyz, False = ruido de esta nube)."""
        xyz = np.asarray(xyz, np.float32).reshape(-1, 3)
        claves = self._claves(xyz) if len(xyz) else np.zeros(0, np.int64)
        uk, idx = np.unique(claves, return_index=True)
        self.hist.append((uk, xyz[idx]))
        todas = np.concatenate([h[0] for h in self.hist])
        pts = np.concatenate([h[1] for h in self.hist])
        if len(todas) == 0:
            return np.zeros((0, 3), np.float32), np.zeros(0, bool)
        u, inv, cnt = np.unique(todas, return_inverse=True, return_counts=True)
        ok = cnt >= self.min_nubes                     # vista en >= min_nubes nubes distintas
        ultimo = np.full(len(u), -1, np.int64)
        np.maximum.at(ultimo, inv, np.arange(len(todas)))   # el punto mas nuevo de cada voxel
        obst = pts[ultimo[ok]]
        confirmado = ok[np.searchsorted(u, claves)] if len(claves) else np.zeros(0, bool)
        return obst, confirmado
