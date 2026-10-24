"""Mapa 3D denso: acumula las nubes del lidar en voxeles (5 cm) en el marco 'map'. Logica pura (numpy).

- cada voxel cuenta en CUANTAS nubes distintas aparecio: un punto suelto (rebote, alguien que paso)
  no llega a 'min_vistas' y no se dibuja; una pared o un mueble si.
- color por ALTURA (como los mapas 3D de las fotos): suelo azul -> verde -> amarillo -> rojo el techo.
- se guarda / carga como PCD binario (x y z rgb): lo abren RViz, CloudCompare, pcl_viewer, Open3D.
"""
import numpy as np

_B = 21                       # bits por eje: +-1 048 576 voxeles = +-52 km a 5 cm
_OFF = 1 << (_B - 1)
_MASK = (1 << _B) - 1


def claves(xyz, res):
    i = np.floor(xyz / res).astype(np.int64) + _OFF
    i = np.clip(i, 0, _MASK)
    return (i[:, 0] << (2 * _B)) | (i[:, 1] << _B) | i[:, 2]


def centros(k, res):
    k = np.asarray(k, np.int64)
    i = np.stack([(k >> (2 * _B)) & _MASK, (k >> _B) & _MASK, k & _MASK], axis=1) - _OFF
    return ((i.astype(np.float64) + 0.5) * res).astype(np.float32)


class Voxeles(object):
    def __init__(self, res=0.05, max_voxeles=3_000_000):
        self.res = float(res)
        self.max = int(max_voxeles)
        self.k = np.zeros(0, np.int64)        # ordenadas
        self.c = np.zeros(0, np.int32)        # en cuantas nubes se vio
        self.pend = []
        self.cambio = False

    def agregar(self, xyz):
        """Una nube (Nx3, ya en 'map'). Cuenta 1 por voxel aunque caigan muchos puntos en el."""
        if len(xyz):
            self.pend.append(np.unique(claves(np.asarray(xyz, np.float64), self.res)))

    def fusionar(self):
        """Mete las nubes pendientes en el mapa (barato: hacerlo 1-2 veces por segundo)."""
        if not self.pend:
            return 0
        nk, nc = np.unique(np.concatenate(self.pend), return_counts=True)
        self.pend = []
        idx = np.searchsorted(self.k, nk)
        hay = idx < len(self.k)
        hay[hay] = self.k[idx[hay]] == nk[hay]
        np.add.at(self.c, idx[hay], nc[hay].astype(np.int32))
        nuevas = ~hay
        if nuevas.any() and len(self.k) < self.max:
            self.k = np.insert(self.k, idx[nuevas], nk[nuevas])
            self.c = np.insert(self.c, idx[nuevas], nc[nuevas].astype(np.int32))
        self.cambio = True
        return int(nuevas.sum())

    def puntos(self, min_vistas=3):
        return centros(self.k[self.c >= min_vistas], self.res)

    def borrar(self):
        self.k = np.zeros(0, np.int64)
        self.c = np.zeros(0, np.int32)
        self.pend = []
        self.cambio = True

    def cargar_puntos(self, xyz, vistas=1000):
        """Mapa guardado: sus voxeles cuentan como muy vistos (siempre se dibujan)."""
        self.borrar()
        self.agregar(xyz)
        self.fusionar()
        self.c[:] = vistas


def colores(z, z0=None, z1=None):
    """Altura -> RGB uint8 (azul, cian, verde, amarillo, rojo). Rango: percentiles 2-98 si no se da."""
    z = np.asarray(z, np.float64)
    if len(z) == 0:
        return np.zeros((0, 3), np.uint8)
    if z0 is None or z1 is None:
        z0, z1 = np.percentile(z, 2), np.percentile(z, 98)
    t = np.clip((z - z0) / max(z1 - z0, 1e-3), 0.0, 1.0)
    puntos = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
    r = np.interp(t, puntos, [0.15, 0.0, 0.2, 1.0, 1.0])
    g = np.interp(t, puntos, [0.30, 0.8, 0.9, 0.9, 0.25])
    b = np.interp(t, puntos, [1.0, 1.0, 0.3, 0.1, 0.1])
    return (np.stack([r, g, b], axis=1) * 255).astype(np.uint8)


def rgb_float(rgb):
    """RGB uint8 -> float32 empaquetado (formato 'rgb' de PCL / RViz)."""
    rgb = np.asarray(rgb, np.uint32)
    u = (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]
    return u.astype(np.uint32).view(np.float32)


def guardar_pcd(ruta, xyz, rgb=None):
    xyz = np.asarray(xyz, np.float32)
    if rgb is None:
        rgb = colores(xyz[:, 2])
    d = np.zeros(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    d["x"], d["y"], d["z"], d["rgb"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb_float(rgb)
    cab = ("# .PCD v0.7 - go2_nav mapa 3D\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n"
           "COUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (len(d), len(d)))
    with open(ruta, "wb") as f:
        f.write(cab.encode("ascii"))
        f.write(d.tobytes())


def leer_pcd(ruta):
    """PCD (ascii o binario, campos float) -> Nx3 float32."""
    with open(ruta, "rb") as f:
        cab = {}
        while True:
            ln = f.readline()
            if not ln:
                raise ValueError("PCD sin DATA")
            t = ln.decode("ascii", "replace").strip().split()
            if not t or t[0].startswith("#"):
                continue
            cab[t[0].upper()] = t[1:]
            if t[0].upper() == "DATA":
                break
        campos = cab["FIELDS"]
        tam = [int(v) for v in cab["SIZE"]]
        tip = cab["TYPE"]
        n = int(cab["POINTS"][0])
        fmt = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 4): "<u4", ("U", 1): "u1", ("I", 4): "<i4",
               ("U", 2): "<u2", ("I", 2): "<i2", ("I", 1): "i1"}
        dt = np.dtype([(c, fmt[(t, s)]) for c, t, s in zip(campos, tip, tam)])
        if cab["DATA"][0] == "binary":
            d = np.frombuffer(f.read(dt.itemsize * n), dtype=dt, count=n)
            return np.stack([d["x"], d["y"], d["z"]], axis=1).astype(np.float32)
        a = np.loadtxt(f, ndmin=2)
        ix = [campos.index(k) for k in ("x", "y", "z")]
        return a[:, ix].astype(np.float32)
