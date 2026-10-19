"""Configuracion de go2nav.

Los valores por defecto estan aqui. Para cambiarlos sin tocar el codigo, crea
/unitree/dev/go2nav/datos/config.json (o la ruta de GO2NAV_DATOS) con solo las claves
que quieras cambiar, por ejemplo:  {"v_max": 0.5, "escaleras": "al_final"}
"""
import json
import os

DATOS = os.environ.get("GO2NAV_DATOS", os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "datos"))

DEFECTO = {
    # --- comunicacion con el perro (DDS interno) ---
    "dds_interfaz": "eth0",            # interfaz donde hablan los servicios de Unitree
    "topico_nube": "rt/utlidar/cloud_deskewed",
    "topico_odom": "rt/utlidar/robot_odom",     # odometria lidar-inercial (mejor que patas)
    # versiones "lf" (~20 Hz): las normales van a 266/440 Hz y decodificarlas en Python
    # se come la CPU del perro (el lazo de control se vuelve lento y gira 8x mas despacio)
    "topico_estado": "rt/lf/sportmodestate",    # respaldo de pose + IMU + modo
    "topico_lowstate": "rt/lf/lowstate",        # bateria y control remoto
    "firmware": "auto",                # auto | legacy (<=1.1.6, modo AI) | mcf (>=1.1.7)

    # --- movimiento ---
    "v_max": 0.60,                     # m/s en recta despejada
    "v_explorar": 0.50,                # m/s mientras mapea (el mapa sale mejor un poco mas lento)
    "w_max": 1.0,                      # rad/s
    "w_min": 0.55,                     # rad/s: el Go2 no gira en el sitio con menos que esto
    "girar_si": 45.0,                  # grados de error de rumbo para girar en el sitio
    "lookahead": 0.55,                 # m (persecucion pura)
    "parar_a": 0.15,                   # m desde el hocico: obstaculo -> alto total
    "frenar_desde": 0.90,              # m desde el hocico: empieza a bajar la velocidad

    # --- mapa ---
    "celda": 0.05,                     # m
    "obst_min": 0.10,                  # m sobre el suelo: mas bajo se pisa
    "obst_max": 0.60,                  # m sobre el suelo: mas alto el perro pasa por debajo
    "alcance": 6.0,                    # m: puntos mas lejos no entran al mapa (ruido)
    "radio_robot": 0.25,               # m: el centro del perro no pasa mas cerca de una pared que esto
    "radio_giro": 0.38,                # m: espacio libre que necesita para girar en el sitio (esquinas del cuerpo)
    "margen_suave": 0.30,              # m extra con costo (prefiere ir por el medio)
    "ajuste_scan": True,               # corrige la deriva con scan matching
    "altura_piso": 1.2,                # m de diferencia para considerar otro piso

    # --- exploracion ---
    "frontera_min": 6,                 # celdas minimas de una frontera
    "t_atasco": 8.0,                   # s sin avanzar -> maniobra de rescate
    "intentos_frontera": 2,            # fallos antes de descartar una frontera
    "volver_al_terminar": True,        # al terminar el mapeo vuelve al punto de inicio

    # --- escaleras ---
    "escaleras": "inmediato",          # inmediato | al_final | off
    "v_subir": 0.30,                   # m/s subiendo (de frente)
    "v_bajar": 0.15,                   # m/s bajando (SIEMPRE en reversa)
    "ancho_min_escalera": 0.45,        # m

    # --- seguridad ---
    "bateria_min": 15,                 # % -> deja de explorar y se queda quieto
    "control_remoto_pausa": True,      # si mueves el control fisico, go2nav se pausa
    "sesgo_cerca": [0.14, 0.19],       # correccion de altura del L1 de cerca (ver terreno.corregir_cerca); [0, 0] = sin correccion
    "t_sensor": 1.5,                   # s sin lidar/odom -> alto
}


def cargar():
    cfg = dict(DEFECTO)
    ruta = os.path.join(DATOS, "config.json")
    if os.path.isfile(ruta):
        try:
            with open(ruta) as f:
                extra = json.load(f)
            desconocidas = [k for k in extra if k not in DEFECTO]
            cfg.update({k: v for k, v in extra.items() if k in DEFECTO})
            if desconocidas:
                print("config.json: claves desconocidas ignoradas: " + ", ".join(desconocidas))
        except (OSError, ValueError) as e:
            print("config.json invalido (%s); uso los valores por defecto" % e)
    return cfg


def guardar_clave(clave, valor):
    """Cambia una clave en config.json (persistente)."""
    if clave not in DEFECTO:
        raise KeyError(clave)
    os.makedirs(DATOS, exist_ok=True)
    ruta = os.path.join(DATOS, "config.json")
    datos = {}
    if os.path.isfile(ruta):
        try:
            with open(ruta) as f:
                datos = json.load(f)
        except (OSError, ValueError):
            datos = {}
    datos[clave] = valor
    tmp = ruta + ".tmp"
    with open(tmp, "w") as f:
        json.dump(datos, f, indent=2)
    os.replace(tmp, ruta)
