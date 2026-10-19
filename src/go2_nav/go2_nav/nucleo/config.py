"""Valores por defecto del nucleo (escaleras y guardian). go2_escaleras los expone como
parametros ROS del mismo nombre (ros2 param / launch)."""

DEFECTO = {
    # --- escaleras ---
    "v_subir": 0.30,                   # m/s subiendo (de frente)
    "v_bajar": 0.15,                   # m/s bajando (SIEMPRE en reversa)
    "ancho_min_escalera": 0.45,        # m: mas angosta no se toma sola
    "w_min": 0.55,                     # rad/s: el Go2 no gira en el sitio con menos que esto
    "radio_giro": 0.38,                # m libres alrededor para girar en el sitio (esquinas del cuerpo)
    # --- guardian ---
    "obst_min": 0.10,                  # m sobre el suelo: mas bajo se pisa (cables, umbrales)
    "obst_max": 0.60,                  # m sobre el suelo: mas alto el perro pasa por debajo
    "sesgo_cerca": [0.14, 0.19],       # error de altura del L1 de cerca (ver terreno.corregir_cerca); [0, 0] = sin correccion
}
