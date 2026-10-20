"""Valores por defecto del nucleo (escaleras y guardian). go2_escaleras los expone como
parametros ROS del mismo nombre (ros2 param / launch)."""

# huella de Nav2 (config/nav2.yaml): la forma del perro, cabeza mas angosta (m, marco del cuerpo)
HUELLA = [(0.40, 0.10), (0.30, 0.16), (-0.37, 0.16), (-0.37, -0.16), (0.30, -0.16), (0.40, -0.10)]


def en_huella(fx, fy, margen=0.0):
    """(fx, fy) dentro de la huella (+ margen), en el marco del cuerpo. Acepta escalares."""
    dentro = True
    n = len(HUELLA)
    for i in range(n):                      # poligono convexo: del lado de adentro de cada arista
        (x0, y0), (x1, y1) = HUELLA[i], HUELLA[(i + 1) % n]
        ex, ey = x1 - x0, y1 - y0
        L = (ex * ex + ey * ey) ** 0.5
        dentro = dentro & (((fx - x0) * ey - (fy - y0) * ex) / L <= margen)
    return dentro


DEFECTO = {
    # --- escaleras ---
    "v_subir": 0.30,                   # m/s subiendo (de frente)
    "v_bajar": 0.15,                   # m/s bajando (SIEMPRE en reversa)
    # m: mas angosta no se toma sola = el cuerpo (0.31 m) + 5 cm por lado. El lidar mide el ancho en
    # franjas de 10 cm (0.40 m es lo minimo que puede medir): se compara con +-5 cm de tolerancia
    "ancho_min_escalera": 0.41,
    "v_escalera_max": 0.40,            # m/s: hasta aca sube la velocidad si el perro no avanza (EMPUJE)
    "w_min": 0.55,                     # rad/s: el Go2 no gira en el sitio con menos que esto
    # m libres alrededor (desde el centro) para girar en el sitio: la punta de la cabeza (0.36 m, angosta)
    # y las patas (~0.30). Antes 0.38: con la escalera de la casa decia "no hay espacio" teniendolo
    "radio_giro": 0.35,
    # --- guardian ---
    "obst_min": 0.10,                  # m sobre el suelo: mas bajo se pisa (cables, umbrales)
    "obst_max": 0.60,                  # m sobre el suelo: mas alto el perro pasa por debajo
    "sesgo_cerca": [0.14, 0.19],       # error de altura del L1 de cerca (ver terreno.corregir_cerca); [0, 0] = sin correccion
}
