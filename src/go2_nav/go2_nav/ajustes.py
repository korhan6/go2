"""Tus ajustes (~/.go2_nav/ajustes.yaml) y parametros que no tumban al nodo.

ROS es muy estricto con el archivo de parametros: si un nodo tiene un valor antes de
'ros__parameters', una lista dentro de otra o un tipo distinto (0 donde va 0.0), el nodo NO
arranca. Y el archivo de ajustes va a todos los nodos: un error de tipeo tiraba todo. Aca se revisa
antes: lo que esta bien pasa, con el tipo del valor del paquete; lo que no, se avisa y se ignora.
"""
import os

ESCALAR = (bool, int, float, str)


def _numero(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def tipo_como(v, ref):
    """v con el tipo de ref (el valor del paquete; None = no se sabe). None = ROS no lo puede leer."""
    if v is None or isinstance(v, dict):
        return None
    if isinstance(ref, str) and not isinstance(v, str):
        return str(v)                                   # la huella: [[0.4, 0.1], ...] -> "[[0.4, 0.1], ...]"
    if isinstance(v, list):
        if not v or not all(isinstance(x, ESCALAR) for x in v):
            return None                                 # lista vacia o con listas dentro
        if all(_numero(x) for x in v):
            if any(isinstance(x, float) for x in v) or (isinstance(ref, list) and ref and isinstance(ref[0], float)):
                return [float(x) for x in v]
            return v
        return v if all(type(x) is type(v[0]) for x in v) else None
    if not isinstance(v, ESCALAR):
        return None
    if ref is None or isinstance(ref, list):
        return v
    if isinstance(ref, bool) or isinstance(v, bool):
        return v if isinstance(ref, bool) and isinstance(v, bool) else None
    if isinstance(ref, float) and _numero(v):
        return float(v)
    if isinstance(ref, int) and _numero(v):
        return int(v) if float(v).is_integer() else None
    if _numero(ref) != _numero(v):
        return None                                     # "hola" donde va un numero
    return v


def nodos(d, ruta=()):
    """{'ns/nodo': parametros} de un archivo de parametros de ROS; los errores van a una lista."""
    salida, malos = {}, []
    for k, v in (d or {}).items():
        nombre = ruta + (str(k).strip("/"),)
        if not isinstance(v, dict):
            malos.append("%s: '%s' no va aca (falta 'ros__parameters:' arriba)" % ("/".join(nombre[:-1]) or "?", k))
            continue
        if "ros__parameters" in v:
            if isinstance(v["ros__parameters"], dict):
                salida["/".join(nombre)] = v["ros__parameters"]
            else:
                malos.append("%s: 'ros__parameters' esta vacio" % "/".join(nombre))
            for k2, v2 in v.items():
                if k2 != "ros__parameters" and not isinstance(v2, dict):
                    malos.append("%s: '%s' va DENTRO de 'ros__parameters'" % ("/".join(nombre), k2))
            v = {k2: v2 for k2, v2 in v.items() if k2 != "ros__parameters" and isinstance(v2, dict)}
        s, m = nodos(v, nombre)
        salida.update(s)
        malos += m
    return salida, malos


def _limpiar(usuario, paquete, ruta, malos, nuevos):
    limpio = {}
    for k, v in usuario.items():
        ref = paquete.get(k) if isinstance(paquete, dict) else None
        nombre = ruta + "." + str(k) if ruta else str(k)
        if isinstance(v, dict):
            sub = _limpiar(v, ref if isinstance(ref, dict) else None, nombre, malos, nuevos)
            if sub:
                limpio[k] = sub
            continue
        v2 = tipo_como(v, ref)
        if v2 is None:
            malos.append("%s = %r: ese valor no sirve (en el paquete es %r)" % (nombre, v, ref))
            continue
        if paquete is not None and ref is None:
            nuevos.append(nombre)
        limpio[k] = v2
    return limpio


def revisar(usuario, paquete):
    """usuario: el YAML de ajustes; paquete: {'ns/nodo': parametros} con los valores del paquete
    (los nodos que no estan ahi pasan sin comparar). -> (yaml limpio, avisos)."""
    if not isinstance(usuario, dict):
        return {}, ["no es una lista de nodos con 'ros__parameters'"]
    por_nodo, avisos = nodos(usuario)
    limpio = {}
    for nodo, prm in por_nodo.items():
        malos, nuevos = [], []
        p = _limpiar(prm, paquete.get(nodo, {}) if nodo in paquete else None, "", malos, nuevos)
        avisos += ["%s: %s" % (nodo, m) for m in malos]
        if nuevos:
            avisos.append("%s: %s no esta en el paquete (si esta mal escrito, no hace nada)"
                          % (nodo, ", ".join(nuevos)))
        if p:
            d = limpio
            for parte in nodo.split("/"):
                d = d.setdefault(parte, {})
            d["ros__parameters"] = p
    return limpio, avisos


def leer(ruta, paquete, salida):
    """Lee y revisa el archivo de ajustes; escribe el revisado en 'salida'.
    -> ([salida] o [], mensaje para el log o None si no hay archivo)."""
    import yaml
    if not os.path.isfile(ruta):
        return [], None
    try:
        with open(ruta) as f:
            usuario = yaml.safe_load(f)
    except Exception as e:
        return [], "go2_nav: %s esta MAL ESCRITO, no lo uso: %s" % (ruta, e)
    if usuario is None:
        return [], None
    limpio, avisos = revisar(usuario, paquete)
    txt = "".join("\n  !! " + a for a in avisos)
    if not limpio:
        return [], "go2_nav: %s no tiene nada que pueda usar%s" % (ruta, txt)
    with open(salida, "w") as f:
        yaml.safe_dump(limpio, f)
    return [salida], "go2_nav: uso tus ajustes de %s (%s)%s" % (ruta, ", ".join(sorted(nodos(limpio)[0])), txt)


class _Valor:
    def __init__(self, value):
        self.value = value


def declarador(nodo):
    """Como nodo.declare_parameter(nombre, defecto) pero no se cae con un tipo distinto: 0 donde
    va 0.0 se convierte; "hola" donde va un numero avisa y usa el defecto."""
    from rcl_interfaces.msg import ParameterDescriptor

    def p(nombre, defecto):
        v = nodo.declare_parameter(nombre, defecto, ParameterDescriptor(dynamic_typing=True)).value
        v2 = tipo_como(v, defecto)
        if v2 is None:
            nodo.get_logger().warn("parametro %s = %r no sirve: uso %r" % (nombre, v, defecto))
            v2 = defecto
        return _Valor(v2)
    return p
