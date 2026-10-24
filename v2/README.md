# Go2 v2 — stack de CMU en Docker

La v2 cambia el corazón de la navegación por el **autonomy stack de CMU para el Go2**
([jizhang-cmu/autonomy_stack_go2](https://github.com/jizhang-cmu/autonomy_stack_go2)). Es el de los
videos donde alguien se le cruza al perro y este lo esquiva en diagonal sin detenerse.

| | v1 (Nav2 propio) | v2 (CMU) |
|---|---|---|
| Odometría | patas × 1.839 (escala corregida a mano) | **Point-LIO** con la IMU del L1 (lidar + inercial) |
| Evasión | MPPI a 15 Hz + costmaps + collision_monitor | **planificador local con cientos de trayectorias precalculadas, a ~100 Hz**: elige otra en el acto, sin replanificar |
| Rutas largas | SmacPlanner2D + BT de Nav2 | **FAR planner** (grafo de visibilidad que se arma mientras camina) |
| Movimiento | gira y avanza | avanza, gira **y se desliza de lado** cerca de los obstáculos (omnidireccional) |
| Le habla al perro | driver (Python) | **go2_guardian**: Move a 50 Hz y StopMove si deja de recibir órdenes por 0.25 s |
| ROS | Jazzy, en tu PC | **Humble, dentro de Docker** (igual que en la Jetson de la mochila) |

La v1 queda intacta: sigue en `~/go2nav_ws` (comando `go2`) y en la rama `claude/v1-checkpoint` del
repo. **No corras las dos a la vez**: las dos le mandarían órdenes al perro, así que `go2v2` se niega
a arrancar si ve la v1 corriendo.

## Qué agrega go2_v2 encima de CMU

- **go2_sensores**: hace la misma cuenta que el `transform_everything.py` de CMU (probada número
  por número contra la original en `pruebas/prueba_transformacion.py`), pero vectorizada. Tarda
  ~1–2 ms por nube; la original iba punto por punto en Python y es la "demora de más de 1 s" que
  CMU reporta en laptops. Además, la IMU no arranca hasta conocer el desfase de relojes.
- **go2_guardian**: es el único nodo que le habla al perro. Hace esto:
  - Pone límites duros de velocidad.
  - Tiene un watchdog: si el planificador se cae, StopMove en 0.25 s.
  - **Frena si Point-LIO o el mapa de terreno dejan de llegar.** El seguidor de CMU sigue mandando
    la última velocidad aunque Point-LIO se caiga, y el perro caminaría a ciegas.
  - No camina mientras se está parando o echando.
  - Maneja los estados acostar, levantar, bloquear, pausa y recuperar, desde el PS4 o por comando.
  - Apaga la evasión de obstáculos de Unitree, que frenaba al perro.
  - Manda un StopMove final al cerrarse con Ctrl+C.
- **Comandos** `go2v2 ir / guardar / puntos / parar / estado`.
- **Pruebas sin ROS** en `pruebas/`: la matemática contra la original de CMU, la lógica del
  guardián, y los tres nodos corriendo con un ROS simulado.

## Instalar (una vez)

```bash
cd ~/go2v2                 # esta carpeta
./go2v2 instalar           # construye la imagen (15-25 min la primera vez) y deja el comando 'go2v2'
go2v2 red                  # buffer de red grande para las nubes del L1 (pide sudo)
go2v2 calibrar             # IMU del L1: perro DE PIE y con 1 m libre (pasitos, quieto, gira 20 s)
```

Si `docker` pide sudo: `sudo usermod -aG docker $USER` y cierra sesión (o reinicia).

## Usar

```bash
go2v2 lanzar               # SLAM + evasión + rutas + RViz
go2v2 lanzar velocidad:=0.6 ruta:=false
```

En RViz, el botón **Goalpoint** manda al perro a donde hagas clic. Si alguien se cruza, lo esquiva.
También desde otra terminal:

```bash
go2v2 ir 3 1.5             # metros desde donde arrancó el perro (x adelante, y izquierda)
go2v2 guardar cocina ; go2v2 ir cocina
go2v2 parar                # deja de navegar
go2v2 pausa                # freno de emergencia (otra vez para seguir)
go2v2 estado               # demora, Hz, pose, estado del guardián
```

**Control PS4:**

| Botón | Qué hace |
|---|---|
| Stick derecho | Manejar con evasión automática |
| L2 sostenido | Modo autónomo (sigue la meta) |
| R2 sostenido | Manual **sin** evasión |
| R1 | Borra el mapa de terreno |
| L1 | Borra el grafo de rutas de FAR (lo de CMU) |
| CÍRCULO | Pausa |
| SHARE + CRUZ | Acostar |
| SHARE + TRIÁNGULO | Levantar |
| SHARE + CUADRADO | Bloquear / desbloquear |
| SHARE + CÍRCULO | Recuperar (RecoveryStand: se cayó o quedó raro) |

Las combinaciones van con SHARE porque CMU ya usa L1 y R1.

## Si algo no anda

- **`go2v2 estado`**:
  - `demora_ms` debería ser < 150. Si es mucho mayor, se están perdiendo nubes: corre `go2v2 red`.
  - Si `pose` dice que no llega, Point-LIO no arrancó: revisa que haya nube (`nubes` > 0) y IMU.
  - `parado_por` del guardián dice por qué no camina, por ejemplo "la odometría tiene 1.2 s de
    atraso" o "no llega el mapa de terreno".
- **"el reloj del perro SALTÓ"**: el perro se reinició con la v2 andando. Relánzala con
  `go2v2 detener` y después `go2v2 lanzar`.
- **El perro se va torcido o el mapa se dobla**: recalibra la IMU (`go2v2 calibrar`) y espera
  10–20 s quieto al arrancar (CMU avisa que al inicio puede haber demora).
- **Obstáculos bajos**: el L1 es ruidoso y CMU no ve lo que mide menos de ~30 cm. Para cosas más
  bajas hace falta otro sensor (la mochila con Mid-360 es la mejora oficial).
- **`go2v2 log`** muestra los últimos mensajes.
- **`go2v2 grabar 60`** graba los datos crudos para revisarlos sin el perro.

## Cambiar la v2

El código de go2_v2 (`ros_ws/src/go2_v2`) se monta dentro del Docker: un cambio en Python vale con
solo volver a `go2v2 lanzar`, sin reconstruir. Los parámetros del guardián están en
`ros_ws/src/go2_v2/config/guardian.yaml`.
