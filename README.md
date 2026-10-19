# go2_nav (ROS 2): navegación autónoma para el Go2 por cable

La laptop se conecta al perro por **Ethernet**, igual que la mochila EDU de Unitree: queda en la
red interna (192.168.123.x) y recibe por **DDS directo** el lidar (~15 Hz), la odometría
(~150 Hz) y el estado. **No hay WebRTC, ni compresión, ni servidores en el medio**: la latencia
es de milisegundos.

## Qué hace

| Pieza | De dónde sale | Qué hace |
|---|---|---|
| `go2_driver` | propio (mensajes de [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)) | `/cmd_vel` → `Move` del perro a 20 Hz desde su propio timer; si nadie manda órdenes en 0.4 s, el perro se detiene; odometría y TF con la hora de la laptop (el reloj del perro está corrido horas); el control remoto siempre manda |
| `go2_limpiador` | propio, con lo medido en TU perro | nube limpia: quita el cuerpo, **corrige el error de altura del L1 de cerca**, quita puntos fantasma (calibración al explorar + aprendizaje solo mientras camina) y ruido suelto, recuerda los obstáculos reales que quedan pegados al cuerpo, marca caídas/escaleras como obstáculo. Publica lo eliminado en `/go2/fantasmas` (rojo en RViz) |
| SLAM | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | el mapa (`/map`) |
| Navegación | [Nav2](https://github.com/ros-navigation/navigation2) | planificador A* + controlador **MPPI omnidireccional** (camina de lado y retrocede) con la **huella exacta** del perro |
| Ruta comprometida | árbol de comportamiento oficial de Nav2 *"replanning only if path becomes invalid"* | calcula la ruta y la sigue hasta el final; solo replanifica si queda bloqueada |
| `go2_gestor` | propio | exploración por fronteras **con compromiso** (`nucleo/exploracion.py`), escaleras y pisos, puntos guardados, estado en vivo |
| `go2_escaleras` | propio (`nucleo/terreno.py`, `nucleo/escaleras.py`) | confirma escaleras (subida: lidar + cámara; bajada: lidar viendo el primer escalón) y las cruza: **sube de frente, baja en reversa** |
| `go2_vision` | [YOLOv8](https://github.com/ultralytics/ultralytics) entrenado con Open Images V7 (600 clases) | la **cámara del perro** por el cable, YOLO en la **GPU** de la laptop: ve escaleras, personas, sillas, puertas... y las ubica en el mapa con la distancia del lidar |

### La huella (obstacle avoidance)
- **Lados:** el cuerpo exacto (±16 cm).
- **Atrás:** exacto.
- **Adelante:** +9 cm de margen, más una zona de 38 cm delante del hocico donde va a la mitad de
  velocidad.
- El controlador revisa la **huella rectangular** completa en cada trayectoria, no un círculo.
  Por eso pasa por donde cabe el cuerpo. Si no cabe girando en el sitio, se desliza de lado o
  retrocede.

## Instalar (una vez, en la laptop)
```
tar xzf go2_nav_ros2.tar.gz
cd go2_nav_ros2
bash instalar.sh
```
- La parte de visión instala PyTorch con CUDA (~3 GB) en `~/.go2_nav/venv_vision` y al final
  prueba tu GPU (`go2 vision_probar`). Sin la GPU NVIDIA (`nvidia-smi`) corre en el CPU, más lento.
- Instala ROS 2 Jazzy (Ubuntu 24.04), Nav2, slam_toolbox y CycloneDDS, y compila en `~/go2_ws`.
- Si tienes un `.venv` activo, primero corre `deactivate`.

## Usar

**Sin perro (para ver que todo anda):**
```
go2 sim                    # perro simulado + RViz
go2 explorar               # en otra terminal
go2 ver
```

**Con el perro:**
1. Conecta el cable. La laptop debe tener una IP `192.168.123.x` (tu perfil `go2`).
2. Cierra la app de Unitree.
3. En una terminal:
   ```
   go2 lanzar rviz:=true
   ```
4. En otra terminal:
   ```
   go2 explorar               # calibra los fantasmas (2 vueltas), mapea solo y vuelve al inicio
   go2 ver                    # estado en vivo
   go2 guardar A              # punto A = donde está ahora
   go2 ir A
   go2 casa
   go2 parar
   ```
También puedes mandarlo a cualquier punto con **"2D Goal Pose"** en RViz.

**Mapeo a mano (si la exploración automática no sale del cuarto):**
```
go2 lanzar rviz:=true          # terminal 1
go2 manejar                    # terminal 2: w/s adelante/atrás, a/d girar, q/e de lado, x salir
go2 guardar A                  # terminal 3: con el perro donde quieras el punto A
                               # (sigue manejándolo hasta B)
go2 guardar B
go2 ir A                       # (sal antes de 'go2 manejar' con x)
go2 ir B
```
Mientras lo manejas, el mapa se arma solo (en RViz: gris = sin ver, transparente = suelo visto,
morado = obstáculo, verde = zonas por explorar, amarillo = la meta actual).

## Fantasmas: se corrigen solos
- **Cada `go2 explorar`** empieza con 2 vueltas en el sitio (~15 s) para aprender los puntos
  fantasma de ese momento. Si algo REAL está tan cerca que lo tocaría al girar, corta el giro.
  Para distinguir, mira si el punto se queda quieto en el mundo mientras el perro gira.
- **Además los aprende solo mientras camina:** un fantasma va pegado al cuerpo y sale en el mismo
  lugar del perro mire hacia donde mire. Un objeto real solo sale cuando el perro mira hacia él.
  Caminar recto junto a una pared no la borra.
- En RViz, los puntos **rojos** son los fantasmas **ya eliminados**: no entran al mapa ni a la
  navegación. Están ahí solo para que veas qué se quitó.
- `go2 explorar sin` = sin las 2 vueltas iniciales.
- **Memoria de obstáculos:** la zona pegada al cuerpo, donde salen los fantasmas, se filtra. Para
  que eso no esconda cosas reales, todo lo que el lidar ve a más de 35 cm del cuerpo se recuerda
  y se sigue marcando cuando queda en esa zona ciega (hasta 20 s). Los fantasmas nunca aparecen
  lejos del cuerpo, así que no entran a la memoria.

## `go2 ver`
```
10:30:01 explorando x= 1.20 y= 0.40  35°  mapa 18.2 m2  adel=0.85 izq=0.42 der=0.31 atras=- [lidar 15Hz 6ms | odom 150Hz move 20Hz] fantasmas=310 | explorando: voy a zona nueva (2.1 m)
```
- `adel/izq/der/atras`: espacio libre desde el BORDE del cuerpo, ya limpio.
- `lidar`: nubes por segundo que llegan, y lo que tarda limpiarlas.
- `move`: órdenes por segundo al perro (20 cuando camina).
- `fantasmas`: puntos eliminados en la última nube.
- `orden`: lo que se le manda al perro (vx, vy, giro). Si Nav2 pide menos de 0.20 m/s (o menos
  de 0.40 rad/s girando en el sitio) y hay espacio libre, se sube a ese mínimo (más lento el Go2
  pisa en el sitio y no avanza).
- `fantasmas=N (aprendidos M)`: puntos quitados en la última nube / celdas de fantasma aprendidas
  caminando.
- `!! ...`: avisos (sin calibrar, evasión de Unitree prendida).

## Probado (Nav2 + slam_toolbox reales, perro simulado con los defectos del L1 real)

| Prueba | Resultado |
|---|---|
| Cuarto con cama + pasillo angosto al lado (el caso que fallaba) | pasa por el pasillo, mapea los 2 cuartos (53 m²) en 3 min, 0 choques, vuelve al inicio |
| Dos cuartos con dos puertas | 64 m² en 1.5 min, sin ir y venir entre puertas, 0 choques |
| **Casa 14 × 10 m con entrepiso de 1.53 m, `go2 explorar` solo** | mapea la planta baja (128 m²), ve la escalera y NO dice "completo"; **sube de frente**, mapea arriba, **gira 180° y baja en reversa**, recarga el mapa de abajo y vuelve al inicio. 9 min, 0 choques |
| Subir y bajar a mano (`go2 subir`, `go2 bajar`) | sube 1.53 m de frente; arriba gira 180° y baja de reversa hasta abajo |
| `guardar A` → explorar → `ir A` | llega a 3 cm de A |
| `calibrar` con fantasmas pegados al cuerpo | adelante 0.17 → 3.06 m; luego explora normal |
| Tu grabación real (18-oct) por el limpiador | lados limpios, 0 caídas falsas con 19 % de reflejos, 3 ms por nube |

## Visión por computadora (cámara + YOLO en la GPU)

- **Cámara:** el video H.264 que el perro manda por el cable (multicast `230.1.1.1:1720`, 30 fps).
  Si no llega, pide fotos al servicio `videohub` del perro por DDS (más lento).
- **Modelo:** YOLOv8m de Open Images V7, en la GPU con FP16. Reconoce 600 cosas, entre ellas
  **escaleras ("Stairs")**, personas, sillas, mesas, puertas, camas, sofás, perros y gatos.
- **Dónde está cada cosa:** la dirección sale de la imagen y la distancia sale del lidar. En RViz
  salen etiquetas en el mapa ("escalera 82 %", "persona 70 %") y el panel **Camara (YOLO)** con los
  recuadros.
- **Subidas con dos testigos:** una escalera de subida del lidar pasa a ser un portal **solo si
  la cámara la confirma** en esa dirección. Las **bajadas** las confirma el lidar viendo el primer
  escalón (desde arriba la cámara no ve una bajada). Ver "Escaleras y varios pisos".

```
go2 camara                 # en vivo: fuente, fps, objetos y escalera
go2 vision_probar          # GPU y velocidad del modelo (sin ROS)
go2 vision_probar camara   # + una foto de la camara con detecciones -> ~/.go2_nav/vision_prueba.jpg
go2 escaleras vision off   # confiar solo en el lidar (tambien para las subidas)
go2 lanzar camara:=videohub   # forzar la otra fuente de video
```

## Escaleras y varios pisos

- **Mientras queden escaleras sin usar, el mapeo NO está completo.** Con `al_final` (por
  defecto) termina el piso, va a la escalera y la toma; mapea el otro piso y vuelve.
- **Subida: SIEMPRE de frente.** Se alinea, se centra, se acerca a 45 cm del primer escalón, pone
  la marcha de escaleras y sube.
- **Bajada automática, aunque la vea de lejos.** De lejos el primer escalón queda en la sombra
  del lidar y una bajada se ve solo como "una caída". La exploración nunca se acerca sola a una
  caída (es obstáculo en el costmap), así que la caída queda como **posible bajada** (portal
  "por confirmar") y, como cualquier escalera pendiente, el perro **va a mirarla de cerca**:
  al terminar el piso con `al_final`, o enseguida con `inmediato`. Si de cerca ve el escalón,
  baja. Si no, era un balcón: lo intenta 2 veces y no vuelve.
- **Bajada: SIEMPRE en reversa.**
  1. se acerca al borde despacio hasta que el lidar **ve el primer escalón** (con el hocico a
     25–55 cm del borde; ver abajo por qué);
  2. se alinea y se centra, y se pone a 65 cm del borde;
  3. **gira 180°**, comprueba por odometría que el borde quedó detrás (y que detrás no hay suelo
     firme ni una pared, si el lidar llega a ver), pone la marcha y **baja de espaldas**;
  4. abajo, sale 30 cm, quita la marcha y se vuelve a poner de frente.
- **Seguridad:**
  - un balcón o un desnivel sin escalones **nunca** se toma como escalera: hace falta ver la
    huella del primer escalón (plana, a 9–30 cm bajo el borde, en varios carriles);
  - antes de pisar los escalones, cualquier duda = cancela;
  - ya encima de los escalones nunca cambia de marcha: si algo falla queda **DETENIDO** y
    espera `go2 seguir` o el control remoto.
- **Un mapa por piso:** antes de cruzar guarda el mapa del piso; en un piso nuevo empieza otro, y
  al volver a un piso conocido lo recarga donde corresponde. `go2 ir A` funciona aunque A esté en
  otro piso.
- **La marcha de escaleras** es FreeWalk (firmware ≥ 1.1.7, el tuyo es 1.5.1).

### Quién confirma cada escalera (`go2 escaleras vision ...`)
| Tipo | Quién la confirma | Por qué |
|---|---|---|
| Subida | lidar **y cámara (YOLO "Stairs")** | de frente la cámara ve los escalones muy bien; evita tomar un mueble bajo por escalera |
| Bajada | **solo el lidar**, viendo el primer escalón 4 veces seguidas | la cámara va a 30 cm del suelo mirando hacia adelante: desde arriba de una bajada **no ve escalones**, ve el piso de abajo o la pared de enfrente. Si la cámara pudiera vetarla, la bajada se descartaría siempre |

- `subida` (por defecto): lo de la tabla. `requerida`: la cámara confirma todo (no recomendado
  para bajadas). `off`: solo el lidar.
- Un "la cámara mira ahí y no ve escalera" descarta la candidata **2 minutos**, no para siempre.

### Por qué una BAJADA se ve solo de cerca (geometría del L1)
El L1 va en la cabeza, a ~30 cm del suelo. Un escalón de 17 cm queda en su **sombra** durante
0.57 × (distancia al borde): con el hocico a 0.7 m del borde no se ve **nada** del primer escalón
(solo "una caída"), a 0.35 m se ven ~10 cm de huella, y pegado al borde deja de ver el suelo de
delante. Por eso go2_escaleras se acerca despacio hasta 25 cm del borde (los pies quedan a
42 cm) y, si se pasó, retrocede. Medido con `python3 pruebas/prueba_nucleo.py deteccion`.

### Si NO bajó la escalera, mira esto (en este orden)
```
go2 escaleras        # que ve el lidar delante y por que (no) lo toma por escalera
go2 log              # 'DESCARTO', 'no confirme', 'escalera no tomada (...)', 'frontera ...'
```
1. **`go2 escaleras` dice `caida a X m pero NO veo el primer escalon`** → el lidar ve el
   desnivel pero no la huella: está lejos del borde (> 0.55 m) o el primer escalón es raro
   (muy alto, > 30 cm; alfombra oscura; borde con listón). Acércalo con `go2 manejar` hasta
   ~30 cm del borde y vuelve a mirar. Si sigue sin verla, con `go2 bajar` tampoco va a bajar:
   **por diseño** (sin huella vista = balcón).
2. **En el log sale `DESCARTO la escalera de bajar ... la camara`** → versión anterior o
   `go2 escaleras vision requerida`: la cámara nunca ve una bajada desde arriba. Pon
   `go2 escaleras vision subida` (ahora es el valor por defecto).
3. **La escalera está en la lista como `(POSIBLE: falta verla de cerca)` o `(sin usar)` y el
   perro sigue explorando** → política
   `al_final`: va al terminar el piso. Si el piso no "termina" (fantasmas, zonas que fallan),
   `go2 escaleras inmediato` o `go2 bajar` con el perro mirándola.
4. **`escalera no tomada (no confirme la escalera de bajar: ...)`** o **`posible bajada ... de
   cerca NO veo el escalon`** → llegó al borde pero al acercarse no vio la huella (ver 1).
   Una posible bajada tiene 2 intentos y después queda como `(balcon/desnivel)`.
5. **`el perro no avanza en la escalera`** → la marcha FreeWalk no se puso o el perro no camina
   a `v_bajar` (0.15 m/s): prueba `ros2 param set /go2_escaleras v_bajar 0.2`.
6. **`no hay espacio para girar junto a la escalera`** → necesita 38 cm libres alrededor para
   girar 180° a 65 cm del borde: pasillos de menos de 80 cm no.

### Órdenes
```
go2 escaleras                   # escaleras vistas, candidatas y que ve el lidar delante
go2 escaleras inmediato         # tomarlas apenas las ve
go2 escaleras al_final          # tomarlas al terminar el piso
go2 escaleras off               # no tomarlas nunca
go2 escaleras vision subida|requerida|off
go2 subir                       # tomar a mano la que tiene delante (de frente)
go2 bajar                       # idem, gira 180 y baja en reversa
```
**La primera vez, pruébalo a mano y con alguien al lado de la escalera.**

## Nav2 en la GPU: no se puede (y qué sí va en la GPU)
Nav2 (planificador, controlador MPPI, costmaps, SLAM) es código de CPU: no tiene versión CUDA
ni se puede "mandar a la GPU" con un parámetro. Lo único con GPU en Nav2 son los plugins de
NVIDIA **Isaac ROS** (nvblox como capa de costmap, cuVSLAM), que son otra pila completa
(contenedores Isaac ROS, otros sensores) y no se enchufan a este sistema. En esta laptop la GPU
la usa **YOLO** (`go2_vision`, FP16 con CUDA), que es lo pesado; Nav2 + slam_toolbox + el
limpiador consumen ~1–2 núcleos. Si la CPU va justa:
- `go2 lanzar vision:=false` quita la cámara (la visión ya corre con `nice 10`, detrás de Nav2);
- en `nav2.yaml` baja `batch_size` (1500 → 1000) y `time_steps` (56 → 40) del MPPI, y
  `update_frequency` del costmap local (10 → 5).

## Pruebas sin el perro y sin ROS
```
python3 pruebas/prueba_nucleo.py           # deteccion de bajadas por distancia, balcon, posible bajada -> verla de cerca,
                                           # subir y bajar en el simulador, fronteras
python3 pruebas/prueba_nucleo.py bajar     # la bajada en reversa paso a paso
```
Solo necesita numpy y scipy. `pruebas/prueba_e2e.py` y `prueba_escalera.py` lanzan todo con ROS 2.

## Si algo falla
- **El perro hace menos de lo que se le pide** (gira o camina más lento que la `orden` de
  `go2 ver`). Mídelo con `go2 probar_mov`, que lo gira 3 s y lo avanza 2 s.
  - **Corre una vez `go2 calibrar_mov`** (~40 s, con 1 m libre delante y detrás). Mide cuánto
    avanza y gira de verdad para cada orden y lo guarda en `~/.go2_nav/movimiento.json`. Desde ahí
    el driver le pide lo necesario para que haga lo que Nav2 quiere. Después `go2 probar_mov` debe
    dar ~90–100 %. Repítelo si cambias de piso (baldosa, alfombra).
  - Si da menos del 30 %, el problema está en el perro:
  1. Cierra la app de Unitree en el celular (su joystick manda órdenes "quieto" todo el tiempo).
  2. Apaga el go2nav de a bordo (la versión vieja que corre DENTRO del perro):
     ```
     ssh root@192.168.123.161 "systemctl stop go2nav; systemctl disable go2nav; pkill -f go2nav"
     ```
  3. Mira qué come la CPU del perro: `ssh root@192.168.123.161 "top -b -n 1 | head -25"`.
  4. Reinicia el perro y vuelve a correr `go2 probar_mov`.
- **El perro se queda "temblando" (pisa en el sitio y no avanza):** mira el final de la línea de
  `go2 ver`:
  - `!! SIN CALIBRAR`: el L1 ve "obstáculos" pegados al hocico. `go2 explorar` ahora calibra
    solo antes de empezar (2 vueltas en el sitio, ~15 s). También puedes correr `go2 calibrar`
    en un lugar despejado.
  - `!! EVASION UNITREE PRENDIDA`: la evasión de obstáculos de la app frena al perro. El driver
    la apaga solo; si sigue saliendo, apágala en la app y ciérrala.
  - `orden 0.00,0.00,0.00`: Nav2 no manda nada (no encuentra salida). Si pasan 20 s sin moverse,
    el gestor hace una maniobra corta hacia donde hay espacio y busca otra ruta; si no sale en 3
    intentos, se pausa: muévelo con el control y `go2 seguir`.
- **No llega nada** (`lidar -Hz`):
  - revisa `ip addr`: la laptop tiene que tener `192.168.123.x`;
  - corre `ros2 topic hz /utlidar/cloud_deskewed`;
  - el lidar se apaga si el perro está echado: `go2 levantar`.
- **El perro no se mueve:** `move` en `go2 ver` debe estar en ~20 Hz mientras navega. Si está en
  20 y no camina, el control remoto o la app tienen el mando: suelta los joysticks y cierra la app.
- **La ruta (línea verde en RViz) cruza una pared en vez de salir por la puerta:** el costmap
  global usa las paredes del SLAM (`/go2/mapa_paredes`, filtradas por el gestor: sin puntitos de
  ruido y con los huecos de 5 cm cerrados) más lo que ve el lidar. Si pasa igual, la pared
  todavía no está en el mapa del SLAM (no la vio de frente): la ruta se corrige sola cuando la ve.
- **El mapa del SLAM sale como una estrella de rayos con puntitos negros:** el L1 es un lidar 3D
  de pocos puntos, y el `/scan` que se le arma al SLAM quedaba con la mayoría de los haces
  vacíos. Ahora junta 0.8 s de nubes, usa haces de 1° y rellena los huecos chicos de una misma
  pared (de 27 % a 62 % de haces con lectura en la prueba). Además el alcance del `/scan` y del
  SLAM es el mismo (6 m): antes, cada lectura entre 8 y 10 m se dibujaba como un rayo largo sin
  pared al final. El mapa nuevo se ve bien desde un mapeo NUEVO (reinicia `go2 lanzar`).
- **Se pasa al alinearse con la ruta y vuelve (zigzag):** el Go2 no gira más lento de ~0.4 rad/s
  y responde ~0.3 s tarde. El driver subía cualquier corrección chica de Nav2 a 0.40 rad/s, se
  pasaba, Nav2 corregía al otro lado y así sin fin. Ahora una corrección de menos de 0.20 rad/s
  en el sitio se ignora (`w_quieto` del driver), el giro inicial de Nav2 frena antes, y la
  maniobra de escaleras gira en grueso, espera quieto y termina con pulsos cortos que nunca
  cruzan. Si todavía se pasa, sube `w_quieto` a 0.25.
- **`!! LIDAR CORTADO`:** dejaron de llegar nubes del lidar. El perro se queda quieto hasta que
  vuelven. Revisa el cable (bien metido en el perro y en la laptop), la batería, y
  `ros2 topic hz /utlidar/cloud_deskewed` (debe dar ~15).
- **`[interrumpida: algo cerca]`:** la calibración de 2 vueltas se cortó porque había algo a
  menos de ~45 cm. Empieza `go2 explorar` con el perro en un lugar despejado, a más de 50 cm de
  paredes y muebles.
- **No bajó la escalera:** ver "Si NO bajó la escalera" más arriba (`go2 escaleras` lo dice).
- **Ajustes:** `~/go2_ws/src/go2_nav/config/nav2.yaml`. Después:
  ```
  cd ~/go2_ws && colcon build --symlink-install
  ```
