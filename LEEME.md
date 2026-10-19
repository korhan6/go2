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
| `go2_gestor` | propio | exploración por fronteras **con compromiso**, puntos guardados, estado en vivo |
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
- **Escaleras con dos testigos:** el lidar solo no basta. Una escalera del lidar pasa a ser un
  portal **solo si la cámara la confirma** en esa dirección. Si la cámara mira hacia ahí y no ve
  escalera, se descarta. Sin visión, no toma escaleras solo.

```
go2 camara                 # en vivo: fuente, fps, objetos y escalera
go2 vision_probar          # GPU y velocidad del modelo (sin ROS)
go2 vision_probar camara   # + una foto de la camara con detecciones -> ~/.go2_nav/vision_prueba.jpg
go2 escaleras vision off   # volver a confiar solo en el lidar (no recomendado)
go2 lanzar camara:=videohub   # forzar la otra fuente de video
```

## Escaleras y varios pisos

- **Mientras queden escaleras sin usar, el mapeo NO está completo.** Con `al_final` (por
  defecto) termina el piso, va a la escalera y la toma; mapea el otro piso y vuelve.
- **Subida:** se alinea, se centra y **sube de frente**.
- **Bajada:**
  1. se acerca al borde despacio (nunca a menos de 30 cm);
  2. confirma que ve escalones;
  3. **gira 180°**, verifica que detrás hay escalones y **baja SIEMPRE en reversa**;
  4. abajo, se vuelve a poner de frente.
- **Seguridad:**
  - un balcón o un desnivel sin escalones **nunca** se toma como escalera;
  - antes de pisar los escalones, cualquier duda = cancela;
  - ya encima de los escalones nunca cambia de marcha: si algo falla queda **DETENIDO** y
    espera `go2 seguir` o el control remoto.
- **Un mapa por piso:** antes de cruzar guarda el mapa del piso; en un piso nuevo empieza otro, y
  al volver a un piso conocido lo recarga donde corresponde. `go2 ir A` funciona aunque A esté en
  otro piso.
- **La marcha de escaleras** es FreeWalk (firmware ≥ 1.1.7, el tuyo es 1.5.1).

### Órdenes
```
go2 escaleras                   # escaleras vistas y a qué piso llevan
go2 escaleras inmediato         # tomarlas apenas las ve
go2 escaleras al_final          # tomarlas al terminar el piso
go2 escaleras off               # no tomarlas nunca
go2 subir                       # tomar a mano la que tiene delante
go2 bajar
```
**La primera vez, pruébalo a mano y con alguien al lado de la escalera.**

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
- **Ajustes:** `~/go2_ws/src/go2_nav/config/nav2.yaml`. Después:
  ```
  cd ~/go2_ws && colcon build --symlink-install
  ```
