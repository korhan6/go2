# go2_nav (ROS 2): navegación autónoma para el Go2 por cable

La laptop se conecta al perro por **Ethernet**, igual que la mochila EDU de Unitree: queda en la
red interna (192.168.123.x) y recibe por **DDS directo** el lidar (~15 Hz), la odometría
(~150 Hz) y el estado. **No hay WebRTC, ni compresión, ni servidores en el medio**: la latencia
es de milisegundos.

## Qué hace

| Pieza | De dónde sale | Qué hace |
|---|---|---|
| `go2_driver` | propio (mensajes de [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)) | `/cmd_vel` → `Move` del perro a 20 Hz desde su propio timer; si nadie manda órdenes en 0.4 s, el perro se detiene; odometría y TF con la hora de la laptop (el reloj del perro está corrido horas); el control remoto siempre manda |
| `go2_limpiador` | propio, con lo medido en TU perro | nube limpia: quita el cuerpo, **corrige el error de altura del L1 de cerca**, quita puntos fantasma (calibración al explorar + aprendizaje solo mientras camina), **los reflejos del piso brillante** y ruido suelto, recuerda los obstáculos reales que quedan pegados al cuerpo, marca caídas/escaleras como obstáculo. Publica lo eliminado en `/go2/fantasmas` (rojo en RViz) |
| SLAM | [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | el mapa (`/map`) |
| Navegación | [Nav2](https://github.com/ros-navigation/navigation2) | planificador A* + controlador **MPPI omnidireccional** (camina de lado y retrocede) con la **huella exacta** del perro |
| Ruta comprometida | árbol de comportamiento oficial de Nav2 *"replanning only if path becomes invalid"* | calcula la ruta y la sigue hasta el final; solo replanifica si queda bloqueada |
| `go2_gestor` | propio | exploración por fronteras **con compromiso** (`nucleo/exploracion.py`), escaleras y pisos, puntos guardados, estado en vivo |
| `go2_escaleras` | propio (`nucleo/terreno.py`, `nucleo/escaleras.py`) | confirma escaleras (subida: lidar + cámara; bajada: lidar viendo el primer escalón) y las cruza: **sube de frente, baja en reversa** |
| `go2_vision` | [YOLOv8](https://github.com/ultralytics/ultralytics) entrenado con Open Images V7 (600 clases) | la **cámara del perro** por el cable, YOLO en la **GPU** de la laptop: ve escaleras, personas, sillas, puertas... y las ubica en el mapa con la distancia del lidar |

### La huella (obstacle avoidance)
- **Adelante, la cabeza (más angosta)**, no un rectángulo: hasta 40 cm del centro y ±10 cm de ancho.
  Antes el frente era un rectángulo hasta 45 cm: para girar en el sitio pedía 57 cm libres
  alrededor y en un pasillo de 1 m decía que no cabía. Ahora pide ~50 cm.
- **Lados y atrás:** el cuerpo exacto (±16 cm, -37 cm), con las esquinas enteras (recortar las de
  atrás hacía que rozara el marco al girar dentro de una puerta).
- **Margen de 7 cm alrededor** (`footprint_padding`) para el controlador: sin él, al salir de una
  puerta girando, la esquina enganchaba el marco. El perro pasa por puertas de 70 cm.
- **El freno (collision_monitor) es solo para emergencias:** frena si la orden lo llevaría a tocar
  el CUERPO (+2 cm) en menos de 0.5 s. Lo que el lidar ve de lejos lo **esquiva** el controlador
  (pasa por al lado con el margen); antes el freno usaba la huella con margen y 0.8 s, y frenaba
  al pasar junto a algo que ya estaba esquivando. Justo delante del hocico (20 cm) va al 70 %.
- El controlador revisa la **huella completa** en cada trayectoria, no un círculo. Por eso pasa
  por donde cabe el cuerpo. Si no cabe girando en el sitio, se desliza de lado o retrocede.

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

**MAPEAR TÚ Y QUE EL PERRO NAVEGUE SOLO (recomendado):**

1. Mapear (una vez):
   ```
   go2 lanzar rviz:=true            # terminal 1 (empieza un mapa nuevo)
   go2 manejar                      # terminal 2: w/s adelante/atras, a/d girar, q/e de lado, x salir
                                    #   (frena si hay algo a menos de 20 cm del hocico o la cola)
                                    #   (o con el control remoto de Unitree)
   ```
   Paséalo despacio por toda la casa. Donde quieras volver, en la terminal 3:
   ```
   go2 guardar cocina               # cualquier nombre sin espacios: cocina, sala, cuarto_2
   ```
   Al terminar:
   ```
   go2 guardar_mapa casa            # guarda el mapa y los puntos en ~/.go2_nav/mapas/
   ```
2. Navegar (siempre que quieras, otro día también):
   ```
   go2 lanzar mapa:=casa rviz:=true # el perro se UBICA en el mapa guardado (no lo cambia)
   go2 ir cocina
   go2 ir sala
   go2 casa                         # donde empezaste a mapear
   ```
   - Pon al perro donde EMPEZASTE a mapear. Si está en otro lado, en RViz usa
     "2D Pose Estimate": clic donde está y arrastra hacia donde mira.
   - Puntos nuevos con el mapa cargado: llévalo con `go2 manejar` y `go2 guardar NOMBRE`
     (quedan guardados con el mapa).
   - `go2 puntos` lista los puntos; `go2 mapas` los mapas guardados.
   - Con un mapa cargado no explora ni toma escaleras (es de un piso).

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

### Qué muestra RViz
- **El perro de verdad** (modelo 3D de Unitree) en vez de la caja: las patas se mueven con los
  ángulos reales de los motores. La caja (la huella de Nav2) y la zona de frenado siguen en la
  lista de RViz, apagadas: prende "Huella del perro" si quieres verla. (Mallas y medidas:
  `unitreerobotics/unitree_ros`, licencia BSD-3, reducidas a 1.2 MB en `go2_nav/meshes`. Si no
  aparece el perro: `sudo apt install ros-jazzy-robot-state-publisher`.)
- **Nube del lidar** (color = altura) y **Obstáculos** (cuadraditos naranjas): los naranjas son
  **lo único que usa Nav2** para esquivar. Si un naranja no está, Nav2 no lo ve; si hay un naranja
  donde no hay nada, es un obstáculo falso (mándame foto).
- **El sombreado (costos locales y globales) viene apagado.** Nav2 lo arma solo con los
  naranjas: celeste = el centro del perro no puede estar ahí, rojo/azul = más caro cuanto más cerca
  de algo. Prende "Costos locales" para ver por dónde no puede ir.
- Los fantasmas ya quitados (puntos rojos en la cabeza) vienen apagados.

## Piso brillante (cerámica): el piso es un espejo
El L1 ve en el piso brillante **todo lo que hay arriba, repetido debajo del piso** (la mesa, la
puerta, las paredes), justo en el mismo lugar. Y lejos, con el rayo rasante, el piso casi no
devuelve. Antes eso se tomaba como "caídas", el mapa de costos se llenaba de puntitos y de su
inflado, y RViz mostraba al perro **"como si no hubiese espacio"** (salvo por donde ya había
pasado). Ahora:
- **Se quitan los reflejos:** un punto debajo del piso donde hay algo arriba es su reflejo (debajo
  de una escalera de verdad hay aire). `go2 ver` dice cuántos quitó ("reflejos").
- **Una caída tiene que tener la sombra de un borde de verdad** entre el perro y ella (el reflejo
  del tablero de una mesa, que el lidar no ve directo, no la tiene: entre el perro y la mesa se ve
  piso), y estar delante o al costado (atrás el cuerpo tapa).
- **Los mapas de costos usan SOLO la nube de ahora:** en cada vuelta se borran enteros (3.3 m
  alrededor del L1) y se marca lo que el limpiador ve ahora. Antes guardaban cada marca hasta que un
  rayo del L1 la cruzara: con el L1 ralo, un rebote de un instante quedaba para siempre, con 55 cm
  de sombreado alrededor, y el piso se llenaba de puntitos (la casa, 20-oct).
- Medido con el simulador de piso brillante (`pruebas/prueba_brillo.py`): mapa de costos local con
  57 celdas de obstáculo falsas → 0. Y el borde de una escalera de verdad se sigue marcando en su
  lugar (±4 cm).

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
- **Memoria de obstáculos:** como Nav2 solo usa la nube de ahora, el limpiador recuerda lo que el
  L1 deja de ver. Lo que queda **fuera de su vista** (detrás de la cabeza, pegado al cuerpo, en la
  zona de los fantasmas) se sigue marcando mientras esté a menos de 3 m (antes eran 20 s: trabado
  en una puerta, olvidaba el marco y lo empujaba); detrás o al costado, si se vio al menos 1 s. Lo
  que sigue **a la vista y ya no se ve** (un rebote, una persona que pasó) se olvida en 1.5 s. Solo entra a la
  memoria lo visto a más de 35 cm del cuerpo: ahí nunca hay fantasmas.

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
| Escalera **angosta entre paredes** (85 cm) con el lidar como el L1 de verdad (`prueba_nucleo.py pegado` y `pared`; con ROS, `prueba_escalera_angosta.py`) | sube con el hocico a 13 cm del primer escalón (también corrido 8 cm o torcido 8°); baja de reversa y para al quedar plano con una pared 70 cm detrás del pie: la cola queda a 12 cm (antes chocaba) |
| Escalera angosta, **28 casos difíciles** (corrido ±10 cm, torcido ±10°, odometría girada, lidar ralo, perro que patina de lado y gira solo) (`prueba_nucleo.py variantes_todas`) | 28 de 28 sin choques, se aparta del centro 4 cm en promedio (antes: 25 de 28, 7 cm, hasta 26 cm) |
| Rumbo inicial mal medido ±7° | lo corrige con lo que ve delante: se aparta ≤ 4 cm (antes hasta 9 cm) |
| Puerta de 72 cm con el frente de la huella más chico (`prueba_puerta.py 150 2500 meta`, 6 veces) | 6 de 6 sin chocar, en 10–12 s |
| Caja de 60 cm en el medio del camino (`prueba_obstaculo.py`, 3 veces) | la rodea y llega en 11–12 s, sin chocar ni frenar |
| `guardar A` → explorar → `ir A` | llega a 3 cm de A |
| `calibrar` con fantasmas pegados al cuerpo | adelante 0.17 → 3.06 m; luego explora normal |
| Tu grabación real (18-oct) por el limpiador | lados limpios, 0 caídas falsas con 19 % de reflejos, 3 ms por nube |
| **Piso brillante** con una mesa delante, girando 20 s (`prueba_brillo.py`, con Nav2) | celdas de obstáculo falsas en el mapa de costos local: 57 → 0; caídas y escaleras de bajada falsas: 0 |

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
  la marcha de escaleras y sube. **Arriba, apenas el cuerpo queda plano** (las 4 patas en el piso),
  frena y dice **`LLEGUE ARRIBA`**. Antes seguía de frente: creía que "había más escalera delante"
  con lo que había visto en los últimos 4 s (la misma escalera que acababa de subir) y después
  caminaba 30 cm más. Ahora solo sigue si AHORA ve más escalones delante (un descanso). Con una
  pared 80 cm después del último escalón: antes chocaba, ahora para a 7 cm de quedar plano.
- **Cualquier bajada es una escalera** (dentro de la casa no hay barrancos ni balcones): basta con
  que el piso se corte hacia abajo en 2 de los 4 carriles del centro, sin importar si el borde es
  recto (escaleras curvas o en abanico) ni si se ve el primer escalón. La confirma desde ~5 cm hasta
  ~1.5 m del borde. Solo una rampa muy suave (menos de 17°) no cuenta: esa se camina normal.
- **Bajada: SIEMPRE en reversa.**
  1. se alinea con la escalera y se centra (rumbo fino con las paredes y los escalones: ±1–2°),
     y se pone a 65 cm del borde;
  2. **gira 180°**. Si no cabe girando (necesita 35 cm libres desde el centro), primero se corre
     hasta 30 cm alejándose de lo que estorba (nunca hacia la bajada) y vuelve a probar;
  3. comprueba por odometría que el borde quedó detrás, pone la marcha y **baja de espaldas**;
  4. abajo, **apenas el cuerpo vuelve a quedar plano o casi** (las 4 patas en el piso: lo dice el
     IMU), frena en seco y dice **`LLEGUE ABAJO`**. No da ni un paso más de reversa (atrás el lidar
     no ve nada) y se queda mirando la escalera: no gira ahí, porque el primer escalón queda pegado
     delante y girando lo pisaría.
- **Derecho y centrado encima de los escalones** (subiendo y bajando): el lidar mide delante el
  rumbo de la escalera con las contrahuellas y las paredes, y el perro sigue la línea del centro
  de la escalera con la odometría (así se corrige aunque patine o al principio de la bajada, cuando
  la escalera todavía queda detrás). Con paredes a los dos lados, se centra entre ellas.
- **Seguridad:**
  - **solo para dentro de la casa:** cualquier bajada se toma como escalera (ver arriba);
  - si pasó el borde y no bajó nada, se detiene; si recorrió 6 m y no terminó, también;
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
El L1 va en la cabeza, a ~25 cm del suelo. Un escalón de 17 cm queda en su **sombra** durante
0.57 × (distancia al borde): con el hocico a 0.7 m del borde no se ve **nada** del primer escalón
(solo "una caída"). Como dentro de la casa toda caída empinada de borde recto es escalera, igual
se confirma de lejos (con el hocico a 1 m, en 0.4 s). Medido con
`python3 pruebas/prueba_nucleo.py deteccion`.

### Si NO bajó la escalera, mira esto (en este orden)
```
go2 escaleras        # que ve el lidar delante y por que (no) lo toma por escalera
go2 log              # 'DESCARTO', 'no confirme', 'escalera no tomada (...)', 'frontera ...'
```
1. **`go2 escaleras` dice `baja solo en N de 4 carriles`, `algo delante` o `no veo el suelo
   delante`** → ponlo de frente y centrado, a 0.5–1 m del borde, y vuelve a mirar. Las líneas
   `lo que ve delante (cm ...)` y `suelo estimado a X cm bajo el cuerpo` son lo que mide el lidar:
   si el suelo no sale ~30 cm, mándame una foto de eso.
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
   a `v_bajar` (0.15 m/s): prueba `ros2 param set /go2_escaleras v_bajar 0.2` (en vivo; para
   dejarlo, ponlo en `~/.go2_nav/ajustes.yaml`, ver [Tus ajustes](#dónde-está-cada-valor-y-cómo-cambiarlo)).
6. **`no hay espacio para girar junto a la escalera (X cm libres, hacen falta 35)`** → ya se corrió
   hasta 30 cm buscando lugar y no lo encontró: pasillos de menos de ~75 cm no. Si tu perro sí cabe,
   `ros2 param set /go2_escaleras radio_giro 0.32` (en vivo; para dejarlo, en `~/.go2_nav/ajustes.yaml`).

### Si el perro no avanza (pisa en el sitio)
El Go2 casi no camina por debajo de ~0.20 m/s. Si avanza menos del 40 % de lo que se le pide, la
velocidad sube sola de a poco (en la escalera 3 cm/s cada 0.5 s hasta 0.40 m/s; con Nav2 x1.15
cada 0.6 s hasta 0.45 m/s, solo con espacio libre en esa dirección) y vuelve a bajar cuando ya
avanza. En `go2 ver` sale "empujando X m/s". Topes: `v_escalera_max` (go2_escaleras) y
`v_empuje_max` (go2_driver).

### Altura del cuerpo en la escalera
Bajando en reversa, el L1 (debajo del mentón) podía rozar el borde del escalón de arriba si el
perro se agachaba. Con la marcha de escaleras el driver sube el cuerpo **+3 cm** (orden
BodyHeight de Unitree, máximo +3 cm) y al terminar lo vuelve a la normal. Para comprobar en
plano que tu firmware la acepta:
```
go2 altura 3     # "el perro la ACEPTO, la altura medida cambio +3.0 cm" = funciona
go2 altura 0     # vuelve a la normal
```
Si dice "RECHAZO", tu firmware no la permite en esa marcha y no hace nada (no es peligroso).

### Subir una escalera (lo más seguro)
Llévalo (con `go2 manejar` o el control) delante del primer escalón, mirándolo de frente y
centrado: sirve **desde pegado al escalón** (`manejar` frena a 20 cm) hasta ~2 m. Si queda a menos
de 10 cm, retrocede solo un poco para ver el primer escalón. Sal de `manejar` con x, y:
```
go2 escaleras   # "subida: escalera de SUBIDA a X m" = la ve. Si dice "no veo escalones", acomódalo
                # (y mira "suelo estimado a X cm": tiene que salir ~30)
go2 subir       # se alinea, se centra, pone la marcha de escaleras y sube DE FRENTE; arriba empieza
                # el mapa de ese piso
```
`go2 manejar` nunca sube escalones: frena antes (el escalón es "algo delante") a propósito.

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

## Dónde está cada valor y cómo cambiarlo
**Las carpetas en la laptop:**

| Carpeta | Qué hay | ¿Se borra al actualizar? |
|---|---|---|
| `~/go2_repo` | la copia de GitHub (de acá se copia el código) | **sí**, se baja de nuevo |
| `~/go2_ws/src/go2_nav` | el código y la configuración que se compilan | **sí**, se reemplaza |
| `~/go2_ws/src/unitree_ros2` | los mensajes de Unitree | no |
| `~/go2_ws/build`, `install`, `log` | lo que arma `colcon build` (no tocar) | se rehace |
| `~/.go2_nav` | **tus datos**: `mapas/`, `puntos.json`, `propio.npz` (`go2 calibrar`), `movimiento.json` (`go2 calibrar_mov`), `ajustes.yaml` | **no** |

**Para cambiar un valor sin que se pierda al actualizar:** usa `~/.go2_nav/ajustes.yaml`. Lo que
pongas ahí pisa el valor del paquete; lo que no pongas queda igual. Al arrancar, `go2 lanzar` dice
`uso tus ajustes de ... (go2_escaleras, local_costmap, ...)`.
```
mkdir -p ~/.go2_nav
cp ~/go2_ws/install/go2_nav/share/go2_nav/config/ajustes_ejemplo.yaml ~/.go2_nav/ajustes.yaml
nano ~/.go2_nav/ajustes.yaml     # quita el '#' de lo que quieras cambiar; 2 espacios por nivel
go2 lanzar                       # relanzar para que tome los cambios
ros2 param get /local_costmap/local_costmap footprint_padding   # comprobar que lo tomó
```
Si algo está mal escrito (falta `ros__parameters:`, una palabra donde va un número), **no se
cae nada**: `go2 lanzar` lo avisa con `!!` y usa el valor del paquete para eso. `0` donde va `0.0`
se arregla solo. Un nombre mal escrito avisa `no esta en el paquete` y no hace nada.

Los valores de `go2_escaleras` también cambian **en vivo** (`ros2 param set /go2_escaleras
radio_giro 0.32`), pero eso se pierde al relanzar. Los de los otros nodos se leen solo al arrancar.

**Dónde está cada cosa** (archivos dentro de `src/go2_nav/`):

| Qué | Archivo | ¿En `ajustes.yaml`? |
|---|---|---|
| **Hitbox de Nav2** (`footprint`, `footprint_padding` 0.07) | `config/nav2.yaml`: `local_costmap` y `global_costmap` | sí (`local_costmap: local_costmap: ...`) |
| Distancia a las paredes (`inflation_radius` 0.55) | `config/nav2.yaml`: `inflation_layer` de los dos costmaps | sí |
| **Freno de emergencia** (`Huella`, 0.5 s; `Frente`, 70 %) | `config/nav2.yaml`: `collision_monitor` | sí |
| Velocidades de Nav2 (`vx_max` 0.6, `wz_max` 1.0) | `config/nav2.yaml`: `controller_server` → `FollowPath` | sí |
| Escaleras: `radio_giro` 0.35, `v_subir`, `v_bajar`, `ancho_min_escalera` | `go2_nav/nucleo/config.py` (`DEFECTO`) | sí (`go2_escaleras`) |
| Lidar: `obst_min`/`obst_max` (qué altura es obstáculo), reflejos, caídas | `go2_nav/limpiador.py` (los `p("...")`) | sí (`go2_limpiador`) |
| Driver: zona muerta, empuje, velocidades máximas | `go2_nav/driver.py` (los `p("...")`) | sí (`go2_driver`) |
| Exploración: radios, tiempos de atasco | `go2_nav/gestor.py` (los `p("...")`) | sí (`go2_gestor`) |
| Maniobra de escalera: inclinación, tiempos, distancias | `go2_nav/nucleo/escaleras.py` (constantes en mayúsculas) | no: es código |
| Medidas del cuerpo para el lidar (`LARGO2`, `ANCHO2`) | `go2_nav/nucleo/terreno.py` | no: es código |
| Freno de `go2 manejar` (20 cm) | `go2_nav/cli.py` (`FRENO`) | no: es código |

Si cambias la forma del hitbox, cambia también el polígono `Huella` del `collision_monitor` (el
cuerpo + 2 cm) y `HUELLA` en `nucleo/config.py` (el dibujo de `go2 costmap` y las pruebas).

## Pruebas sin el perro y sin ROS
```
python3 pruebas/prueba_nucleo.py           # deteccion de bajadas por distancia, balcon, posible bajada -> verla de cerca,
                                           # subir y bajar en el simulador, fronteras
python3 pruebas/prueba_nucleo.py bajar     # la bajada en reversa paso a paso
python3 pruebas/prueba_escalera_angosta.py # con ROS: la escalera angosta de la casa (subir y bajar)
python3 pruebas/prueba_nucleo.py variantes_todas   # 28 subidas/bajadas dificiles en la escalera angosta (~30 min)
python3 pruebas/prueba_obstaculo.py        # con ROS: una caja en el camino, la tiene que rodear
```
Solo necesita numpy y scipy. `pruebas/prueba_e2e.py` y `prueba_escalera.py` lanzan todo con ROS 2.

Con ROS 2 (Nav2 + slam_toolbox reales, perro simulado):
```
python3 pruebas/prueba_puerta.py 150 2500   # cuarto -> puerta de 72 cm -> pasillo con escalera; lidar ralo como el L1
go2 sim mundo:=puerta puntos_sim:=2500      # lo mismo, para mirarlo en RViz
```

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
- **Llega a una puerta y no pasa / se traba en el marco / se queda quieto delante:**
  manda una foto de esto, con el perro intentando:
  ```
  go2 costmap          # lo que ve Nav2 alrededor del perro (¿la puerta está abierta en el mapa?)
  go2 costmap global   # lo que usa el planificador
  go2 nav2             # 20 s de avisos de Nav2, con lo que significa cada uno
  ```
  Lo que ya se corrigió (reproducido con Nav2 real, perro simulado con lidar ralo como el L1, en
  una casa como la tuya: `pruebas/prueba_puerta.py`):
  - el marco de la puerta aparecía y desaparecía del mapa de obstáculos (el filtro anti-ruido
    pedía el mismo cubito en 2 de 3 nubes; con el L1, ralo, el marco lo pasaba la mitad de las
    veces). Ahora se confirma con 2 de las últimas 6 nubes en cubos de 10 cm;
  - los rayos que BORRAN obstáculos del mapa salían del centro del perro y no del lidar (en la
    cabeza): cada rayo hacia un punto detrás del marco lo "atravesaba" y lo borraba;
  - el borde de una escalera que baja quedaba sin marcar (sombra del lidar): Nav2 llevaba al
    perro hasta el filo. Ahora se marca la caída en el borde real (±4 cm, desde 0.6 m);
  - la memoria de obstáculos guardaba el centro de cubos de 8 cm (corría el marco hasta 8 cm);
  - el driver subía la velocidad mínima sin subir el giro (la curva salía 2.5 veces más abierta);
  - la arista del marco a veces quedaba borrada y la esquina del perro la enganchaba al girar:
    margen de 7 cm en la huella.

  | Prueba (Nav2 real, perro simulado con lidar ralo, puerta de 72 cm) | Antes | Ahora |
  |---|---|---|
  | Meta directa al pasillo, 6 veces: cruza sin chocar | 2 de 6 | 6 de 6 (en 10–13 s) |
  | Desvío de la línea verde (promedio por corrida) | — | 1–12 cm |
- **Nav2 quiere mandarlo por una rendija (entre patas de mesa, un hueco en una pared vista a
  medias):** el mapa de paredes que usa el planificador cierra todo hueco de menos de 40 cm
  entre obstáculos (el perro mide 31 cm de ancho: por ahí no pasa). Las puertas (≥ 60 cm) quedan
  abiertas. Es `HUECO_MIN` en `nucleo/exploracion.py`.
- **El perro no sigue la línea verde:** en la prueba con Nav2 real se aparta 1–12 cm en promedio
  (máximo 19 cm). Si en tu perro se aparta mucho más, el perro hace menos de lo que se le pide:
  corre `go2 calibrar_mov` y después `go2 probar_mov` (tiene que dar 90–100 %).
- **La ruta (línea verde en RViz) cruza una pared en vez de salir por la puerta:** el costmap
  global usa las paredes del SLAM (`/go2/mapa_paredes`, filtradas por el gestor: sin puntitos de
  ruido y con los huecos de 5 cm cerrados) más lo que ve el lidar. Si pasa igual, la pared
  todavía no está en el mapa del SLAM (no la vio de frente): la ruta se corrige sola cuando la ve.
- **El mapa del SLAM no dibuja las paredes (estrella de rayos con puntitos negros):** el L1 es un
  lidar 3D de pocos puntos. El `/scan` del SLAM usaba solo los obstáculos "confirmados" (vistos 2
  veces en el mismo lugar) de 10 a 55 cm de alto: a 3 m el L1 casi nunca confirma una pared, y el
  piso libre llegaba hasta ella sin pared al final. Ahora el `/scan` usa **todo lo que hay de 10 cm
  a 1.8 m** (una pared es alta: el L1 le pega a cualquier altura) sin pedir confirmación (el SLAM
  ya filtra el ruido). En la prueba (`pruebas/prueba_mapa_slam.py`) las 4 paredes del cuarto salen
  continuas y sin puntitos falsos. Además: 0.8 s de nubes juntas, haces de 1°, huecos chicos de
  una misma pared rellenos, y el mismo alcance en el `/scan` y el SLAM (6 m). Se ve desde un mapeo
  NUEVO (reinicia `go2 lanzar`; un mapa guardado con la versión anterior queda como estaba).
  Ajustes: `scan_alto_max` y `scan_confirmar` (go2_limpiador).
  - El L1 ve **solo hacia delante**: lo que queda detrás no se mapea hasta que lo mira. Al mapear a
    mano, gira despacio en cada cuarto (una vuelta entera) para que dibuje todas las paredes.
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
