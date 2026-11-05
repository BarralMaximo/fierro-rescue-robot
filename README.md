# Fierro

Software de búsqueda y rescate para un robot móvil con ROS 2. Recibe el mapa de un laberinto, se ubica, lo recorre buscando cajas rojas (las víctimas), reporta dónde están, vuelve a la largada y avisa que terminó.

Lo hicimos para el [Challenge JAR 2026](https://airclub-udesa.github.io/jar_site/reglamento/), organizado por el [AIR Club UdeSA](https://github.com/AIRclub-UdeSA) en la Jornada Argentina de Robótica. Corre en un Yahboom ROSMASTER X3 (ruedas mecanum, lidar 2D, cámara, IMU) con ROS 2 Humble y Python. La localización, la planificación y el control son propios: no usa Nav2 ni AMCL.

![Recorrido del robot en el mapa de práctica maze_3_6x6](docs/maze_3_run.png)

*Una corrida en el simulador con este código. La línea azul es el recorrido, las estrellas son las víctimas y los puntos, los reportes. Encontró las 3 y volvió a la largada: 600 puntos, el máximo de este mapa.*

## Resultado

Este repo es el código con el que corrimos la clasificación del 4 de noviembre de 2026, en el robot real y en una arena que no conocíamos. El desarrollo completo, commit a commit, está en el [fork de entrega](https://github.com/marijanicki/jar_challenge_fierro).

| | Simulador (Gazebo, 11 mundos × 2) | Robot real (clasificación) |
|---|---|---|
| Víctimas encontradas | 55 de 58 | 4 de 4 |
| Error de los reportes | mediana 4.8 cm | de 3.6 a 10 cm |
| Regreso a la largada | 22 de 22 corridas | sí, `/done` a los 370 s |
| Colisiones | 0 choques, 8 roces de 0 a 2 cm | **15** |
| Puntaje | 94.4 % de lo posible | **0** (400 + 100 − 750; el total no baja de 0) |

Encontrar y ubicar las cajas funcionó en la pista igual que en el simulador. Esquivar, no: cada colisión descuenta 50 puntos y las 15 borraron todo lo demás. Fueron roces de costado y de cola. En el análisis posterior vimos dos cosas que el simulador no tenía: unos 0.4 s de retardo, porque el software corre en una laptop que le habla al robot por wifi, y motores que daban el 88 % del avance y el 67 % del giro pedidos. La esquina de la cola llegó a tocar puntas de pared.

Después probamos parámetros más conservadores contra los choques y la segunda corrida salió peor, así que acá quedan los de la primera.

## Cómo funciona

Son tres nodos. Al largar, los tres reciben el mapa en `/map`; antes de eso el robot no se mueve.

```mermaid
flowchart LR
    sensores["lidar, odometría, IMU"] --> localizer
    sensores --> perception
    sensores --> navigator
    camara["cámara"] --> perception
    localizer -->|"TF map→odom"| perception
    localizer -->|"/jar/pose"| navigator
    perception -->|"/jar/red, /victimas"| navigator
    perception -->|"/victimas"| juez["juez"]
    navigator -->|"/done"| juez
    navigator -->|"/cmd_vel"| robot["robot"]
```

### Ubicarse (`localizer`)

Es un filtro de partículas propio, con 500 partículas y 90 rayos del lidar por scan.

- **Predicción.** El avance sale de las ruedas y el giro del giróscopo. Si la IMU se cae o gira al revés que las ruedas, el giro también sale de las ruedas.
- **Corrección.** Cada partícula se puntúa por qué tan cerca de una pared caen los rayos, con un campo de distancias calculado una sola vez al llegar el mapa.
- **Largada.** Los jueces ponen el robot a mano cerca del origen, así que la primera pose sale de una búsqueda por fuerza bruta: ±0.4 m y la vuelta entera, primero gruesa y después fina.
- **Relocalización.** Si varios scans seguidos calzan mal, repite esa búsqueda cerca de la última pose. Solo la acepta si calza claramente mejor; si no, lo que cambió es el mundo (un objeto que no está en el mapa), no la pose.

### Encontrar las cajas (`perception`)

La cámara dice qué es rojo y el lidar dice dónde está.

1. Arma una máscara del rojo en HSV. Los umbrales se eligieron con fotos reales del robot, alterando la luz.
2. Proyecta cada rayo del lidar sobre la foto. Los que caen en píxeles rojos dan la distancia a la caja.
3. Filtra el paralaje entre cámara y lidar: erosiona la silueta y corta los rayos donde la distancia salta.
4. Agrega media caja para pasar de la cara que ve el lidar al centro, que es lo que mide el juez.
5. Agrupa las detecciones por caja y toma la mediana. Reporta solo con suficientes rayos en varios scans y poca dispersión, porque cada reporte gasta uno de los N disponibles.

Sobre una grabación del robot real con ruido agregado (67 pasadas) no hubo ningún reporte falso.

### Recorrer y volver (`navigator`)

- **Recorrido.** Como el mapa se conoce, no hace falta explorar: alcanza con elegir desde dónde mirar. Al llegar `/map` calcula una sola vez los puntos de paso (cobertura golosa) y su orden (vecino más cercano y 2-opt).
- **El lidar busca y la cámara confirma.** Lo que aparece fuera de las paredes del mapa con forma de caja (RANSAC, una cara o dos en L) es un candidato. Con ruedas mecanum, hacia dónde va y hacia dónde mira son independientes: el robot apunta la cámara al candidato sin dejar de avanzar.
- **Caminos.** Dijkstra sobre la grilla, con un costo que crece cerca de las paredes para ir por el medio del pasillo, y pure pursuit para seguirlo.
- **Protección.** El freno modela el chasis como un círculo, que no cubre la cola. Para eso está `tail_barrier`, un problema de optimización (QP) que cambia el comando lo menos posible para que el rectángulo del chasis no se acerque a lo que ve el lidar detrás.
- **Regreso.** Vuelve cuando reportó las N víctimas, cuando ya vio todo o cuando se le acaba el tiempo, y publica `/done` a menos de 15 cm de la largada.

## Código

```text
fierro/
├── core.py         mapa, filtro de partículas, recorrido y agrupado de detecciones
├── detector.py     rojo en la imagen + lidar → centros de caja
├── mission.py      la misión: caminos, candidatos, protección y regreso
├── localizer.py    nodo: pose del robot y TF map→odom
├── perception.py   nodo: detecta las cajas y las reporta en /victimas
└── navigator.py    nodo: corre la misión y publica /cmd_vel y /done
launch/fierro.launch.py     levanta los tres nodos
params/params.yaml          todos los parámetros, comentados
test/                       pruebas sin simulador
```

`core.py`, `detector.py` y `mission.py` no dependen de ROS: los nodos solo conectan esa lógica con los topics. Por eso los tests pueden probar la misión entera en una simulación cinemática, sin Gazebo.

## Cómo correrlo

Necesitás ROS 2 Humble. Cloná este repo en la carpeta `src` de tu workspace y, desde el workspace:

```bash
colcon build --packages-select fierro
source install/setup.bash
python3 -m pytest src/fierro/test     # pruebas, sin simulador
```

Para una corrida completa hacen falta el [simulador](https://github.com/AIRclub-UdeSA/yahboom_rosmaster) (tag `jar2026-v1`) y el juez de práctica, `tools/juez.py` del [template del Challenge](https://github.com/AIRclub-UdeSA/jar_challenge), que publica el mapa y da el puntaje. En tres terminales, en este orden:

```bash
# 1. El simulador
SIM=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:=$SIM/worlds/maze_3_6x6_victimas.world

# 2. Fierro (victims:=N es la cantidad a buscar)
ros2 launch fierro fierro.launch.py use_sim_time:=true victims:=3

# 3. El juez, desde el template
SIM=$(ros2 pkg prefix --share yahboom_rosmaster_gazebo)
python3 tools/juez.py --mundo maze_3_6x6 --mapa $SIM/maps/maze_3_6x6.yaml \
  --ground-truth --esperar-topic /scan --retardo 5 --duracion 300
```

## Créditos

Equipo Fierro: [Máximo Barral](https://github.com/BarralMaximo) y [marijanicki](https://github.com/marijanicki).

El proyecto parte del [template del Challenge](https://github.com/AIRclub-UdeSA/jar_challenge) del AIR Club UdeSA. Este repo es el commit `f24afc5` del fork reorganizado, con la misma lógica y los mismos parámetros.

## Licencia

[MIT](LICENSE).
