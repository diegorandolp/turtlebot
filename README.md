# TurtleBot 4 · Proyecto final de Computer Vision (grupo 3)

**Autonomous and Gesture-Controlled Navigation Challenge** ([enunciado](docs/enunciado_proyecto.pdf)):
el robot va del punto A al punto B esquivando obstáculos, reconoce a la persona de una foto en B y
vuelve a A anunciando su nombre.

**Reglas del laboratorio (las que valen para la competencia):** el circuito es una sala de ~4×4 m con
cajas y señales en las paredes (que se pueden usar o no; pueden ser de color: importa la forma). En la
meta hay **varias fotos (~6)**, cada una con un **QR en una esquina de abajo**. Le decimos al robot a quién
buscar; lee el QR de esa foto, vuelve a A y **dice la palabra del QR**. Límite: **3 min por carrera**.
Choques y respuestas equivocadas tienen penalidad.

| Tarea | Navegación | Reconocimiento |
|---|---|---|
| **A** (autónoma) | LiDAR + odometría, sin intervención humana | Cámara del robot: busca a la persona entre las fotos de la meta y lee su QR |
| **B** (gestos) | Gestos de una persona captados por la webcam de la PC (sin teclado, ratón ni mando) | Igual que en A |

### Estado (16/11)

| | Hecho | Falta |
|---|---|---|
| **Tarea A** ([carrera-1](task-a/carrera-1/README.md)) | Navegación rápida sin choques en salas con cajas, flechas, varias fotos + QR (`--buscar`), volver exacto a A, video del recorrido con la cámara del robot de ida y vuelta. Simulador con el aspecto del laboratorio (cajas de cartón, piso gris), croquis fiel y señales de colores leídas por su forma. Laberintos largos (`laberinto:N`) con señales en cada giro. En simulación: 28/28 con 6 fotos y QR (croquis, `lab:N`, `denso:N`, 12 laberintos), 0 choques, 0 palabras equivocadas | **Probar con el robot real** (guía en el README de carrera-1); medir CPU si se quiere correr en la Raspberry |
| **Tarea B** ([gestos-1](task-b/gestos-1/README.md)) | Conducir con gestos de la mano (webcam de la PC) | Reconocer a la persona, leer su QR y anunciarlo (se puede reutilizar `task-a/carrera-1/qr.py` y `reconocimiento/`) |

Arquitectura: la PC habla con el robot por **UDP** (no necesita ROS). En el robot corre un puente
que traduce a ROS 2 y aplica un **freno de emergencia por LiDAR**. Detalles en
[docs/comunicacion_udp.md](docs/comunicacion_udp.md).

## Estructura

```text
turtlebot67/
├── task-a/            Tarea A (autónoma)
│   ├── carrera-1/     ★ La solución de competencia: A → B → A rápido, flechas, varias fotos + QR, simulador, video
│   ├── auto-1/        Exploración por fronteras y cobertura de cámara (plan B de carrera-1)
│   ├── slam-1/        Mapa 2D en la PC (odometría o BreezySLAM)
│   └── camara/        Lanzar la cámara solo en color (apaño USB) y con imagen HD
├── task-b/gestos-1/   Tarea B: conducir con gestos de la mano (webcam de la PC)
├── reconocimiento/    Reconocimiento facial común a A y B (YuNet + ArcFace con OpenCV, galería, votación, voz)
├── datos/rostros/     Fotos de las 17 personas posibles en B (una carpeta por persona)
├── tb4/               Librería común: protocolo UDP + cliente (from tb4 import TB4Client)
├── robot/             Lo que corre o se gestiona en el robot: puente, despliegue, inicio de sesión, cámara
├── herramientas/      Utilidades para la PC (monitor del LiDAR y del estado)
├── ejemplos/          Maniobras de ejemplo (esquivar.py)
├── tests/             Pruebas sin robot (protocolo, reconocimiento)
├── docs/              Enunciado, guía del robot del grupo 3, protocolo UDP
└── setup/             Repositorio inicial: instalar ROS 2, configurar y diagnosticar el robot por ROS
```

## Inicio rápido

Todos los comandos, **desde la raíz del repositorio**.

```bash
pip install -r requirements.txt     # PC: Python 3.10+ con numpy y OpenCV (no hace falta ROS)
robot/tb4_inicio.sh                 # 1 persona, tras encender el robot: hora + ROS + puente
python3 -m tb4.client               # prueba de conexión
python3 -m herramientas.monitor     # ver el LiDAR (añade --drive para conducir con WASD)
python3 -m unittest discover tests  # pruebas sin robot
```

Tarea A ([task-a/carrera-1](task-a/carrera-1/README.md)):

```bash
python3 task-a/carrera-1/carrera.py --sim                 # probar en el simulador (g = empezar)
python3 task-a/carrera-1/banco_pruebas.py --n 20          # 40 carreras simuladas en circuitos aleatorios
python3 task-a/carrera-1/probar_vision.py                 # con el robot: ver flechas, caras y nombres (sin moverlo)
python3 task-a/carrera-1/carrera.py --sim --mundo croquis+6 --go --video   # reglas del lab: 6 fotos con QR
python3 task-a/carrera-1/carrera.py --sim --mundo lab:3+6 --go --video    # meta en otra pared, sala con cajas al azar
python3 task-a/carrera-1/carrera.py --sim --mundo laberinto:3+6 --go --video   # laberinto largo 7x6 con senales en cada giro
python3 task-a/carrera-1/probar_vision.py --qr --hd       # con el robot: ver QR y de que cara es cada uno
python3 task-a/carrera-1/carrera.py --undock --buscar "Nombre Apellido"   # COMPETENCIA: arranca en pausa, g = empezar
```

Antes de competir, seguir la guía [Preparar y probar en el laboratorio](task-a/carrera-1/README.md#preparar-y-probar-en-el-laboratorio)
(todo está validado solo en simulación).

Reconocimiento facial: [reconocimiento/README.md](reconocimiento/README.md). Tarea B:
[task-b/gestos-1](task-b/gestos-1/README.md).

Antes de nada: red, IP, reglas para usar el robot entre varios y problemas conocidos en
**[docs/robot_grupo3.md](docs/robot_grupo3.md)**. Para instalar ROS 2 o reconfigurar el robot desde
cero: [setup/README.md](setup/README.md).

## Ramas

`master` es la versión buena: todo lo de la Tarea A (carrera-1) y la Tarea B (gestos) está ahí. Se trabaja
en una rama nueva desde `master` y se une con un pull request.

| Rama | Qué tiene | Estado |
|---|---|---|
| `master` | Todo lo vigente | ★ usar esta |
| `task-a/carrera` | carrera-1 de Diego | Igual a `master` (ya unida) |
| `task-a/carrera-mejoras` | Mejoras de Alvaro (PR #1) | Ya unida a `master`: se puede borrar |
| `task-a/qr-persona` | Varias fotos + QR, `probar_vision.py --qr` | Para unir a `master` |
| `task-b/gestos` | Gestos de Sofía | Los gestos están en `master` desde el 9/11. Su último commit ("add: simulation") es de la Tarea A sobre una versión vieja de auto-1 y choca con `master`: se archiva con el tag `archivo/task-b-gestos` |
| `manzana-zanabu` | laberinto-1 (Marcelino y Sofía): señales por color, QR, auto-1 con registro de caras | **No unir a `master` tal cual**: renombra `coverage.py` (rompe carrera-1), sube ~68 MB de resultados y modelos, y su README de `fast-set-up` tiene contraseñas (el repo es público). El lector de QR se pasó a carrera-1 (`qr.py`). Se archiva con el tag `archivo/manzana-zanabu` |

## Reglas rápidas

- No cambies la hora del robot a mano ni reinicies solo `turtlebot4.service`: usa `robot/tb4_inicio.sh`.
- Solo una persona conduce a la vez; avisa antes de mover el robot.
- En el dock el robot apaga el LiDAR y la cámara: sácalo con `undock` para probar.
