# TurtleBot 4 · Proyecto final de Computer Vision (grupo 3)

**Autonomous and Gesture-Controlled Navigation Challenge** ([enunciado](docs/enunciado_proyecto.pdf)):
el robot va del punto A al punto B esquivando obstáculos, reconoce a la persona de una foto en B y
vuelve a A anunciando su nombre.

| Tarea | Navegación | Reconocimiento |
|---|---|---|
| **A** (autónoma) | LiDAR + odometría, sin intervención humana | Cámara del robot en el punto B |
| **B** (gestos) | Gestos de una persona captados por la webcam de la PC (sin teclado, ratón ni mando) | Igual que en A |

Arquitectura: la PC habla con el robot por **UDP** (no necesita ROS). En el robot corre un puente
que traduce a ROS 2 y aplica un **freno de emergencia por LiDAR**. Detalles en
[docs/comunicacion_udp.md](docs/comunicacion_udp.md).

## Estructura

```text
turtlebot67/
├── task-a/            Tarea A (autónoma)
│   ├── carrera-1/     ★ La solución de competencia: A → B → A rápido, flechas, reconocimiento, simulador
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
python3 task-a/carrera-1/carrera.py --undock              # COMPETENCIA: arranca en pausa, g = empezar
```

Antes de competir, seguir la guía [Preparar y probar en el laboratorio](task-a/carrera-1/README.md#preparar-y-probar-en-el-laboratorio)
(todo está validado solo en simulación).

Reconocimiento facial: [reconocimiento/README.md](reconocimiento/README.md). Tarea B:
[task-b/gestos-1](task-b/gestos-1/README.md).

Antes de nada: red, IP, reglas para usar el robot entre varios y problemas conocidos en
**[docs/robot_grupo3.md](docs/robot_grupo3.md)**. Para instalar ROS 2 o reconfigurar el robot desde
cero: [setup/README.md](setup/README.md).

## Reglas rápidas

- No cambies la hora del robot a mano ni reinicies solo `turtlebot4.service`: usa `robot/tb4_inicio.sh`.
- Solo una persona conduce a la vez; avisa antes de mover el robot.
- En el dock el robot apaga el LiDAR y la cámara: sácalo con `undock` para probar.
