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
├── tb4/            Librería común: protocolo UDP + cliente (from tb4 import TB4Client)
├── robot/          Lo que corre o se gestiona en el robot: puente, despliegue, inicio de sesión, cámara
├── herramientas/   Utilidades para la PC (monitor del LiDAR y del estado)
├── ejemplos/       Maniobras de ejemplo (esquivar.py)
├── tests/          Pruebas sin robot
├── docs/           Enunciado, guía del robot del grupo 3, protocolo UDP
├── setup/          Repositorio inicial: instalar ROS 2, configurar y diagnosticar el robot por ROS
└── reference/      Código de otros grupos, solo como referencia (no se importa)
```

Por venir: módulos de la Tarea A (navegación autónoma), la Tarea B (gestos) y el reconocimiento facial.

## Inicio rápido

Todos los comandos, **desde la raíz del repositorio**.

```bash
pip install -r requirements.txt     # PC: Python 3.10+ con numpy y OpenCV (no hace falta ROS)
robot/tb4_inicio.sh                 # 1 persona, tras encender el robot: hora + ROS + puente
python3 -m tb4.client               # prueba de conexión
python3 -m herramientas.monitor     # ver el LiDAR (añade --drive para conducir con WASD)
python3 -m unittest discover tests  # pruebas sin robot
```

Antes de nada: red, IP, reglas para usar el robot entre varios y problemas conocidos en
**[docs/robot_grupo3.md](docs/robot_grupo3.md)**. Para instalar ROS 2 o reconfigurar el robot desde
cero: [setup/README.md](setup/README.md).

## Reglas rápidas

- No cambies la hora del robot a mano ni reinicies solo `turtlebot4.service`: usa `robot/tb4_inicio.sh`.
- Solo una persona conduce a la vez; avisa antes de mover el robot.
- En el dock el robot apaga el LiDAR y la cámara: sácalo con `undock` para probar.
