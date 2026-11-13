# Tarea A · carrera-1 · De A a B y de vuelta, lo más rápido posible

El robot sale de **A**, encuentra la foto de **B** (con o sin flechas que indiquen el camino), reconoce a
la persona, **vuelve a A** y anuncia su nombre. La nota de la competencia depende del **tiempo** frente a
los demás grupos, así que todo está pensado para ir rápido: es un método **arriesgado**, con los riesgos
acotados por varias redes de seguridad. Todo corre en la PC; el robot recibe `move(v, w)` por UDP y
conserva el freno de emergencia por LiDAR del puente.

**En el simulador**, 120 carreras en circuitos aleatorios que el programa no conoce (semillas 1–60, cada
circuito con flechas y sin flechas; `banco_pruebas.py --n 60`):

| | Éxitos | Choques | Nombre equivocado | Tiempo total (mediana · p90 · máx) | Ida (mediana) | Vuelta (mediana) |
|---|---:|---:|---:|---|---:|---:|
| Con flechas | 60/60 | 0 | 0 | 53 s · 71 s · 154 s | 26 s | 26 s |
| Sin flechas | 60/60 | 0 | 0 | 53 s · 75 s · 133 s | 28 s | 26 s |

Éxito = vuelve a menos de 35 cm de A con el nombre correcto; la foto de B es una foto de la galería que
se excluye al reconocer (el reconocedor nunca la vio). Son circuitos de 2–4 tramos (de 4 a 13 m de camino de A a B).

## La estrategia (y dónde está el riesgo)

| Decisión arriesgada | Cómo se acota el riesgo |
|---|---|
| Velocidad máxima del robot (0.30 m/s ida, 0.31 vuelta) y giros a 1.8 rad/s | Cada paso se prueban 114 arcos de 1.2 s contra el LiDAR (ventana dinámica): ninguno pasa a menos de 20 cm de nada ni mete un obstáculo en la zona de freno del puente en ningún momento. Detrás, el freno del puente |
| Pasa por huecos angostos (entre cajas, junto a la pared) | Si en 3 s no avanza 6 cm, se desvía 2.5 s hacia el carril más abierto y retoma; si el robot ya quedó pegado a algo, se premian los arcos que lo despegan |
| Sin barrido inicial ni mapa previo: sale directo por el espacio más abierto | Si 25 s no descubre sitios nuevos, pasa al explorador sistemático de auto-1 (plan B) |
| Las flechas cambian el rumbo apenas se confirman (2 fotos) | El giro se "arma" y se ejecuta solo cuando aparece una abertura de ese lado cerca de la señal; si no aparece en 12 s, se ignora la flecha |
| Reconoce **mientras se acerca**, sin detenerse a mirar | Votación entre fotos pesada por tamaño de cara; decide con probabilidad ≥ 0.97 o 8 fotos, **nunca sin votos** |
| Conjunto cerrado: siempre anuncia a alguien de la galería, nunca "desconocido" | La persona de B es de la clase (confirmado). 0 nombres equivocados en las 120 carreras simuladas |
| Vuelve por la ruta **más corta** del mapa de la ida (no desanda) | Si el mapa no da ruta o no avanza en 8 s: ruta con margen chico y, si no, el camino de ida acortado con rectas seguras |
| `--temprano`: vuelve apenas sabe quién es (a ≤ 1.3 m de B), sin terminar de acercarse | Desactivado por defecto: no sabemos si el jurado exige llegar más cerca de B |
| `--hd`: fotos 1280×720 para ver la cara y las flechas ~3 veces más lejos | Desactivado por defecto: no está probado con el robot real (sí en simulación, también por el puente) |

## Cómo funciona

```text
cámara (~7-12 fotos/s) ─► flechas.py ─► señales en el mapa (posición + sentido votado) ─┐
                     └─► reconocimiento/ ─► cara en el mapa + votación de quién es ─────┤
LiDAR (7 Hz) + odometría ─► mapa (slam-1) y "carriles" libres ──────────────────────────┤
                                                                                         ▼
                 navegador.py: IDA ─► ACERCAR ─► RECONOCER ─► VOLVER ─► HECHO (anuncio)
                               └──── EXPLORAR (plan B, auto-1) ◄── 25 s sin sitios nuevos
                                                                                         │
                                         ventana dinámica (arcos seguros) ─► move(v, w) ─┘─► puente (freno)
```

1. **Percepción** (`percepcion.py`): de cada foto salen las caras (YuNet) y las flechas, con su rumbo y su
   posición en el mapa (rumbo + distancia del LiDAR en ese rumbo: la foto y las señales están en una
   superficie). Con fotos tomadas girando rápido no se mueven las posiciones (foto y scan no coinciden).
2. **Flechas** (`flechas.py`): detector **genérico**, sin entrenamiento con ninguna señal concreta:
   - "Tinta": lo bastante más oscuro que su entorno en `min(B, G, R)`.
   - Cada mancha con forma plausible, **rodeada por los 4 lados de papel** claro, neutro y parejo, es
     candidata (descarta el fondo del aula, cajas de colores, el zócalo, tableros).
   - Se compara con un banco de **flechas sintéticas** dibujadas en el código (rectas, de giro como la
     señal de tránsito, curvas, chevrones, gruesas y finas) a la derecha, su espejo a la izquierda y las
     rectas hacia arriba/abajo, y con formas que no son flechas (aro, prohibido, cruz, disco). El aro de
     una señal circular se quita si la flecha está pegada a él.
   - Validado con 540 fotos reales de la cámara del TurtleBot (señales circulares de giro, de otros
     semestres, solo para medir; no están en el repo): **89 % / 86 %** de flechas izquierda/derecha
     detectadas por foto, **97 %** de fotos sin flecha bien rechazadas y **0 %** con el sentido equivocado.
   - Las flechas de color (no negras) se aceptan solo con `--flechas-color` (más falsos positivos).
3. **Navegación** (`navegador.py`), estados:
   - **IDA**: en cada paso se miden los "carriles" (rumbos por los que el robot cabe, con el LiDAR) y se
     elige el que tiene espacio, sigue la dirección de avance (o la de una flecha) y **no lleva a donde ya
     estuvo** (todo lo que cubrió el robot, no solo su centro: así sale de una rama sin salida y no repite
     caminos). La **ventana dinámica** convierte ese rumbo en (v, w): el arco seguro que más avanza hacia él.
   - **ACERCAR**: con 2 detecciones de cara, va hacia ella (apuntando a donde la ve en la foto) votando
     quién es; para cuando la cara mide ≥ 46 px a ≤ 1.2 m, a 0.55 m, o si la cara toca el borde de arriba
     (más cerca se saldría de la foto). Si en 6 s no logra acercarse (cajas en el medio) va por una ruta
     del mapa a un punto frente a la foto. Si la pierde de vista, va por el mapa a donde la vio y gira a
     buscarla; recién a los 30 s sin verla vuelve a explorar. La posición de B es la mediana de las
     últimas detecciones (una mala medida no la arrastra).
   - **RECONOCER**: quieto, centrando la cara, hasta que la votación decide (casi siempre ya decidió en el
     camino). Sin votos no decide: se acerca más.
   - **VOLVER**: ruta más corta a A (Dijkstra de auto-1 sobre el mapa de la ida), a 0.31 m/s.
   - **HECHO**: en A (a < 22 cm): para, anuncia el nombre en voz alta y en grande en la ventana.
   - **EXPLORAR** (plan B): el explorador de fronteras y cobertura de cámara de auto-1.
4. **Reconocimiento**: ver [reconocimiento/README.md](../../reconocimiento/README.md) (ArcFace con OpenCV,
   galería de 17 personas, ≥ 98 % de acierto por foto desde caras de 24 px).

## Pruebas en simulación

Tres niveles, del más rápido al más completo:

| Nivel | Qué prueba | Comando |
|---|---|---|
| Pruebas unitarias | Detector de flechas, carriles, ventana dinámica, estados, simulador, mundos y una carrera completa | `python -m unittest discover -s task-a/carrera-1 -p "test_*.py"` |
| Simulador (`simulador.py`) | La estrategia completa en circuitos aleatorios, con LiDAR, odometría con ruido, freno del puente y **cámara renderizada** (las fotos de B son fotos reales de la galería, excluidas al reconocer) | `python task-a/carrera-1/carrera.py --sim` · `banco_pruebas.py` |
| Simulación por ROS (`sim_ros.sh`) | El **camino real** del día de la competencia: `carrera.py` en modo robot → UDP → **el puente real** → ROS 2 → el simulador haciendo de robot | `task-a/carrera-1/sim_ros.sh` + `carrera.py 127.0.0.1` |

El simulador dibuja la cámara como un *raycaster* (paredes, cajas, cilindros y carteles con su altura
real, cámara a 13 cm del suelo, desenfoque, ruido y JPEG), así se prueba la visión de verdad, no un
"sensor de cara" simulado. Los circuitos aleatorios (`mundos.py`) tienen 2–4 tramos con giros de 90°,
pasillos de 0.95–1.5 m, a veces una rama sin salida, 1–4 obstáculos (cajas y cilindros) y la foto de una
persona al azar en B; con flechas en cada giro (de estilos variados) o sin ellas. Más allá de las paredes
se ve "el aula" (rectángulos de colores y patas de mesas), que pone a prueba los falsos positivos.

## Uso

Desde la raíz del repositorio, con el entorno virtual activado (en la PC de Diego:
`distrobox enter ros-jazzy`, que ya carga ROS y `~/.venvs/tb4`).

```bash
# simulador
python task-a/carrera-1/carrera.py --sim                          # mundo "demo" con ventana; g = empezar
python task-a/carrera-1/carrera.py --sim --mundo aleatorio:12:sin --go
python task-a/carrera-1/carrera.py --sim --mundo sala --hd        # sala abierta, fotos HD
python task-a/carrera-1/banco_pruebas.py --n 40                   # 80 carreras (con y sin flechas), resumen
python task-a/carrera-1/banco_pruebas.py --n 20 --opciones --temprano    # comparar opciones

# simulación por ROS (camino real, sin robot): en una terminal...
task-a/carrera-1/sim_ros.sh demo                                  # (PYTHON=~/.venvs/tb4/bin/python si hace falta)
# ...y en otra
python task-a/carrera-1/carrera.py 127.0.0.1 --go

# robot real
robot/tb4_inicio.sh                                               # hora + ROS + puente (copia el puente nuevo)
python task-a/carrera-1/probar_vision.py                          # ver flechas/caras/nombres sin mover el robot
python task-a/carrera-1/carrera.py --undock                       # arranca EN PAUSA; g = empezar
```

**Teclas** (ventana enfocada): `g` empezar · **espacio = PAUSA (parada inmediata)** · `q`/Esc salir.

La ventana muestra la última foto (flechas en naranja, caras con el nombre, barras de la votación), el
mapa (rastro, ruta de vuelta en verde, señales: naranja = confirmada, gris = usada; estrella = B; rombo =
A; línea amarilla = rumbo elegido) y, en el simulador, la vista real desde arriba. Cada carrera queda en
`task-a/carrera-1/corridas/` (JSON con lo que pasó y cuándo; con `--traza`, también la trayectoria).

| Opción | Defecto | Para qué |
|---|---|---|
| `--vel` | 0.30 | Velocidad máxima de la ida (m/s; la vuelta va a 0.31) |
| `--temprano` | no | Volver apenas se sabe quién es, sin terminar de acercarse a B |
| `--hd` | no | Fotos 1280×720 (cámara lanzada con `task-a/camara/camara_rgb.sh --hd`) |
| `--sin-flechas` | no | Ignorar las flechas |
| `--flechas-color` | no | Aceptar flechas de color (por defecto solo tinta oscura) |
| `--modelo` | auto | Reconocimiento: `r50` si está instalado, si no `mbf` |
| `--hfov` / `--hfov-hd` | 40 / 66 | Campo de visión horizontal (grados) de la preview y de la foto HD |
| `--engine` | odom | Mapa con odometría pura (`breezy` = BreezySLAM, si está instalado) |
| `--max-t` | 300 | Segundos máximos de la carrera |

## Preparar y probar en el laboratorio

Todo lo de esta carpeta se validó **solo en simulación**. Antes de competir hay que comprobar con el robot
real lo que el simulador supone. Los comandos van desde la raíz del repositorio.

### Antes (con internet)

1. `pip install -r requirements.txt`. Opcional: R50, mejor con caras chicas (ver
   [reconocimiento/README.md](../../reconocimiento/README.md)), y `espeak-ng` para la voz
   (`python -m reconocimiento.voz "prueba"`).
2. `python -m unittest discover tests` y una carrera simulada: `python task-a/carrera-1/carrera.py --sim`.
3. Preguntar al profesor, porque decide opciones:
   - ¿Qué tan cerca de la foto cuenta como "llegar a B"? Si basta con ~1 m, `--temprano` ahorra segundos.
   - ¿Cómo son las flechas? Si son de color (no negras), hay que usar `--flechas-color`.
   - ¿A qué altura estará la foto? La cámara está a ~13 cm del suelo y mira horizontal: a 1 m ve hasta
     ~0.5 m de altura. Una foto alta solo se ve de lejos y pequeña.

### En el laboratorio: pruebas antes de competir

4. `robot/tb4_inicio.sh`: corrige la hora, arranca ROS y **copia y arranca el puente nuevo** (hace falta
   para las fotos rápidas y las HD). Si la cámara no da imagen: `task-a/camara/camara_rgb.sh`
   (con `--hd` si van a usar fotos HD).
5. Visión con el robot **quieto**: `python task-a/carrera-1/probar_vision.py` (con `--undock` si está en
   el dock). Comprobar que:
   - llegan fotos (mirar **fotos/s**: el simulador supone ~6–7; con menos, el robot reacciona más tarde);
   - las flechas del circuito salen con el sentido correcto (`izq`/`der`) desde 1–2 m;
   - reconoce la foto de B, o una foto impresa de algún compañero, y la votación llega a ~1.0.

   Si las flechas no se detectan, probar `--flechas-color`. Para medir el detector con las señales reales:
   elegir la clase con `1` izq / `2` der / `3` nada, guardar fotos con `s` (quedan en
   `task-a/carrera-1/fotos/`) y luego `python task-a/carrera-1/evaluar_flechas.py --izq task-a/carrera-1/fotos/izq --der task-a/carrera-1/fotos/der --nada task-a/carrera-1/fotos/nada`.
6. Medir el campo de visión (se supone 40° la preview y 66° la HD): con el robot mirando una pared a
   d = 1 m, poner dos marcas a ±0.30 m del centro, guardar una foto (`s`) y ver en qué columnas caen:
   `hfov = 2·atan(0.30 · ancho / (d · (u_der − u_izq)))` en grados. Si sale distinto, pasarlo con
   `--hfov` (o `--hfov-hd`).
7. Primera carrera **lenta**, mirando la ventana y con la mano en la barra espaciadora:
   `python task-a/carrera-1/carrera.py --undock --vel 0.15 --traza` y `g`. Ver que esquiva sin frenazos,
   toma bien las flechas, se acerca a la foto, dice el nombre correcto y vuelve a A. Luego repetir a la
   velocidad normal (sin `--vel`).
8. Si hay tiempo:
   - **HD**: cámara con `camara_rgb.sh --hd`, `probar_vision.py --hd` (necesita ≥ 4 fotos/s) y una
     carrera con `--hd`. Comparar el tiempo con la preview.
   - **`--temprano`**: solo si el profesor acepta que el robot no llegue pegado a la foto.
9. Cada carrera queda en `task-a/carrera-1/corridas/<fecha>.json` (qué pasó y cuándo; con `--traza`,
   también la trayectoria). Sirve para depurar y para el informe.

### En cada intento

10. Poner el robot en A **mirando hacia el circuito**: la dirección inicial cuenta como "hacia adelante".
11. `python task-a/carrera-1/carrera.py --undock` (sin `--undock` si ya está fuera del dock), más las
    opciones elegidas. Arranca en **pausa**: el robot sale del dock antes de que empiece la carrera, y la
    carrera empieza con `g`.
12. Un humano con la mano en la **barra espaciadora** (pausa inmediata) por seguridad. En la Tarea A
    tocar el robot o el programa termina el intento.
13. Al volver a A, el nombre se dice en voz alta y queda en grande en la ventana hasta pulsar `q`.

## Archivos

| Archivo | Qué hace |
|---|---|
| `carrera.py` | Programa principal (robot real o `--sim`): sensores, cámara, navegador, ventana, anuncio, registro |
| `navegador.py` | Estados y control: carriles, ventana dinámica, flechas, acercarse, votar, volver |
| `percepcion.py` | De cada foto: caras y flechas con rumbo y posición en el mapa; memoria de B y de las señales; cámara en otro hilo (robot real) |
| `flechas.py` | Detector genérico de flechas (banco de flechas sintéticas) |
| `simulador.py` | Robot simulado con LiDAR, freno del puente y cámara renderizada (misma API que `TB4Client`) |
| `mundos.py` | Mundos fijos (`demo`, `demo-sin-flechas`, `sala`) y circuitos aleatorios (`aleatorio:N[:sin]`) |
| `banco_pruebas.py` | Muchas carreras en paralelo y su resumen |
| `robot_ros_sim.py`, `sim_ros.sh` | El simulador como robot ROS 2, con el puente real delante |
| `probar_vision.py` | Ver la percepción en vivo sin mover el robot; guardar fotos para evaluar |
| `evaluar_flechas.py` | Acierto del detector de flechas sobre carpetas de fotos (izq/der/nada) |
| `test_carrera.py` | Pruebas sin robot |

Reutiliza de otras carpetas: el mapa de [slam-1](../slam-1) (`slam_core`), el planificador Dijkstra y el
explorador de fronteras de [auto-1](../auto-1) (`planner`, `coverage`, `explorer`) y el reconocimiento de
[reconocimiento/](../../reconocimiento).

## Límites conocidos (lo que falta comprobar con el robot real)

- **Todo se validó en simulación**, incluido el puente real por ROS 2; nada se probó todavía con el robot.
  Ver [Preparar y probar en el laboratorio](#preparar-y-probar-en-el-laboratorio).
- **Campo de visión supuesto**: 40° para la preview 250×250 (recorte cuadrado de la imagen 16:9) y 66° para
  la HD. Si es otro, los rumbos a las caras y señales salen corridos (se corrige solo al acercarse). Medirlo.
- **La cámara mira horizontal a ~13 cm del suelo**: a 1 m ve hasta ~0.5 m de altura y a 0.6 m solo hasta
  ~0.35 m. Una foto de B colgada alta solo se ve de lejos (pequeña): el reconocimiento con caras de
  14–18 px acierta 86–97 % (mejor con R50). Si la cara toca el borde de arriba, el robot no se acerca más.
- **Obstáculos que el LiDAR no ve** (más bajos que ~17 cm, patas finas, vidrio) no se evitan; el
  parachoques de la Create 3 los detecta al tocarlos.
- **Odometría**: el mapa usa la odometría de la Create 3 (buena en recorridos cortos). En recorridos largos
  con mucho giro, A puede quedar unos centímetros corrida. `--engine breezy` usa BreezySLAM.
- **Flechas**: el detector supone tinta oscura sobre papel claro. Una flecha pintada en el piso, hecha con
  cinta, o de color (sin `--flechas-color`) puede no detectarse; entonces el robot navega como si no hubiera.
- **Fotos HD** (`--hd`): implementadas en el puente (suscripción a `/oakd/rgb/image_raw`) y probadas por ROS
  con el simulador, no con la OAK-D. Con el cable USB problemático del robot, mejor probar antes.
