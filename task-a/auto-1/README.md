# Tarea A · auto-1 · Exploración autónoma hasta encontrar un rostro (punto B)

El robot parte de **A** (donde lo enciendes / lo sacas del dock), **no sabe dónde está B**, y explora solo
hasta que su cámara ve una cara. Ese lugar es **B**. Opcionalmente, después vuelve a A.
Construye sobre [slam-1](../slam-1) (mapa y pose con BreezySLAM). Todo corre en la PC; el robot recibe
`move(v, w)` por UDP y **conserva su freno de emergencia por LiDAR** (0.28 m al frente).

## Cómo funciona

**La idea clave:** el LiDAR ve 360°, pero la cámara solo ve un cono hacia delante (60°) y a poca distancia
(~1.5 m: la foto es de 250×250 px y una cara impresa pequeña solo ocupa los 24 px que pide el detector a poco más
de un metro). Que una zona esté en el mapa **no** significa que la cámara la haya mirado. Por eso hay dos cosas
que cubrir: el **mapa** (fronteras) y las **superficies** (paredes y obstáculos, donde puede estar la foto).

1. **SLAM** (slam-1) da el mapa de ocupación y la pose del robot.
2. **EXPLORAR (fronteras):** una *frontera* es una celda libre pegada a zona desconocida. El robot va a la más
   cercana, el LiDAR revela mapa nuevo y se repite. La ruta es Dijkstra sobre celdas libres con las paredes infladas
   por el radio del robot (0.25 m) y coste extra cerca de ellas; se sigue con *pure pursuit* (0.35 m).
3. **Cobertura de cámara** (`coverage.py`): cada foto marca el cono que vio (recortado con el LiDAR: no se ve a
   través de paredes). Se distinguen *superficies vistas* y *pendientes* (puntos rojos en la ventana; verde = visto).
4. **BARRIDO:** al empezar (mirada inicial de 360° desde A) y al llegar a cada frontera, el robot **para y gira en
   el sitio solo hacia los rumbos donde queda algo pendiente**, esperando una foto en cada uno.
5. **INSPECCIONAR:** cuando ya no hay fronteras, va a los mejores puntos de vista (*next-best-view*: el que cubre más
   superficie pendiente por metro de camino) y barre en cada uno, hasta cubrir las superficies.
6. **Rostro:** cada ~0.6 s pide una foto y la pasa por YuNet. Una detección **para**; dos seguidas la **confirman**
   (B = pose en ese momento). Un falso positivo suelto se descarta y se retoma lo que se hacía.
7. **Recuperación:** si en 6 s no avanza 12 cm descarta la meta y gira; si un obstáculo **en el pasillo del robot**
   (±0.22 m) lo bloquea 4 s, cambia de meta. Replanifica cada ~1 s.
8. **Si la cámara no da fotos** no se queda barriendo: cancela el barrido a los 6 s, explora solo con el LiDAR y al
   final se rinde (`SIN_B`) avisando. Con la cámara ya arreglada retoma la inspección.

Resultado del simulador, mismo mundo `lateral` (foto en una pared al costado del pasillo, invisible al ir de frente):
el explorador solo con fronteras **se rindió a los 45 s sin ver la foto**; con cobertura la encontró a los 254 s
(11 barridos, 97 % de las superficies cubiertas, 0 colisiones).

| Archivo | Qué hace |
|---|---|
| `explorar.py` | Programa principal (robot real o `--sim`), ventana y teclas |
| `planner.py` | Fronteras y rutas sobre el mapa |
| `coverage.py` | **Cobertura de la cámara**: qué superficies ya miró y cuáles faltan; barridos y puntos de vista |
| `explorer.py` | Estados y control (PAUSA, EXPLORAR, BARRIDO, INSPECCIONAR, CONFIRMAR, VOLVER, HECHO, SIN_B) |
| `face.py` | Detector YuNet y lógica de confirmación |
| `sim.py` | Simulador 2D del robot (mismo API que `TB4Client`), con cámara de cono limitado y dos mundos: `laberinto`, `lateral` |
| `probar_camara.py` | Prueba la cámara y el detector sin autonomía |
| `test_auto.py` | Pruebas sin robot |
| `modelos/` | `face_detection_yunet_2023mar.onnx` (opencv_zoo, 227 KB) |

## Uso (desde la raíz del repositorio, con el entorno virtual activado)

```bash
python -m unittest discover -s task-a/auto-1 -p "test_*.py"          # pruebas
python task-a/auto-1/explorar.py --sim                                # simulador con ventana (g = empezar si no pones --go)
python task-a/auto-1/explorar.py --sim --no-gui --go --png mapa.png   # simulador sin ventana, guarda el mapa final
python task-a/auto-1/explorar.py --undock                             # ROBOT REAL: arranca EN PAUSA; g = empezar
python task-a/auto-1/explorar.py --undock --volver                    # idem, y al encontrar B regresa a A
python task-a/auto-1/explorar.py --sim --no-gui --go --mundo lateral --engine odom   # simulador, mundo con la foto de lado
python task-a/auto-1/explorar.py --sim --no-gui --go --sin-camara                    # simulador con la cámara caída
python task-a/auto-1/explorar.py --sin-cobertura ...                  # desactiva barridos e inspección (solo fronteras)
```

Opciones de la cámara: `--cam-fov` (60°) y `--cam-rango` (1.5 m) describen lo que *de verdad* ve la cámara; si el
detector alcanza más lejos, súbelos y el robot barrerá menos. Mide el alcance real con `probar_camara.py`.

**Teclas** (ventana enfocada): `g` empezar/reanudar · **espacio = PAUSA (parada inmediata)** · `l` LiDAR ·
`m` guardar mapa · `q`/Esc salir (para el robot y guarda el mapa).

Ventana: gris = desconocido, negro = pared, blanco = libre; **tinte verde = zonas que la cámara ya miró**,
**puntos rojos = superficies aún sin mirar**; puntos cian = fronteras, línea verde = ruta, cruz verde = meta,
flecha naranja = punto de vista objetivo, rombo blanco = A, estrella magenta = B, puntos amarillos = LiDAR en vivo.
La línea de abajo muestra el estado de la cámara y el % de superficies cubiertas.

## Seguridad (léelo antes de probar con el robot real)

- **Arranca en pausa**: no se mueve hasta que pulses `g`. La velocidad máxima es 0.12 m/s (tope 0.20).
- **Un humano vigilando siempre**, con la mano cerca de la tecla espacio. Despeja el espacio detrás antes de
  `--undock`: el robot retrocede solo y atrás no tiene sensores.
- El freno del puente (0.28 m al frente, pasillo de ±0.20 m) funciona aunque este programa se equivoque, pero
  **no vigila los costados ni la parte trasera**. Si el laberinto tiene pasos estrechos, prueba primero en vacío.
- Avisa al grupo: solo una PC conduce a la vez.
- Pasos de ~0.6 m o menos de ancho pueden quedar bloqueados: el robot los trata como pared por su radio de
  seguridad (`--radio`, defecto 0.20 m). Bájalo con cuidado si hace falta.

## Detección de rostros

OpenCV 5 ya no incluye las cascadas Haar, así que se usa **YuNet** (`cv2.FaceDetectorYN`, ~30 ms por foto en CPU).
El modelo está en `modelos/`; si falta, se baja de
`https://github.com/opencv/opencv_zoo/raw/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx`.
Aquí solo se detecta *que hay* una cara; **reconocer a quién** (la foto de B) es la etapa siguiente
(OpenCV trae `FaceRecognizerSF`).

## Límites conocidos

- Lo validado hasta ahora es el simulador (odometría perfecta o con poco ruido). **Con el robot real hace falta
  afinar**: velocidades, `--sigma-xy/--sigma-th` de BreezySLAM, el radio de seguridad y el umbral del detector.
- Sin cierre de bucle en el SLAM: en recorridos largos el mapa puede desviarse, y B quedaría con ese error.
- La cobertura asume el campo de visión y el alcance de `--cam-fov/--cam-rango`; si la cámara real ve menos, habrá
  superficies marcadas como vistas que no lo estaban. **Mídelo con `probar_camara.py`** antes de fiarte.
- Una foto sobre una superficie que el LiDAR no ve (cristal, material absorbente, soporte muy fino) no es una
  "superficie" para el planificador: solo se vería de pasada.
- Falta el reconocimiento de *quién* es (comparar con la foto de referencia y anunciar el nombre) y volver a A
  con eso. `--volver` ya regresa a A al encontrar B.
- El detector se probó con ruido y en el simulador (que sustituye la cara por una imagen blanca), **no con una cara
  real**. Pruébalo antes con `FaceDetector()(foto)` sobre una foto tuya.
- Si no hay internet para bajar el modelo, copia el `.onnx` a `modelos/` a mano.
