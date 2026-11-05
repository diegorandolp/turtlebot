# Tarea A · auto-1 · Exploración autónoma hasta encontrar un rostro (punto B)

El robot parte de **A** (donde lo enciendes / lo sacas del dock), **no sabe dónde está B**, y explora solo
hasta que su cámara ve una cara. Ese lugar es **B**. Opcionalmente, después vuelve a A.
Construye sobre [slam-1](../slam-1) (mapa y pose con BreezySLAM). Todo corre en la PC; el robot recibe
`move(v, w)` por UDP y **conserva su freno de emergencia por LiDAR** (0.28 m al frente).

## Cómo funciona (exploración por fronteras)

1. **SLAM** (slam-1) da el mapa de ocupación y la pose del robot.
2. **Frontera** = celda libre pegada a una zona aún desconocida. Ir a la frontera alcanzable más cercana
   revela mapa nuevo; se repite hasta ver la cara o hasta que no queden fronteras.
3. **Ruta**: Dijkstra sobre celdas libres, con las paredes infladas por el radio del robot (0.20 m) y un coste
   extra cerca de ellas (la ruta tiende a ir por el centro de los pasillos). Se recalcula cada ~1 s o si el mapa
   bloquea la ruta, sin cambiar de meta sin motivo.
4. **Seguimiento**: *pure pursuit* a 0.35 m; gira en sitio si la meta está muy de lado; frena al acercarse a un
   obstáculo y, si hay algo a menos de 0.38 m, para y gira hacia el lado más libre.
5. **Atascos**: si en 6 s no avanza 12 cm, descarta esa meta y gira; si lleva 4 s bloqueado, cambia de meta.
6. **Rostro**: cada ~1 s pide una foto a la cámara (250×250) y la pasa por YuNet. Con una detección **para**;
   con dos seguidas la **confirma** (B = pose en ese momento). Un falso positivo suelto se descarta.

| Archivo | Qué hace |
|---|---|
| `explorar.py` | Programa principal (robot real o `--sim`), ventana y teclas |
| `planner.py` | Fronteras y rutas sobre el mapa |
| `explorer.py` | Estados y control (PAUSA, EXPLORAR, CONFIRMAR, VOLVER, HECHO, SIN_B) |
| `face.py` | Detector YuNet y lógica de confirmación |
| `sim.py` | Simulador 2D del robot en un laberinto (mismo API que `TB4Client`) |
| `test_auto.py` | Pruebas sin robot |
| `modelos/` | `face_detection_yunet_2023mar.onnx` (opencv_zoo, 227 KB) |

## Uso (desde la raíz del repositorio, con el entorno virtual activado)

```bash
python -m unittest discover -s task-a/auto-1 -p "test_*.py"          # pruebas
python task-a/auto-1/explorar.py --sim                                # simulador con ventana (g = empezar si no pones --go)
python task-a/auto-1/explorar.py --sim --no-gui --go --png mapa.png   # simulador sin ventana, guarda el mapa final
python task-a/auto-1/explorar.py --undock                             # ROBOT REAL: arranca EN PAUSA; g = empezar
python task-a/auto-1/explorar.py --undock --volver                    # idem, y al encontrar B regresa a A
```

**Teclas** (ventana enfocada): `g` empezar/reanudar · **espacio = PAUSA (parada inmediata)** · `l` LiDAR ·
`m` guardar mapa · `q`/Esc salir (para el robot y guarda el mapa).

Ventana: gris = desconocido, negro = pared, blanco = libre; puntos cian = fronteras, línea verde = ruta,
cruz verde = meta actual, rombo blanco = A, estrella magenta = B, puntos amarillos = LiDAR en vivo.

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
- La cámara apunta al frente: una cara que el robot pase de lado puede no verse. No hay barrido de búsqueda.
- El detector se probó con ruido y en el simulador (que sustituye la cara por una imagen blanca), **no con una cara
  real**. Pruébalo antes con `FaceDetector()(foto)` sobre una foto tuya.
- Si no hay internet para bajar el modelo, copia el `.onnx` a `modelos/` a mano.
