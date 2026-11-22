# Tarea B · gestos-1 · Conducir el robot con gestos (webcam de la PC)

Una persona frente a la **webcam de la computadora** conduce el TurtleBot 4 con la mano, sin teclado ni mando.

Tres tipos de gesto: **señalar** con el índice solo **gira** (y se queda quieto); **V / puño** hacen **avanzar**;
**mano abierta** = **STOP** (también a mitad de un giro). Mientras tanto, la cámara del robot busca a la persona y su QR.

| Gesto | Qué hace el robot |
|---|---|
| ☝️ Señalar a la **derecha / izquierda** | Gira **90°** hacia ese lado |
| ☝️ Señalar en **diagonal hacia arriba** ↗ ↖ | Gira **45°** hacia ese lado |
| ☝️ Señalar en **diagonal hacia abajo** ↘ ↙ | Gira **135°** hacia ese lado |
| ☝️ Señalar hacia **abajo** | **Da la vuelta** (180°) |
| ✌️ **V**: índice + corazón hacia arriba | **Avanza lento** (0.15 m/s) |
| ✊ **Puño** (los 4 dedos doblados) | **Avanza rápido** (0.31 m/s, el máximo del robot) |
| ✋ **Mano abierta** en cualquier dirección (también en pleno giro) | **Se para** y espera |
| Sin mano / otro gesto | Sigue haciendo lo que estaba haciendo |

| Tecla (en la ventana) | Qué hace |
|---|---|
| **ESPACIO** | Parada de emergencia (luego hay que hacer una señal nueva) |
| `n` | Olvidar la persona y el QR reconocidos y buscar de nuevo |
| `q` / Esc | Salir (el robot se para) |

> ⚠️ **Sin mano el robot sigue avanzando.** Haz ✋ antes de bajar la mano.

Los giros son **relativos a hacia donde mira el robot** (como si tú fueras el robot). La imagen se ve como un
espejo (vista selfie): "derecha" es **tu** derecha, y el robot gira a **su** derecha.

Reglas:
- Al terminar un giro el robot **se queda quieto**: para avanzar haz V o puño.
- **Señalar mientras avanza** lo para y gira. **V / puño mientras gira**: termina el giro y luego avanza.
- Mientras avanza, pasar de V a puño (o al revés) solo cambia la velocidad.
- Mantener la misma señal no repite el giro. **Bajar la mano y volver a señalar sí** gira otra vez.
- Los giros seguidos se suman sobre el rumbo objetivo, no sobre donde quedó el robot: dos veces derecha =
  180° exactos, sin acumular error. Después de un STOP se cuenta desde donde esté el robot.
- STOP tiene prioridad: se confirma más rápido (2 cuadros) y para el robot aunque esté a mitad del giro: ✋ en pleno giro lo detiene ahí mismo.

## Cómo funciona

1. **MediaPipe HandLandmarker** (`modelos/hand_landmarker.task`, 7.8 MB) encuentra la mano y da 21 puntos.
2. **`gestos.py`** clasifica con geometría simple: un dedo está *estirado* si su punta está claramente más
   lejos de la muñeca que su articulación media, y *doblado* si está más cerca. El pulgar no cuenta.
   - Señalar: solo el índice estirado. Su dirección (nudillo → punta) se compara con 7 direcciones
     (cada 45°, todas menos arriba) con ±20° de margen; entre dos direcciones queda un hueco que no cuenta.
     El índice hacia arriba no es un gesto.
   - Puño: los 4 dedos doblados, en cualquier orientación.
   - V: índice y corazón estirados apuntando hacia arriba (±40°).
   - STOP: los 4 dedos estirados, en cualquier dirección (para cortar un giro sin girar la mano).
     Hacia los lados o abajo no cuentan.
3. **Estabilizador**: un gesto se confirma si aparece en 5 de los últimos 8 cuadros (STOP: 2 de 8).
4. **`control.py`**: estados `PARADO`, `GIRANDO` (en el sitio con la odometría, hasta ±3° del rumbo objetivo)
   y `AVANZANDO` (recto, corrigiendo el rumbo). El robot recibe `move(v, w)` por UDP y conserva el
   **freno de emergencia por LiDAR** del puente (0.28 m al frente por defecto; ver abajo para bajarlo).

| Archivo | Qué hace |
|---|---|
| `navegar_gestos.py` | **Programa principal**: webcam + gestos + robot real o simulador (`--sim`) |
| `probar_gestos.py` | Solo reconocer gestos, sin robot (para practicar y depurar) |
| `gestos.py` | Clasificador de gestos + estabilizador |
| `control.py` | Del gesto a la velocidad (girar, avanzar, parar) |
| `mano.py` | Detector MediaPipe, cámara y dibujo de la mano |
| `test_gestos.py` | Pruebas sin cámara ni robot (manos sintéticas + control en bucle cerrado) |

## Caras y QR (cámara del robot)

Mientras conduces, un hilo pide fotos a la **cámara del robot**, busca una cara, la reconoce con
[`reconocimiento/`](../../reconocimiento/README.md) (votando entre varias fotos) y lee el QR que esté junto a ella
(`task-a/carrera-1/qr.py`). Va en dos pasos, y cada uno se anuncia por voz y queda en la banda de abajo a
la derecha de la ventana:

1. `Persona: Thiago Frias reconocida` → desde ahí **sigue buscando el QR** sin límite de tiempo (acerca el robot a la foto).
2. `QR reconocido: <texto>`.

Con varias fotos juntas (cada una con su QR debajo de la cara), se reconoce **cada cara**, se ubica la de la
persona y **solo se acepta el QR que está debajo de ella**; los QR de las otras fotos se ignoran. Si en la
imagen no se ve la cara de la persona, no se acepta ningún QR.

Arriba a la derecha se ve la última foto del robot con las caras.

**No mueve el robot**: en la Tarea B todo el movimiento es manual, con gestos (también volver al inicio).
`n` = buscar otra cara · `--hd` = fotos HD (ve caras más lejos, más lento) · `--sin-caras` = solo gestos.
El código está en `vigia.py`.

## Uso

**WSL2 no ve la webcam.** Ejecuta los programas con el **Python de Windows** (`python.exe`); funciona
directamente desde la terminal de WSL, en la raíz del repositorio. Instalación (una vez):

```bash
python.exe -m pip install mediapipe opencv-python numpy
```

```bash
python.exe task-b/gestos-1/probar_gestos.py               # 1) practicar los gestos (sin robot)
python.exe task-b/gestos-1/navegar_gestos.py --sim        # 2) conducir el SIMULADOR de task-a
python.exe task-b/gestos-1/navegar_gestos.py --undock     # 3) ROBOT REAL (192.168.0.103), sale del dock
python.exe task-b/gestos-1/navegar_gestos.py --lento 0.05 --rapido 0.12
```

En Linux con cámara se usa `python` normal. Pruebas (sin cámara, en WSL vale el venv):

```bash
python -m unittest discover -s task-b/gestos-1 -p "test_*.py"
```

Opciones: `--cam N` otra cámara · `--no-espejo` · `--margen 22` (más permisivo con el ángulo al señalar; máx. 22.5).

**Teclas** (solo seguridad): **ESPACIO = parada de emergencia** · `q`/Esc salir (el robot se para).
Después de ESPACIO, para volver a moverlo hay que hacer una señal nueva.

La ventana muestra la cámara (gesto detectado y estado del robot) y, a la derecha, la odometría vista
desde arriba: rastro naranja, flecha blanca = hacia dónde mira, flecha verde = rumbo objetivo,
distancia libre al frente y batería.

Si no reconoce bien: buena luz, la mano entera dentro del cuadro, los dedos bien horizontales al señalar
(la mano de lado a la cámara) y la palma de frente para el STOP. La fila de arriba dice qué dedos ve
estirados (`IM..` = índice y corazón = V).

## Seguridad con el robot real

- Arranca **PARADO**: no avanza hasta que hagas V o puño.
- **Sin mano, el robot sigue avanzando**: haz STOP (o pulsa ESPACIO) antes de bajar la mano si quieres
  que se quede quieto. El freno del puente para ante obstáculos al frente, pero no vigila costados ni atrás.
- Velocidad máxima 0.31 m/s (tope de fábrica; el programa no acepta más). Un humano vigilando siempre.
- Si la cámara deja de dar imagen o cierras el programa, el robot se para.
- Avisa al grupo: solo una PC conduce a la vez. Con `--undock` el robot retrocede solo: deja espacio detrás.

### Freno más corto (solo para probar la Tarea B)

El freno lo aplica el **puente en el robot** (`--stop-dist`, 0.28 m desde el centro del LiDAR ≈ 11 cm
desde el parachoques; entre 0.60 m y ese valor frena de forma progresiva). Es compartido con la Tarea A, así
que no se cambia el valor por defecto: se relanza el puente con otro valor y, al terminar, se vuelve a lanzar normal.

```bash
robot/deploy_bridge.sh --bg -- --stop-dist 0.22 --slow-dist 0.45   # avisar al grupo: reinicia el puente
robot/deploy_bridge.sh --bg                                        # al terminar: volver a 0.28 / 0.60
```

No bajes de ~0.22 m: el parachoques está a ~0.17 m del LiDAR y a 0.31 m/s el robot necesita unos cm para parar.

## Comandos con caras y QR

Desde la raíz del repositorio (en WSL con `python.exe`, por la webcam). Cambia la IP si el robot tiene otra.

```bash
# Conducir y reconocer a CUALQUIER persona (anuncia la primera que reconozca + su QR)
python.exe task-b/gestos-1/navegar_gestos.py 192.168.0.104 --undock

# Buscar a UNA persona: las demás caras se descartan y sigue buscando hasta encontrarla
python.exe task-b/gestos-1/navegar_gestos.py 192.168.0.104 --undock --Thiago_Frias
python.exe task-b/gestos-1/navegar_gestos.py 192.168.0.104 --undock --buscar "Sofia Salazar"   # equivalente

# Fotos HD del robot: ve caras y QR más lejos (más lento)
python.exe task-b/gestos-1/navegar_gestos.py 192.168.0.104 --undock --Sofia_Salazar --hd

# Solo gestos, sin caras ni QR
python.exe task-b/gestos-1/navegar_gestos.py 192.168.0.104 --undock --sin-caras

# Simulador (las fotos del simulador son blancas: no reconoce caras, sirve para los gestos)
python.exe task-b/gestos-1/navegar_gestos.py --sim
```

- El nombre es el de la carpeta en [`datos/rostros/`](../../datos/rostros) (no importan tildes, mayúsculas ni `_`).
  Si no existe, el programa termina **antes de mover el robot** y muestra la lista de personas válidas.
- Teclas: `n` = buscar de nuevo (tras un anuncio) · ESPACIO = parada de emergencia · `q` = salir.
- Reconocer sin mover el robot (para probar la cámara): `python task-a/carrera-1/probar_vision.py --qr`.

## Si la cámara va lenta (lag)

- La webcam se abre a 640×480 con MJPG y se lee en un hilo que **siempre entrega el cuadro más reciente**
  (antes se procesaban cuadros viejos del búfer y la imagen iba retrasada).
- El reconocimiento de caras pide ~4 fotos/s al robot. Si aún va lento: `--sin-caras` para conducir solo con gestos.
- Cierra otros programas que usen la webcam (Zoom, Teams, la app Cámara).

### Cámara del robot (fotos para caras y QR)

- Un hilo pide fotos al robot sin parar y otro las procesa: siempre se procesa la **más nueva** (antes se
  pedía, se procesaba y recién entonces se pedía la siguiente). JPEG calidad 70 para que llegue antes por Wi-Fi.
- En la ventana, `[foto NNN ms]` = cuánto tarda cada foto en llegar. Normal: 100–300 ms con buen Wi-Fi.
- Recién salido del dock la cámara **está arrancando** (se apaga en el dock): verás
  `[camara del robot arrancando / sin imagen]` unos segundos. Es normal; saca el robot del dock un rato antes.
- Si tarda mucho: acércate al router, que nadie más esté pidiendo fotos al robot, y no uses `--hd`.
