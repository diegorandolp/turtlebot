# Tarea B · gestos-1 · Conducir el robot con gestos (webcam de la PC)

Una persona frente a la **webcam de la computadora** conduce el TurtleBot 4 con la mano, sin teclado ni mando.

Dos tipos de gesto: **señalar** con el índice solo **gira** (y se queda quieto); **V / 3 dedos** hacen **avanzar**.

| Gesto | Qué hace el robot |
|---|---|
| ☝️ Señalar a la **derecha / izquierda** | Gira **90°** hacia ese lado |
| ☝️ Señalar en **diagonal hacia arriba** ↗ ↖ | Gira **45°** hacia ese lado |
| ☝️ Señalar en **diagonal hacia abajo** ↘ ↙ | Gira **135°** hacia ese lado |
| ☝️ Señalar hacia **abajo** | **Da la vuelta** (180°) |
| ✌️ **V**: índice + corazón hacia arriba | **Avanza lento** (0.07 m/s) |
| 🤟 **3 dedos** (índice + corazón + anular) hacia arriba | **Avanza rápido** (0.15 m/s) |
| ✋ **Mano abierta hacia arriba** (señal de stop) | **Se para** y espera |
| Sin mano / otro gesto | Sigue haciendo lo que estaba haciendo |

Los giros son **relativos a hacia donde mira el robot** (como si tú fueras el robot). La imagen se ve como un
espejo (vista selfie): "derecha" es **tu** derecha, y el robot gira a **su** derecha.

Reglas:
- Al terminar un giro el robot **se queda quieto**: para avanzar haz V o 3 dedos.
- **Señalar mientras avanza** lo para y gira. **V / 3 dedos mientras gira**: termina el giro y luego avanza.
- Mientras avanza, pasar de V a 3 dedos (o al revés) solo cambia la velocidad.
- Mantener la misma señal no repite el giro. **Bajar la mano y volver a señalar sí** gira otra vez.
- Los giros seguidos se suman sobre el rumbo objetivo, no sobre donde quedó el robot: dos veces derecha =
  180° exactos, sin acumular error. Después de un STOP se cuenta desde donde esté el robot.
- STOP tiene prioridad: se confirma más rápido (3 cuadros) y para el robot aunque esté a mitad del giro.

## Cómo funciona

1. **MediaPipe HandLandmarker** (`modelos/hand_landmarker.task`, 7.8 MB) encuentra la mano y da 21 puntos.
2. **`gestos.py`** clasifica con geometría simple: un dedo está *estirado* si su punta está claramente más
   lejos de la muñeca que su articulación media, y *doblado* si está más cerca. El pulgar no cuenta.
   - Señalar: solo el índice estirado. Su dirección (nudillo → punta) se compara con 7 direcciones
     (cada 45°, todas menos arriba) con ±20° de margen; entre dos direcciones queda un hueco que no cuenta.
     El índice hacia arriba no es un gesto.
   - V / 3 dedos / STOP: 2, 3 o 4 dedos estirados (desde el índice) apuntando hacia arriba (±40°).
     Hacia los lados o abajo no cuentan.
3. **Estabilizador**: un gesto se confirma si aparece en 6 de los últimos 8 cuadros (STOP: 3 de 8).
4. **`control.py`**: estados `PARADO`, `GIRANDO` (en el sitio con la odometría, hasta ±3° del rumbo objetivo)
   y `AVANZANDO` (recto, corrigiendo el rumbo). El robot recibe `move(v, w)` por UDP y conserva el
   **freno de emergencia por LiDAR** del puente (0.28 m al frente).

| Archivo | Qué hace |
|---|---|
| `navegar_gestos.py` | **Programa principal**: webcam + gestos + robot real o simulador (`--sim`) |
| `probar_gestos.py` | Solo reconocer gestos, sin robot (para practicar y depurar) |
| `gestos.py` | Clasificador de gestos + estabilizador |
| `control.py` | Del gesto a la velocidad (girar, avanzar, parar) |
| `mano.py` | Detector MediaPipe, cámara y dibujo de la mano |
| `test_gestos.py` | Pruebas sin cámara ni robot (manos sintéticas + control en bucle cerrado) |

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

- Arranca **PARADO**: no avanza hasta que hagas V o 3 dedos.
- **Sin mano, el robot sigue avanzando**: haz STOP (o pulsa ESPACIO) antes de bajar la mano si quieres
  que se quede quieto. El freno del puente para ante obstáculos al frente, pero no vigila costados ni atrás.
- Velocidad máxima 0.20 m/s (el programa no acepta más). Un humano vigilando siempre.
- Si la cámara deja de dar imagen o cierras el programa, el robot se para.
- Avisa al grupo: solo una PC conduce a la vez. Con `--undock` el robot retrocede solo: deja espacio detrás.
