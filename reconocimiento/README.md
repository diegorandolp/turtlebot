# Reconocimiento facial (Tareas A y B)

¿Quién es la persona de la foto en el punto B? Lo usan la Tarea A ([carrera-1](../task-a/carrera-1))
y la Tarea B (el reconocimiento es el mismo). Todo corre en la PC con **OpenCV** (`cv2.FaceDetectorYN` y
`cv2.dnn`): no hace falta onnxruntime, insightface ni GPU.

```text
foto del robot ──► YuNet ──► 5 puntos ──► alinear 112x112 ──► ArcFace ──► vector 512-D ──► similitud ──► votación
 (250x250 o HD)    (caras)   (ojos, nariz,  (plantilla de       (+ espejo)                  con la galería  entre fotos
                              boca)          ArcFace)                                       (17 personas)
```

1. **Detección (YuNet)**: encuentra la cara y 5 puntos. Las imágenes chicas (la preview de 250×250) se
   amplían ×2 antes de detectar: así aparecen caras de hasta ~12 px.
2. **Alineación**: con los 5 puntos, la cara se gira y escala a 112×112 tal como se entrenó ArcFace.
3. **ArcFace**: convierte la cara en un vector de 512 números. Se suma el de la imagen espejada (más estable).
   - `mbf` (por defecto si no hay R50): MobileFaceNet entrenado en WebFace600K, 13 MB, **está en el repo**.
   - `r50`: ResNet-50 (WebFace600K), 174 MB, mejor con caras muy chicas. No cabe en el repo (ver abajo).
   - `auto` (lo que usa carrera-1): `r50` si está instalado, si no `mbf`.
4. **Galería**: un vector por foto de [`datos/rostros/`](../datos/rostros) (17 personas, 94 fotos). Para cada
   persona cuenta su foto más parecida (vecino más cercano).
5. **Votación** (`Votacion`): cada foto del robot suma log-probabilidades `softmax(similitud / 0.04)`,
   pesadas por el tamaño de la cara (una cara de 14 px pesa 0.15; desde 56 px, 1). Se decide cuando el
   líder supera 0.97 de probabilidad con al menos 2 fotos, o al juntar 8 fotos. **Conjunto cerrado**: la
   respuesta siempre es una de las 17 personas, nunca "desconocido" (la persona de B es de la clase).

## Qué tan bien funciona

`python -m reconocimiento.evaluar`: deja una foto fuera de la galería, la "imprime" y la encuadra como la
vería la cámara del robot (250×250, cara de N px, menos contraste, tinte de luz, algo de perspectiva,
desenfoque, ruido y JPEG; ver `degradar.py`) y la reconoce contra las demás. 94 fotos × 3 cuadros por tamaño:

| Cara (px) | Detecta | `mbf` 1 foto | `mbf` voto de 3 | `r50` 1 foto | `r50` voto de 3 |
|---:|---:|---:|---:|---:|---:|
| 14 | 99 % | 79 % | 86 % | 88 % | 93 % |
| 18 | 100 % | 91 % | 96 % | 95 % | 97 % |
| 24 | 100 % | 98 % | 99 % | 99 % | 100 % |
| 32 | 100 % | 99 % | 100 % | 99 % | 100 % |
| 48 | 100 % | 99 % | 100 % | 100 % | 100 % |
| 64–96 | 100 % | 100 % | 100 % | — | — |

Con la preview de 250×250, una cara impresa de ~10 cm mide ~40 px a 1 m y ~20 px a 2 m: por eso
carrera-1 se acerca hasta que la cara tenga ~46 px, votando con cada foto en el camino. Agregar a la
galería versiones de baja resolución (`--aumentar`) **no** mejoró (probado: 84 % y 94 % a 14 y 18 px).

Las fotos de cada persona se tomaron el mismo día: una foto distinta (otra ropa, otra luz) será algo más
difícil que esta prueba. Si se puede, agreguen 1–2 fotos más por persona con otra ropa y luz.

## Uso

Desde la raíz del repositorio, con el entorno virtual activado:

```bash
python -m reconocimiento.galeria                    # reconstruye la galería (tras agregar fotos)
python -m reconocimiento.galeria --montaje caras.jpg # y guarda las caras alineadas, para revisarlas
python -m reconocimiento.evaluar                    # la tabla de arriba (mbf); --modelo r50, --frontal
python -m reconocimiento.voz "Prueba de voz"        # probar el anuncio en voz alta
python task-a/carrera-1/probar_vision.py            # en vivo con la cámara del robot (sin moverlo)
python -m unittest discover tests                    # pruebas sin robot
```

```python
from reconocimiento import Reconocedor, Votacion, peso_por_tamano
rec = Reconocedor("auto")
r = rec.mejor(img_bgr)                     # la cara más grande, o None
if r:
    print(r.nombre, r.sim, r.margen)       # nombre, similitud y ventaja sobre el segundo
```

**Agregar a alguien:** crear `datos/rostros/Nombre_Apellido/` con 3–10 fotos (de frente y algo de lado,
buena luz) y correr `python -m reconocimiento.galeria` (y `--modelo r50` si usan R50). Se anuncia el nombre
de la carpeta con espacios. Las fotos se reducen a 1024 px y sin EXIF antes de subirlas (el repo es público).

**Instalar R50** (opcional, con internet, una vez): bajar `buffalo_l.zip` de
`https://github.com/deepinsight/insightface/releases/download/v0.7/buffalo_l.zip`, descomprimirlo en
`~/.insightface/models/buffalo_l/` (basta `w600k_r50.onnx`) y correr
`python -m reconocimiento.galeria --modelo r50`. La galería de R50 ya está en el repo; solo falta el modelo.

**Voz:** el TurtleBot 4 Lite no tiene parlante para voz: el nombre se anuncia en la PC (`voz.py` prueba
`espeak-ng`, `spd-say`, PowerShell en Windows/WSL o `say` en macOS) y además se muestra en grande en la
ventana. En Linux: `sudo apt install espeak-ng` (Arch: `sudo pacman -S espeak-ng`).

| Archivo | Qué hace |
|---|---|
| `rostros.py` | Detector, alineación, ArcFace, galería, `Reconocedor` y `Votacion` |
| `galeria.py` | Construye `modelos/galeria_<modelo>.npz` desde `datos/rostros/` |
| `evaluar.py` | Evaluación dejando una foto fuera, como la vería el robot |
| `degradar.py` | Simula la cámara del robot sobre una foto impresa |
| `voz.py` | Anuncio en voz alta desde la PC |
| `modelos/` | YuNet (230 KB), ArcFace MobileFaceNet (13 MB), galerías `mbf` y `r50` |
