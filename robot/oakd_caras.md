# Detectar caras EN la cámara (chip Myriad X de la OAK-D) — **no probado todavía con el robot**

La OAK-D Lite trae un chip para redes neuronales (Myriad X). Si la detección de caras corre ahí, la Raspberry
se ahorra YuNet sobre la imagen entera (17 ms por foto en una PC; ×3–5 en la Raspberry) y solo corre YuNet
en el recorte de cada cara que la cámara ya encontró (1–3 ms), que es lo que da los 5 puntos de la cara
para ArcFace. En el simulador (`carrera.py --sim --cam-nn`) la detección pasa de 5.6 s a 1.6 s de CPU por
carrera y el resultado es el mismo.

## Cómo va el dato

```
OAK-D (red face-detection-retail-0004 en el Myriad X)
   └─ /oakd/nn/detections  (vision_msgs/Detection2DArray, cajas)
        └─ robot/tb4_bridge.py --nn-topic /oakd/nn/detections   → mensaje UDP DET (cajas en fracción 0..1)
             └─ tb4.client.det()  → percepcion.procesar(..., cajas=...)  → YuNet solo en los recortes
```

Si la cámara no manda cajas (puente viejo, red no configurada) todo sigue como siempre: YuNet en la imagen
entera. Si manda "ninguna cara", la imagen entera se revisa igual una vez por segundo por si a la red de la
cámara se le escapa una cara chica.

## Qué hay que hacer en el robot (una vez)

1. La cámara del TurtleBot 4 la lanza `turtlebot4_bringup` con `depthai_ros_driver`. Ver con qué parámetros:
   `ros2 param dump /oakd` (busca `camera.i_nn_type` y `nn.i_nn_config_path`). El archivo de parámetros está
   en `/opt/ros/jazzy/share/turtlebot4_bringup/config/` (o `/etc/turtlebot4/`): copiarlo antes de tocarlo.
2. En ese archivo poner:
   ```yaml
   camera:
     i_nn_type: rgb                    # red sobre la imagen de color (sin profundidad)
   nn:
     i_nn_config_path: /home/ubuntu/turtlebot/robot/oakd_caras.json
   ```
   `oakd_caras.json` pide el modelo `face-detection-retail-0004` del zoo de Intel (entrada 300×300, familia
   mobilenet: la misma que el `mobilenet.json` de ejemplo del driver). La primera vez el driver descarga el
   blob por internet (blobconverter): hacerlo con el robot conectado a una red con salida.
3. Reiniciar la cámara (`sudo systemctl restart turtlebot4.service` **solo** vía `robot/tb4_inicio.sh --force`,
   por el tema de la hora) y comprobar: `ros2 topic hz /oakd/nn/detections` y `ros2 topic echo --once /oakd/nn/detections`.
   Si las cajas vienen en píxeles del cuadro de la red (300×300) el puente las convierte con `--nn-ancho/--nn-alto`;
   si vienen en 0..1, las usa tal cual.
4. Arrancar el puente con `robot/deploy_bridge.sh --bg -- --nn-topic /oakd/nn/detections` y verificar desde la
   PC: `python3 -c "from tb4.client import TB4Client; import time; c=TB4Client('192.168.0.103'); c.wait_ready(); time.sleep(1); print(c.det())"`
   frente a una foto: debe imprimir una lista con una caja.

## Si no funciona

- `vision_msgs` no instalado en la Raspberry: `sudo apt install ros-jazzy-vision-msgs`.
- El driver no encuentra el blob: convertirlo en la PC con `pip install blobconverter` →
  `blobconverter --zoo-name face-detection-retail-0004 --zoo-type intel --shaves 6` y en el JSON usar
  `"model_path": "/home/ubuntu/turtlebot/robot/face-detection-retail-0004.blob"` en vez de `model_name`/`zoo`.
- La preview 250×250 del puente y la red ven cuadros distintos (la red mira la imagen de color completa,
  la preview es su recorte central cuadrado): las cajas se dan en fracción del cuadro de la red; si al
  comparar con `probar_vision.py` salen corridas, ajustar `--nn-ancho/--nn-alto` o pedir las fotos HD
  (`--hd`), que sí tienen el cuadro completo.
- Sin esto el sistema funciona igual: es una optimización para que la carrera entera quepa en la Raspberry.
