# Tarea A · camara · Cámara solo en color (apaño para el X_LINK_ERROR)

## El problema (diagnosticado el 2026-11-07)

La OAK-D conecta por USB 3, configura su pipeline y, **al empezar a transmitir, el enlace USB se cae**
(`X_LINK_ERROR` en `stereo_stereo`, `imu_imu`…); el kernel registra `USB disconnect` y la cámara se reinicia
en bucle. No es software: con un mal contacto llegó a desaparecer del bus. Lo más probable es el cable
USB-C (calidad a velocidad USB 3) o un pico de consumo al transmitir. Solución real: **otro cable USB-C a
USB-A 3.0, corto y bueno**.

## El apaño

`camara_rgb.sh` lanza la cámara **solo con la imagen en color** (sin la profundidad estéreo) y en **USB 2**:
manda mucho menos datos. Para detectar caras basta (se usa la imagen en color de 250×250). **La navegación no
cambia**: usa el LiDAR, que es otro aparato.

```bash
task-a/camara/camara_rgb.sh                 # color + USB 2
task-a/camara/camara_rgb.sh --usb SUPER     # color en USB 3
task-a/camara/camara_rgb.sh --estado        # solo mirar: proceso, imágenes por segundo, log
task-a/camara/camara_rgb.sh --normal        # volver a la cámara de siempre
```
IP: `TB4_IP=192.168.0.x task-a/camara/camara_rgb.sh` (por defecto 192.168.0.104). Pide la contraseña SSH.

No toca nada del robot: los parámetros van a `~/tb4_udp/task_a/oakd_rgb.yaml` (copia de
`turtlebot4_bringup/config/oakd_lite.yaml` cambiando solo `i_pipeline_type` e `i_usb_speed`) y la cámara se
lanza aparte, como `robot/reiniciar_camara.sh`. **Al reiniciar el robot, todo vuelve a ser como siempre.**
Mientras esté activo, la profundidad de la cámara no está disponible para nadie: avisa al grupo.
