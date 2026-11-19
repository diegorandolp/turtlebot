# Qué falta probar antes del simulacro (sábado)

← [Robot del grupo 3](robot_grupo3.md) · [carrera-1](../task-a/carrera-1/README.md)

Escrito el 2026-11-17, tras la primera sesión completa con el robot real. Marca `[x]` y anota los
números medidos al lado: sirven para calibrar (`lab.json`) y para el informe.

## ✅ Ya probado con el robot real (2026-11-17)

- [x] Wi-Fi `Lab_Computech_3`, robot encontrado por MAC (`192.168.0.104`), SSH con llave.
- [x] `robot/tb4_inicio.sh --force` → `robot listo en 300 s: bateria 81%, dock True`.
- [x] Puente UDP + `tb4.client` + `herramientas.monitor`: batería, dock, odometría, LiDAR (720 pts).
- [x] Conducir con WASD, cambiar velocidad, undock (`u`) y volver al dock (`k`), foto (`p`).
- [x] Cámara: `camara_rgb.sh --hd` (~21 fps en el robot); preview en `probar_vision.py` (~18 fps).

## ✅ Primera carrera dentro de la Raspberry (2026-11-19)

Circuito: salida frente a una señal "derecha" (~1 m), al doblar un pasillo de cajas con una fila de
fotos con QR en las paredes. Todo en la Raspberry; START por UDP con `robot/control_carrera.py`.

- [x] Flecha vista a 1.7 s, giro a la derecha a 4.5 s, recorrió la fila, reconoció a la persona
      (p = 0.96), **leyó su QR** a 76 s, volvió y paró a **0.21 m de A** (93.9 s, 0 choques).
- [x] Anuncio: llegó al puente; a la PC no llegó (un solo paquete UDP). Quedó en `RESULTADO.txt`.
- [ ] Anunció solo el nombre: el QR se leyó 1 vez y la regla pedía 2 (arreglado, falta re-probar).
- Tiempos muertos: 15–50 s recorriendo la fila (la Pi procesa ~2 fotos/s) y 52–82 s frente al QR
  (cada foto HD tarda ~4 s: 6 fotos en 30 s).

Bugs que solo aparecieron en el robot (arreglados):

| Síntoma | Causa | Arreglo |
|---|---|---|
| `--correr` no imprime nada y la carrera no corre | el `pkill` encontraba al propio shell remoto (su línea contiene el comando) | `pkill -f '^python3 -u task-a/carrera-1/carrera[.]py'` |
| tras START el puente no responde a nadie | deadlock: `on_packet` tiene el lock y `broadcast` lo vuelve a tomar | `threading.RLock` en `tb4_bridge.py` |
| la carrera va sin cámara (`fotos 0`) | el numpy 2 de `~/.local` (instalado para la carrera) rompe `cv_bridge` en el puente | el puente corre con `PYTHONNOUSERSITE=1` |
| la palabra no aparece en la PC | un solo paquete UDP | `ANUNCIO:` se repite 3 s |
| "odometría None" en `--verificar` | se leía antes del primer dato | espera 5 s; ahora también prueba la cámara |

Operación (lo que costó tiempo):

- `tb4_inicio.sh --force` corrige la hora y **reinicia la Create 3: esperar ~5 min** hasta ver `pose (...)` en `tb4.client`.
- Tras reiniciar ROS el LiDAR puede no arrancar (no hay nodo `rplidar_composition`):
  `ssh -t ubuntu@192.168.0.104 "sudo systemctl restart turtlebot4.service"` y esperar ~40 s.
- La cámara se lanza a mano (`camara_rgb.sh --hd`) después de cada reinicio de ROS.
- En el dock el power_saver apaga cámara y LiDAR: verificar y correr **fuera del dock**.
- `--parar` mata la carrera y deja el video sin cerrar: mejor levantar el robot o usar el botón.

Secuencia que funcionó (robot fuera del dock). Desde el 19/11 los 6 primeros pasos (y los arreglos del LiDAR y
la cámara) están en **un solo script**, `robot/preparar.sh`:

```bash
bash robot/preparar.sh --undock                       # inicio, LiDAR, puente, cámara, copiar, verificar (--force: hora)
bash robot/desplegar_carrera.sh --correr "Nombre Apellido"
python robot/control_carrera.py 192.168.0.104        # START; imprime ANUNCIO
bash robot/desplegar_carrera.sh --resultado           # la palabra, si no llegó a la PC
bash robot/desplegar_carrera.sh --video               # (--parar ahora cierra el video y guarda el registro)
```

A mano, lo mismo es: `tb4_inicio.sh` → (si no hay nodo del LiDAR) `sudo systemctl restart turtlebot4.service` →
`deploy_bridge.sh --bg` → `camara_rgb.sh --hd` → `desplegar_carrera.sh` → `desplegar_carrera.sh --verificar`.
Para `--vel 0.45`: `bash robot/preparar.sh --rapido` y `CARRERA_ARGS="--vel 0.45"`.

Al final de cada carrera en el robot el log muestra `[camara] ms promedio (n): foto ..., foto_hd ..., proc ...,
proc_qr_hd ...`: cuánto tarda pedir cada foto al puente y procesarla. Anotarlo: dice si los ~4 s de la HD son del
puente o del procesado.

Respuestas del profe (19/11): las fotos van **pegadas a los costados** del último tramo, nunca en la
pared del fondo; el QR va como lo pusimos (debajo de la foto); solo cuentan los rostros subidos hasta
el 19/11. Sin respuesta aún: cuántas fotos habrá y si las cajas tendrán letras impresas.

## 🏠 Antes del próximo lab (sin robot)

- [ ] Imprimir 3–6 fotos de personas del dataset **con su QR**, del tamaño que se espera en la competencia.
- [ ] Imprimir las señales de giro de cada color; conseguir 2–3 cajas.
- [ ] Plan de internet **para el robot** (solo para `desplegar_carrera.sh --instalar`): anclaje USB del celular a la Raspberry.
- [ ] Preguntar al profe: dónde está la meta B, tamaño/altura de fotos y QR, tiempo límite, cómo se anuncia el nombre.
- [ ] Decidir el anuncio del nombre: PC con `robot/control_carrera.py` o parlante USB en la Raspberry.

## 🤖 Próximo lab, en este orden (cada bloque desbloquea el siguiente)

Inicio de cada sesión: [robot_grupo3.md §3.0](robot_grupo3.md#30-desde-windows-paso-a-paso-verificado-2026-11-17).

### 1. Visión con el robot quieto

`python task-a/carrera-1/probar_vision.py --qr` y luego con `--hd` (antes `bash task-a/camara/camara_rgb.sh --hd`).

| Prueba | Preview 250×250 | HD 1280×720 |
|---|---|---|
| QR impreso a 0.4 m | | |
| QR impreso a 0.6 m | | |
| QR impreso a 1.0 m | | |
| Cara reconocida con nombre correcto a 0.5 m | | |
| Cara reconocida a 1.0 m | | |
| Cara reconocida a 1.5 m | | |
| Señal de color: dirección correcta a 1 m | | |
| Señal de color: dirección correcta a 2 m | | |

- [ ] Campo de visión real (catálogo: 42° preview / 69° HD): medir con una hoja en el borde de la imagen.
- [ ] Altura de la cámara y de las fotos en la pared: ¿la cara queda centrada en la imagen?

### 2. Carrera dentro de la Raspberry

```bash
bash robot/desplegar_carrera.sh                 # copia el código al robot
bash robot/desplegar_carrera.sh --instalar      # UNA vez, con internet en el robot
bash robot/desplegar_carrera.sh --verificar     # librerías, modelos, puente, LiDAR, memoria
bash robot/desplegar_carrera.sh --medir         # % de CPU → decide --cam-periodo y si se usa HD
python robot/control_carrera.py --luz           # luces, notas y botones de la Create 3
```
- [ ] `--verificar` todo `[ok]`.
- [ ] `--medir`: anotar % de CPU: ______
- [ ] Luces, notas, botón 1 y START por UDP funcionan.

### 3. Recorridos cortos (cronometrar y grabar con `--video`)

- [ ] A→B→A en recto con 1 foto + QR: nombre correcto y vuelve a < 35 cm de A.
- [ ] Igual con 2 cajas en el camino: sin choques.
- [ ] Igual con 1–2 señales de giro: las sigue.
- [ ] Primero desde la PC (`python task-a/carrera-1/carrera.py --undock --buscar "Nombre" --video`, tecla `g`),
      luego en la Raspberry (`desplegar_carrera.sh --correr "Nombre"` + `python robot/control_carrera.py`).

### 4. Ensayo de la carrera completa

| Intento | Tiempo | Choques | Nombre correcto | Volvió a A | Notas |
|---|---|---|---|---|---|
| 1 | | | | | |
| 2 | | | | | |
| 3 | | | | | |

- [ ] 3–6 fotos + QR + señales, como en la competencia.
- [ ] `desplegar_carrera.sh --servicio`: arranca al encender y sale con el botón 1, sin SSH.
- [ ] Batería: cuántas corridas aguanta; volver al dock entre intentos.
- [ ] `desplegar_carrera.sh --video`: traer video y registro (para el informe).

### 5. Tarea B (gestos, Sofía)

- [ ] Reconocer + QR + anunciar integrado sobre `master` y probado en vivo.

## Mínimo para llegar bien al simulacro

Bloques 1 y 3 desde la PC. Si la Raspberry (bloque 2) no queda a tiempo, el simulacro se puede
correr con `carrera.py` desde la PC.
