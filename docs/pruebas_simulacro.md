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
