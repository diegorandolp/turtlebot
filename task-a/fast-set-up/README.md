# Tarea A · arranque rápido en el laboratorio

Todo desde `repo/turtlebot`, con tu entorno de Python activado.

## 0. Preparar (una vez, CON internet, antes de ir al laboratorio)

1. Abre <https://drive.google.com/drive/folders/1iCaf1s0bpf0A4yShd8UdUV0XDVCas-oc>, descarga la carpeta `rostros`
   (Drive la baja como `rostros-....zip`) y copia ese zip en `raw-data/`.
2. Entorno de Python (3.10+). Elige **uno** de los dos:

```bash
sudo apt install sshpass build-essential python3-dev python3-venv   # sshpass: no escribir la contraseña del robot 100 veces
BREEZY="git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python"

# con pip
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt pillow pillow-heif
pip install "$BREEZY"                                    # opcional: SLAM, solo para --engine breezy

# con uv
uv venv .venv && source .venv/bin/activate
uv pip install -r requirements.txt pillow pillow-heif
uv pip install "$BREEZY"                                 # opcional: SLAM, solo para --engine breezy
```

`requirements.txt` trae `numpy` y `opencv-python`; `pillow` y `pillow-heif` leen las fotos `.HEIC`. Si BreezySLAM
falla al instalar, no pasa nada: los comandos usan `--engine odom`. En cada terminal nueva: `source .venv/bin/activate`.

3. Fotos y modelos:

```bash
raw-data/descomprimir.sh                                 # crea data/rostros/<Nombre>/

# modelos: detector (YuNet: ¿hay cara?) y reconocedor (SFace: ¿de quién?). Solo descarga los que falten.
Z=https://github.com/opencv/opencv_zoo/raw/main/models
mkdir -p task-a/auto-1/modelos task-a/reconocimiento/modelos
[ -f task-a/auto-1/modelos/face_detection_yunet_2023mar.onnx ] || curl -fL -o task-a/auto-1/modelos/face_detection_yunet_2023mar.onnx $Z/face_detection_yunet/face_detection_yunet_2023mar.onnx
[ -f task-a/reconocimiento/modelos/face_recognition_sface_2021dec.onnx ] || curl -fL -o task-a/reconocimiento/modelos/face_recognition_sface_2021dec.onnx $Z/face_recognition_sface/face_recognition_sface_2021dec.onnx
ls -l task-a/auto-1/modelos task-a/reconocimiento/modelos  # deben verse los 2 .onnx (227 KB y 37 MB)
```

## 1. Conexión y puente

Wi-Fi `Lab_Computech_3_5G` (clave `Computech2025!`). En cada terminal:

```bash
IP=192.168.0.104
export SSHPASS=turtlebot4
ping -c 3 $IP                                            # ¿responde el robot?
```

Si **no responde**, la IP pudo cambiar. Búscalo por su MAC:

```bash
for i in $(seq 1 254); do ping -c1 -W1 192.168.0.$i >/dev/null & done; wait
IP=$(ip neigh | grep -i d8:3a:dd:7e:5e:48 | awk '{print $1}' | head -1)
echo "robot en: $IP"                                     # vacío = no lo encontró (¿está encendido?)
ping -c 3 $IP
```

Si la IP cambió, usa la nueva en **todos** los comandos de abajo (también en los de la sección 3).

Tras **encender** el robot (solo una persona, tarda ~5 min):

```bash
TB4_IP=$IP robot/tb4_inicio.sh
python -m tb4.client $IP                                 # debe mostrar batería, dock y pose
```

Si sale `accion /undock no disponible`: `TB4_IP=$IP robot/deploy_bridge.sh --bg` (reinicia solo el puente).

## 2. Cámara

La cámara normal falla al azar (`X_LINK_ERROR`). Usa siempre la de la Tarea A (solo color, USB 2), **tras cada reinicio** del robot:

```bash
TB4_IP=$IP task-a/camara/camara_rgb.sh                   # debe acabar con "average rate: ~30"
python task-a/auto-1/probar_camara.py $IP                # ver la cámara en vivo (robot FUERA del dock)
```

Si sigue sin imagen: reconecta el cable USB de la cámara y repite `camara_rgb.sh`.

## 3. Experimentos

Hay que decir a quién buscar con `--buscar`: el nombre **exacto** de su carpeta en `data/rostros`
(míralos con `ls data/rostros`; si tiene espacio, entre comillas: `--buscar "Brigitte Rojas"`).
La primera vez tarda ~1 min en procesar la galería; después es instantáneo.

Ejemplo completo: robot en `192.168.0.104`, buscando a `Melisa_Rivera`, saliendo del dock.

**1. Más robusta** (encontró la cara en 15/15 mapas simulados):

```bash
python task-a/auto-1/explorar.py 192.168.0.104 --buscar Melisa_Rivera --frontera rama --engine odom --cam-fov 42 --score 0.7 --min-ancho 12 --undock
```

**2. Segunda** (más rápida en laberintos perfectos, falló 1/15):

```bash
python task-a/auto-1/explorar.py 192.168.0.104 --buscar Melisa_Rivera --frontera cercana --engine odom --cam-fov 42 --score 0.7 --min-ancho 12 --undock
```

**3. Simple** (seguir la pared derecha):

```bash
python task-a/auto-1/explorar.py 192.168.0.104 --buscar Melisa_Rivera --estrategia derecha --engine odom --cam-fov 42 --score 0.7 --min-ancho 12 --undock
```

Solo cambian la IP (si cambió) y el nombre. El resto se deja igual.

⚠️ La versión 3 es experimental: en simulación abandona pronto la pared y sigue como la versión 2.

Quita `--undock` si el robot ya está fuera del dock (al salir retrocede solo: deja espacio detrás).

**Teclas** (clic en la ventana): `g` empezar · **espacio = parar** · `q` salir.

Al terminar, la terminal muestra `PERSONA RECONOCIDA: <nombre> (de vuelta en A)`.
Anota para cada intento: versión, si encontró a la persona, si volvió a A y el tiempo.

## 4. Posibles fallas

| Si pasa esto | Haz esto |
|---|---|
| Va hacia una cara pero **llega desviado** (no queda de frente a la foto) o dice `desde el punto de vista no veo ninguna cara` | El `--cam-fov` (42) está mal. Medirlo (ver abajo) y usar el valor medido |
| No detecta caras salvo muy de cerca | Bajar umbrales: `--score 0.6 --min-ancho 8` |
| Se para ante cosas que no son caras | Subir umbrales: `--score 0.8 --min-ancho 20` |
| Encuentra la foto correcta pero dice `no es la persona buscada` / `(similitud baja)` | `--umbral-reco 0.40` |
| Acepta a una persona equivocada | `--umbral-reco 0.55` |
| No entra en pasillos estrechos (se queda girando en la entrada) | `--radio 0.18` |
| El mapa sale torcido o con paredes dobles | Probar `--engine breezy` en vez de `--engine odom` |

**Medir el campo de visión (`--cam-fov`)**, 2 min, con `python task-a/auto-1/probar_camara.py $IP --score 0.5 --min-ancho 5`:
pon una foto de cara a **1.0 m** delante de la cámara y deslízala hacia un lado hasta que la caja verde **toque el borde**
de la imagen. Mide cuánto la desplazaste (`X`, en m) desde el centro. Entonces `--cam-fov` = 2·atan(X / 1.0) en grados
(`python -c "import math; print(2*math.degrees(math.atan(X)))"`). Ejemplo: X = 0.38 m → 42.
