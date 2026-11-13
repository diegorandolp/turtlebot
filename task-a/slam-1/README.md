# Tarea A · slam-1 · SLAM 2D en la PC (BreezySLAM)

Objetivo: conducir el robot por el laberinto con el teclado y que el mapa se vea limpio.
Todo corre en la PC (CPU, sin ROS); el robot solo envía `/scan` y `/odom` y recibe `move(v, w)` por UDP
mediante el puente (`tb4/`). El freno de emergencia por LiDAR sigue activo en el robot.

| Archivo | Qué hace |
|---|---|
| `slam_core.py` | Motores de SLAM: `BreezyEngine` (BreezySLAM) y `OdomEngine` (odometría pura, para comparar o de respaldo). Sin dependencias del robot |
| `slam_run.py` | Programa principal: conecta, mapea, dibuja el mapa y deja conducir con WASD |
| `test_slam_core.py` | Pruebas sin robot |

## Instalar BreezySLAM (una vez, con internet)

BreezySLAM **no está en PyPI** (`pip install breezyslam` falla con "not found"): se instala desde su
repositorio de GitHub. Lleva una extensión en C, así que hace falta `gcc` y las cabeceras de Python
(`python3-dev`; en esta PC ya están). Con el entorno virtual del proyecto activado:

```bash
pip install "git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python"
# con uv (desde la carpeta proyecto/):
uv pip install --python .venv/bin/python "git+https://github.com/simondlevy/BreezySLAM.git#subdirectory=python"
python -c "import breezyslam; print('ok')"        # comprobar
```
Sin BreezySLAM el programa sigue funcionando con `--engine odom` (aviso en pantalla).

### Si falla en WSL: "Could not resolve host: github.com"

Pasó aquí (WSL 2 en modo `mirrored`, Wi-Fi del laboratorio sin internet + celular por USB). Síntomas:
`ping 8.8.8.8` y `ping 140.82.112.3` (una IP de GitHub) funcionan, pero `ping github.com` da
"Temporary failure in name resolution" y `getent hosts github.com` no devuelve nada. Es decir, hay
internet, pero el DNS de WSL (`10.255.255.254`, un proxy hacia el DNS de Windows) no resuelve nombres.
(`uv` llega a PyPI porque usa su propio resolvedor, pero descarga de GitHub con `git`, que usa el del sistema.)

`dnsTunneling=true` en `.wslconfig` no lo arregló. Lo que sí funcionó fue fijar el DNS en WSL:

```bash
sudo tee /etc/wsl.conf >/dev/null <<'EOF'
[boot]
systemd=true
[network]
generateResolvConf=false
EOF
sudo rm -f /etc/resolv.conf
echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf
getent hosts github.com        # debe devolver una IP; no hace falta reiniciar WSL
```
Solo resuelve mientras haya internet (p. ej. el celular por USB). No afecta al robot: la comunicación con él
va por IP (`192.168.0.104`).

## Uso (desde la raíz del repositorio)

```bash
python3 -m unittest discover -s task-a/slam-1 -p "test_*.py"   # pruebas sin robot
python3 task-a/slam-1/slam_run.py --drive --undock             # conducir y mapear
python3 task-a/slam-1/slam_run.py --engine odom --drive        # misma ruta con odometría pura, para comparar
```

Teclas: `w/s` adelante/atrás (mantener), `a/d` girar, espacio parar, `+/-` velocidad, `u` sacar del dock, `l` ver/ocultar el LiDAR en vivo (puntos amarillos sobre el mapa),
`m` guardar el mapa, `q`/Esc salir (guarda al salir). Los mapas van a `task-a/slam-1/mapas/<fecha>/`
(`mapa.png`, `mapa.npy`, `mapa.json` con la escala).

Conduce despacio (0.12 m/s por defecto) y gira en el sitio: BreezySLAM corrige mejor con movimientos suaves.

## Recursos

Con solo CPU alcanza de sobra:
- Mapa 500×500 px (20 m, 4 cm/px) = 250 KB. BreezySLAM (núcleo en C, 1 hilo) con 360 rayos por scan.
- `OdomEngine` mide ~8–15 ms por scan en esta PC; el proceso entero (con OpenCV) ronda los cientos de MB como máximo.
- El cuello de botella es la RAM de WSL (~0.5 GB libres cuando se probó), no la CPU. Si hace falta, sube `memory=` en `C:\Users\<tú>\.wslconfig`.
- Si el laberinto es pequeño, baja `--map-m 10` para más resolución con el mismo coste.

## Límites conocidos

- Sin cierre de bucle: en recorridos largos con vueltas el mapa puede desviarse algo. Para un laberinto pequeño debería bastar.
- El LiDAR del TurtleBot 4 Lite es de baja resolución (720 puntos por vuelta, uno cada 0.5°, hasta 12 m); en pasillos largos y lisos la odometría domina.
- Los valores de `OdomEngine` (peso ocupado/libre) están calibrados con escaneos simulados, no con el robot.
