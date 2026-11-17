"""
La Raspberry del TurtleBot 4 Lite no tiene parlante. Cuando la carrera corre EN el robot, la PC sigue su log
por SSH y este programa dice en voz alta el anuncio ("==== La palabra es X ====") y muestra el estado.

    ssh ubuntu@ROBOT "tail -F ~/turtlebot/carrera.log" | python3 robot/escuchar_anuncio.py
    (eso hace robot/desplegar_carrera.sh --escuchar)

Tambien sirve con un archivo:  python3 robot/escuchar_anuncio.py < carrera.log
"""
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reconocimiento.voz import anunciar  # noqa: E402

PATRON = re.compile(r"^==== (.+) ====$")
dicho = False
for linea in sys.stdin:
    linea = linea.rstrip("\n")
    m = PATRON.match(linea.strip())
    if m and not dicho:
        dicho = True
        texto = m.group(1).replace("_", " ").replace("-", " ")
        print(f"\n>>> ANUNCIO DEL ROBOT: {texto}\n", flush=True)
        anunciar(texto, imprimir=False)
        continue
    if linea.startswith(("[", "  [")) or "[fin]" in linea:
        print(linea, flush=True)
