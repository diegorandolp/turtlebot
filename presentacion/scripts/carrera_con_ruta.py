"""Corre una carrera del simulador (sin ventana, sin voz, sin guardar en corridas/) y guarda, ademas de la traza del
navegador, la RUTA VERDADERA del robot simulado (la traza es la odometria, con ruido: dibujada sobre el plano del
mundo se sale de las paredes). Para la diapositiva del circuito:

  distrobox enter ros-jazzy -- bash -lc 'source ~/.venvs/tb4/bin/activate; \
      python presentacion/scripts/carrera_con_ruta.py --mundo tubo:3 --turbo --video RUTA.mp4'

-> presentacion/figuras/datos/tubo_traza.json (las mismas opciones que la carrera del video).
"""
import json
import sys
from pathlib import Path

AQUI = Path(__file__).resolve().parent
REPO = AQUI.parent.parent
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "task-a" / "carrera-1"))

import carrera  # noqa: E402


def main():
    args = carrera.parse(["--sim", "--go", "--no-gui", "--mudo", "--no-guardar", "--traza"] + sys.argv[1:])
    mundo, robot = carrera.crear_sim(args)
    ruta = []
    avanzar = robot.advance

    def advance(dt):                         # cada paso del simulador: la pose verdadera, con su tiempo
        avanzar(dt)
        ruta.append((round(robot.time, 3), round(robot.wx, 4), round(robot.wy, 4), round(robot.wth, 4)))

    robot.advance = advance
    res = carrera.correr(robot, args, sim=True, gui=False, mundo=mundo, max_t=args.max_t)
    res["ruta"] = ruta[::2]
    res["opciones"] = sys.argv[1:]
    out = AQUI.parent / "figuras" / "datos" / "tubo_traza.json"
    out.write_text(json.dumps(res, ensure_ascii=False))
    print("[ok]", out, {k: res.get(k) for k in ("estado", "tiempo", "tiempo_b", "choques", "exito", "palabra")})


if __name__ == "__main__":
    main()
