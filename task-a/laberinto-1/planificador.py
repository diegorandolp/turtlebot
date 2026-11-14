"""Planificador del laberinto: el de auto-1 + preferencia por la zona de la ultima senal AZUL (ida)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "auto-1"))
from planner import Planner  # noqa: E402


class PlannerLab(Planner):
    """Elige la frontera que minimiza (camino + peso * distancia a la ultima senal azul). Sin senal azul se
    comporta como la estrategia "rama" de auto-1 (terminar la rama actual antes de cruzar a otra). Si la rama
    de la senal se acaba (callejon sin salida), el termino es solo una preferencia: el robot retrocede solo
    a la frontera libre mas conveniente de las que ya detecto."""

    def __init__(self, *a, peso_ida=2.0, **kw):
        kw.setdefault("frontera", "rama")
        super().__init__(*a, rama_peso=peso_ida, **kw)
        self.ida_anchor = None

    def plan_explore(self, img, start_xy, prev_goal=None, bad_goals=(), anchor=None):
        return super().plan_explore(img, start_xy, prev_goal, bad_goals, anchor=self.ida_anchor or anchor)
