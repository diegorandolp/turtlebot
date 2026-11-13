"""Reconocimiento facial comun a las Tareas A y B (ver rostros.py y README.md de esta carpeta)."""
from .rostros import (ROSTROS, Cara, Detector, Embebedor, Galeria, Reconocedor, Resultado, Votacion,
                      alinear, galeria_path, modelo_auto, nombre_bonito, peso_por_tamano)

__all__ = ["ROSTROS", "Cara", "Detector", "Embebedor", "Galeria", "Reconocedor", "Resultado", "Votacion",
           "alinear", "galeria_path", "modelo_auto", "nombre_bonito", "peso_por_tamano"]
