"""
Anunciar el nombre en voz alta desde la PC (el TurtleBot 4 Lite no tiene parlante para voz).

Prueba, en orden: espeak-ng / espeak (Linux), spd-say (Linux), PowerShell + System.Speech (Windows o
WSL), say (macOS). Si no hay ninguno, solo imprime. Nunca bloquea ni lanza excepciones.

  python -m reconocimiento.voz "La persona es Ana Perez"      # probar
Instalar una voz en Linux: sudo apt install espeak-ng   (Arch: sudo pacman -S espeak-ng)
"""
import shutil
import subprocess
import sys


def _comandos(texto):
    if shutil.which("espeak-ng"):
        yield ["espeak-ng", "-v", "es-419", "-s", "150", texto]
    if shutil.which("espeak"):
        yield ["espeak", "-v", "es", "-s", "150", texto]
    if shutil.which("spd-say"):
        yield ["spd-say", "-l", "es", texto]
    ps = shutil.which("powershell.exe") or shutil.which("powershell")
    if ps:
        t = texto.replace("'", "''")
        yield [ps, "-NoProfile", "-Command",
               f"Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{t}')"]
    if shutil.which("say"):
        yield ["say", texto]


def anunciar(texto, imprimir=True):
    """Dice `texto` en voz alta (en segundo plano). Devuelve el programa usado o None."""
    if imprimir:
        barra = "=" * (len(texto) + 8)
        print(f"\n{barra}\n    {texto}\n{barra}\n", flush=True)
    for cmd in _comandos(texto):
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return cmd[0]
        except OSError:
            continue
    return None


if __name__ == "__main__":
    usado = anunciar(" ".join(sys.argv[1:]) or "Prueba de voz")
    print("voz:", usado or "ninguna (instala espeak-ng)")
