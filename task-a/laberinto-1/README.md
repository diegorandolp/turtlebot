# Tarea A · laberinto-1 · Ida con señales → reconocer cara → leer QR → volver → anunciar

Construido sobre [auto-1](../auto-1) (SLAM, fronteras, cobertura de cámara, YuNet) y
[reconocimiento](../reconocimiento) (SFace). **No modifica auto-1.**

## Flujo

1. **IDA**: explora solo. La cámara marca en el mapa las señales **por color**: **azul = ida**, **rojo = vuelta**
   (se anotan; la ruta de vuelta no las necesita). Una señal azul se confirma a la 2.ª detección (robustez ante ruido)
   y desde entonces el robot prefiere las fronteras cercanas a la última azul (`--peso-ida`, defecto 2).
2. **Callejón sin salida**: no hay caso especial. Sin fronteras cerca de la última azul, el robot va a la mejor frontera
   libre que ya detectó (retrocede) y sigue; las metas atascadas se descartan y se reintentan.
3. **CARA**: cada foto vista se apunta en el mapa; el robot va de frente a cada una y SFace decide si es `--buscar`.
4. **QR**: ante la persona correcta lee el QR de debajo (`qr.py`: quieto → gira ±9° → se acerca hasta 0.42 m) y guarda el texto.
5. **VUELTA**: ruta más corta por el mapa conocido hasta A (ya sin callejones).
6. **ANUNCIO** en A: banner en pantalla, voz si la PC tiene `spd-say`/`espeak-ng`/`espeak`/`say`, y `corridas/<fecha>/mensaje.txt`
   + `resultado.json` + `mapa.png` (con las señales).

## Uso (desde la raíz, venv activado)

```bash
python -m unittest discover -s task-a/laberinto-1 -p "test_*.py"
python task-a/laberinto-1/laberinto.py --sim --no-gui --go --mundo lab_desvios --buscar F3 --hd   # simulador
python task-a/laberinto-1/laberinto.py --undock --buscar "Nombre Apellido" --hd                  # ROBOT REAL (g = empezar)
```
Mundos del simulador: `lab_desvios`, `lab_camino`, `lab_serpiente`, `lab_laberinto` (señales azules/rojas y un QR real bajo
cada foto). Las demás opciones son las de `auto-1/explorar.py`. **Usa `--hd`**: el QR a 0.55 m solo se lee en 1280×720
(necesita el puente nuevo: `robot/deploy_bridge.sh`, y `camara_rgb.sh --hd`).

## Resultado en el simulador

`lab_desvios`, `--buscar F3`: encontró F3, leyó el QR, volvió a A y anunció el mensaje; 0 colisiones, 5 señales
confirmadas (3 azules, 2 rojas).

## Pendiente / sin validar con el robot real

- Umbrales HSV de `senales.py` calibrados solo con colores puros del simulador: **pruébalos con tus señales impresas
  y la luz del salón** (`detectar_senales(foto)`), sobre todo el rojo y el azul oscuro.
- El QR de la foto real: ajusta `FaseQR` (distancias, giro) según tamaño y altura; la cámara real puede no ver el QR a 0.55 m.
- La señal azul solo orienta (no tiene dirección): en un cruce con dos ramas libres cerca de la azul puede probar ambas.
- Sin BreezySLAM instalado el simulador usó odometría pura; con el robot real instálalo (ver slam-1).
- La voz depende de que la PC tenga sintetizador; si no, solo imprime el mensaje.
