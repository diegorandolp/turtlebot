# Tarea A · reconocimiento · ¿De quién es la cara?

YuNet ([../auto-1/face.py](../auto-1/face.py)) encuentra la cara; **SFace** (`cv2.FaceRecognizerSF`, opencv_zoo,
`modelos/face_recognition_sface_2021dec.onnx`, 37 MB) la convierte en un vector de 128 números y se compara con la
galería `data/rostros/<Nombre>/` por similitud coseno. Umbral 0.363 (el de OpenCV): por debajo, "desconocido".

```bash
raw-data/descomprimir.sh                          # una vez: fotos en data/rostros
pip install pillow pillow-heif                    # una vez: leer las fotos .HEIC del iPhone
python task-a/reconocimiento/evaluar.py           # 10 fotos al azar contra el resto de la galería
python task-a/reconocimiento/evaluar.py --n 20 --semilla 3
```

Resultado (2026-11-13, Ryzen 5 3500U, solo CPU): **10/10 aciertos**, ~63 ms por foto (detección + embedding),
galería de 94 fotos / 17 personas cargada en ~47 s.
