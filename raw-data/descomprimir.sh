#!/usr/bin/env bash
# Descomprime los zips de raw-data/ en data/ (p. ej. data/rostros/<Nombre>/...).
#   raw-data/descomprimir.sh            # todos los .zip de raw-data/
#   raw-data/descomprimir.sh archivo.zip
# No sobrescribe archivos que ya existan en data/ (usa -o en unzip si quieres reemplazarlos).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$(dirname "$HERE")/data"
mkdir -p "$DEST"
if [ $# -gt 0 ]; then zips=("$@"); else zips=("$HERE"/*.zip); fi
for z in "${zips[@]}"; do
  [ -f "$z" ] || { echo "[FALLA] no existe: $z"; exit 1; }
  echo "[..]   $z -> $DEST"
  unzip -q -n "$z" -d "$DEST"
done
echo "[ok]   listo:"; find "$DEST" -mindepth 1 -maxdepth 2 -type d | sort
