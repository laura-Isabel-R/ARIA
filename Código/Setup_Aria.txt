#!/usr/bin/env bash
set -euo pipefail

echo "[1/7] comprobando paquetes del sistema (no toca python del sistema)…"
sudo apt-get update -y
# utilidades seguras (no reemplazan opencv/python del sistema)
sudo apt-get install -y python3-venv python3-pip git alsa-utils v4l-utils opencv-data

echo "[2/7] creando/activando entorno virtual aislado con acceso de solo lectura a paquetes del sistema…"
VENV="$HOME/aria_env/.venv"
mkdir -p "$HOME/aria_env"
python3 -m venv "$VENV" --system-site-packages
# shellcheck disable=SC1090
source "$VENV/bin/activate"
python -V
pip install --upgrade pip

echo "[3/7] instalando pytorch (cpu) y torchvision en el venv…"
pip install --extra-index-url https://download.pytorch.org/whl/cpu torch
pip install --extra-index-url https://download.pytorch.org/whl/cpu torchvision

echo "[4/7] instalando dependencias de python del proyecto en el venv…"
pip install numpy "qrcode[pil]" pyserial pandas requests matplotlib

echo "[5/7] asegurando cascade haar de opencv para detección de rostro…"
mkdir -p "$HOME/cascades"
if [ -f /usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml ]; then
  cp /usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml "$HOME/cascades/"
elif [ -f /usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml ]; then
  cp /usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml "$HOME/cascades/"
else
  echo "[warn] no encontré el cascade en /usr/share; opencv-data debería haberlo provisto. puedes actualizar con: sudo apt-get install -y opencv-data"
fi

echo "[6/7] pruebas rápidas de import (no abre cámara)…"
python - <<'PY'
import sys, torch, torchvision, numpy as np, cv2, os
print("python:", sys.executable)
print("torch:", torch.__version__, "torchvision:", torchvision.__version__)
print("numpy:", np.__version__, "opencv:", cv2.__version__)
p = getattr(cv2.data,"haarcascades","") + "haarcascade_frontalface_default.xml"
print("opencv haarcandidate:", p, "exists?", os.path.exists(p))
print("home cascade:", os.path.expanduser("~/cascades/haarcascade_frontalface_default.xml"),
      "exists?", os.path.exists(os.path.expanduser("~/cascades/haarcascade_frontalface_default.xml")))
PY

echo "[7/7] listo."
echo
echo "para ejecutar tu script sin depender de configuraciones de vscode, usa SIEMPRE:"
echo "$VENV/bin/python /home/modulomk/ARIA/Prueba.py"
echo
echo "nota: si tu código aún usa CASCADE_PATH fijo, puedes apuntarlo a:"
echo "CASCADE_PATH = '/home/modulomk/cascades/haarcascade_frontalface_default.xml'"
