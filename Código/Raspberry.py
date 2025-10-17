#!/usr/bin/env python3
import time
import threading
import subprocess
import shutil
from pathlib import Path
import os

import torch
import cv2
from picamera2 import Picamera2
import numpy as np

# --- Serial y QR ---
import serial
import qrcode

# --- GPIO (LEDs + Botón) ---
import RPi.GPIO as GPIO

# ========= Configuración =========
AUDIO_MAP = {
    1: "/home/modulomk/ARIA/Audios/Audios Inicio/Faltan 3 personas.wav",
    2: "/home/modulomk/ARIA/Audios/Audios Inicio/Faltan 2 personas.wav",
    3: "/home/modulomk/ARIA/Audios/Audios Inicio/Falta 1 persona.wav",
    4: "/home/modulomk/ARIA/Audios/Audios Inicio/Listos.wav",  # 4 o más
    5: "/home/modulomk/ARIA/Audios/Audios Inicio/CodigoQR.wav",  # Audio del QR
}
ALSA_DEVICE = "plughw:3,0"

PROCESS_EVERY = 4
YOLO_SIZE = 320
WINDOW_NAME = "Detección YOLOv5"
MIN_INTERVAL_BETWEEN_TRIGGERS = 2.0

# Face-check (rostro frontal) — UMBRALES MÁS ESTRICTOS
CASCADE_PATH = "/home/modulomk/cascades/haarcascade_frontalface_default.xml"
FACE_MIN_SIZE = (60, 60)
FACE_NEIGHBORS = 6
MAX_PERSON_BBOXES = 6

DRAW_LAST_BOXES = True
ASSUME_RAW_IS_RGB = True

# Serial hacia ESP32
SERIAL_PORT = "/dev/serial0"   # Pi GPIO14/15 habilitado
SERIAL_BAUD = 115200

# QR del hotspot ESP32
ESP32_URL = "http://192.168.4.1/"
QR_CACHE = Path("/home/modulomk/ARIA/qr_esp32.png")

# ========= LEDs =========
LED_PINS = [18, 23, 24, 25]  # cambia si usas otros pines
GPIO.setmode(GPIO.BCM)
for p in LED_PINS:
    GPIO.setup(p, GPIO.OUT)
    GPIO.output(p, GPIO.HIGH)

def set_led_count(n: int):
    """Enciende n LEDs (0..4) de izquierda a derecha, considerando que los relés son NO."""
    n = max(0, min(4, n))
    for i, pin in enumerate(LED_PINS):
        GPIO.output(pin, GPIO.LOW if i < n else GPIO.HIGH)

# ========= Botones (4 botones físicos) =========
# Rojo=13, Verde=5, Azul=19, Amarillo=6 (pull-up interno)
BTN_PINS = {"rojo": 5, "verde": 13, "azul": 19, "amarillo": 6}
for p in BTN_PINS.values():
    GPIO.setup(p, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ========= Videos =========
VIDEO_SOURCE_ROJO = "/home/modulomk/ARIA/Videos/Ingeniería es en la UAO.mp4"
VIDEO_SOURCE_VERDE = "/home/modulomk/ARIA/Videos/Video 2 -ARIA.mp4"  # <-- CAMBIA esta ruta

PLAYER_CMD_ROJO  = ["vlc", "--fullscreen", "--play-and-exit", "--quiet","--no-video-title-show", VIDEO_SOURCE_ROJO]
PLAYER_CMD_VERDE = ["vlc", "--fullscreen", "--play-and-exit", "--quiet","--no-video-title-show", VIDEO_SOURCE_VERDE]

# ========= Imágenes estáticas (para botones Azul/Amarillo) =========
IMG_EXPER_PATH    = "/home/modulomk/ARIA/Codigo_qr/Experiencia.jpg"
IMG_CONTACTO_PATH = "/home/modulomk/ARIA/Codigo_qr/Contacto.jpg"
SPECIAL_IMG_SECS  = 15  # duración en pantalla

img_exper = cv2.imread(IMG_EXPER_PATH, cv2.IMREAD_COLOR)
if img_exper is None:
    print(f"[IMG][WARN] No se pudo cargar {IMG_EXPER_PATH}")

img_contacto = cv2.imread(IMG_CONTACTO_PATH, cv2.IMREAD_COLOR)
if img_contacto is None:
    print(f"[IMG][WARN] No se pudo cargar {IMG_CONTACTO_PATH}")

# Control de imagen especial mostrada en ventana
special_img = None
special_img_until = 0.0

def show_static_image(img):
    """Muestra 'img' a pantalla completa en la ventana por SPECIAL_IMG_SECS."""
    global special_img, special_img_until
    if img is None:
        print("[IMG][WARN] Imagen no disponible.")
        return
    special_img = img.copy()
    special_img_until = time.time() + SPECIAL_IMG_SECS
    print("[IMG] Mostrando imagen en pantalla…")

# Flags y locks
_button_enabled = False
_event_registered = False
_video_lock = threading.Lock()
_video_playing = False

def _play_video_worker(cmd):
    global _video_playing
    try:
        subprocess.run(cmd)
    finally:
        with _video_lock:
            _video_playing = False

def play_video_cmd(cmd):
    global _video_playing
    with _video_lock:
        if _video_playing:
            return
        _video_playing = True
    threading.Thread(target=_play_video_worker, args=(cmd,), daemon=True).start()

# ========== Callbacks de botones ==========
def _btn_guard():
    time.sleep(0.03)  # debounce suave

def on_button_rojo(channel):
    _btn_guard()
    if GPIO.input(channel) != GPIO.HIGH: return
    if not _button_enabled: return
    print("[BTN ROJO] → reproducir VIDEO 1")
    play_video_cmd(PLAYER_CMD_ROJO)

def on_button_verde(channel):
    _btn_guard()
    if GPIO.input(channel) != GPIO.HIGH: return
    if not _button_enabled: return
    print("[BTN VERDE] → reproducir VIDEO 2")
    play_video_cmd(PLAYER_CMD_VERDE)

def on_button_azul(channel):
    _btn_guard()
    if GPIO.input(channel) != GPIO.HIGH: return
    if not _button_enabled: return
    print("[BTN AZUL] → mostrar imagen de EXPERIENCIA")
    show_static_image(img_exper)

def on_button_amarillo(channel):
    _btn_guard()
    if GPIO.input(channel) != GPIO.HIGH: return
    if not _button_enabled: return
    print("[BTN AMARILLO] → mostrar imagen de CONTACTO")
    show_static_image(img_contacto)

def ensure_buttons_event_registered():
    global _event_registered
    if _event_registered: return
    GPIO.add_event_detect(BTN_PINS["rojo"], GPIO.RISING, callback=on_button_rojo, bouncetime=200)
    GPIO.add_event_detect(BTN_PINS["verde"], GPIO.RISING, callback=on_button_verde, bouncetime=200)
    GPIO.add_event_detect(BTN_PINS["azul"], GPIO.RISING, callback=on_button_azul, bouncetime=200)
    GPIO.add_event_detect(BTN_PINS["amarillo"], GPIO.RISING, callback=on_button_amarillo, bouncetime=200)
    _event_registered = True

def disable_button():
    global _button_enabled
    _button_enabled = False

def enable_button():
    global _button_enabled
    _button_enabled = True
    ensure_buttons_event_registered()

# ========= Carga cascade =========
face_cascade = cv2.CascadeClassifier(CASCADE_PATH)
if face_cascade.empty():
    raise RuntimeError(f"No se pudo cargar el cascade: {CASCADE_PATH}")

# ========= Audio helpers =========
_play_lock = threading.Lock()
_is_playing = False
_last_trigger_time = 0.0
_last_announced_bucket = None

def _run(cmd):
    p = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if p.returncode != 0:
        print(f"[AUDIO][ERROR] rc={p.returncode}")
        if p.stdout: print(f"[AUDIO][STDOUT]\n{p.stdout}")
        if p.stderr: print(f"[AUDIO][STDERR]\n{p.stderr}")
    return p.returncode

def _play_once(path: str, device: str | None):
    src = Path(path)
    if not src.exists():
        print(f"[AUDIO][ERR] No existe: {path}")
        return 1
    aplay = shutil.which("aplay")
    if not aplay:
        print("[AUDIO][ERR] Falta 'aplay' (instala alsa-utils).")
        return 1
    cmd = [aplay]
    if device: cmd += ["-D", device]
    cmd += [str(src)]
    print(f"[AUDIO] Ejecutando: {' '.join(cmd)}")
    return _run(cmd)

def play_async(path: str, device: str | None):
    def worker():
        rc = _play_once(path, device)
        with _play_lock:
            global _is_playing
            _is_playing = False
        if rc == 0:
            print("[AUDIO] Reproducción OK")
    with _play_lock:
        global _is_playing, _last_trigger_time
        now = time.time()
        if _is_playing: return
        if now - _last_trigger_time < MIN_INTERVAL_BETWEEN_TRIGGERS: return
        _is_playing = True
        _last_trigger_time = now
    threading.Thread(target=worker, daemon=True).start()

def play_listos_force(path: str, device: str | None, wait_timeout: float = 5.0):
    def worker():
        t0 = time.time()
        while True:
            with _play_lock:
                busy = _is_playing
            if not busy or (time.time() - t0) > wait_timeout:
                break
            time.sleep(0.05)
        rc = _play_once(path, device)
        if rc == 0:
            print("[AUDIO] Reproducción OK (Listos)")
        time.sleep(0.5)
        qr_audio_path = AUDIO_MAP.get(5)
        if qr_audio_path:
            print(f"[AUDIO] Reproduciendo audio del QR: {qr_audio_path}")
            _play_once(qr_audio_path, device)
    threading.Thread(target=worker, daemon=True).start()

def bucket_for_count(n: int) -> int | None:
    if n <= 0: return None
    if n >= 4: return 4
    return n

def maybe_announce(curr_count: int):
    global _last_announced_bucket
    b = bucket_for_count(curr_count)
    if b is None or b >= 4:
        return
    if b != _last_announced_bucket:
        audio_path = AUDIO_MAP.get(b)
        if audio_path:
            print(f"[AUDIO] Conteo (frontal)={curr_person_count} -> bucket={b}. Reproduciendo: {audio_path}")
            play_async(audio_path, ALSA_DEVICE)
        _last_announced_bucket = b

# ========= YOLO =========
print("Cargando modelo YOLOv5n…")
model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True, trust_repo=True)
try:
    model.classes = [0]  # solo personas
except Exception:
    pass
model.conf = 0.40          # más estricto
model.cpu().eval()

# ========= Cámara =========
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "BGR888"}
)
picam2.configure(camera_config)
picam2.start()

# ========= Utilidades de imagen/rostro =========
def to_bgr(frame):
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) if ASSUME_RAW_IS_RGB else frame

def clip_bbox(x1,y1,x2,y2,w,h):
    x1 = max(0, min(int(x1), w-1)); y1 = max(0, min(int(y1), h-1))
    x2 = max(0, min(int(x2), w-1)); y2 = max(0, min(int(y2), h-1))
    if x2 <= x1: x2 = min(x1+1, w-1)
    if y2 <= y1: y2 = min(y1+1, h-1)
    return x1,y1,x2,y2

def face_frontal_in_roi(bgr_roi):
    if bgr_roi.size == 0: return False
    gray = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2GRAY)
    cv2.equalizeHist(gray, gray)
    faces = face_cascade.detectMultiScale(
        gray,
        scaleFactor=1.15,
        minNeighbors=FACE_NEIGHBORS,
        minSize=FACE_MIN_SIZE,
        flags=cv2.CASCADE_SCALE_IMAGE
    )
    return len(faces) > 0

# ========= Serial + QR helpers =========
ser = None
def serial_open():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"[SERIAL] Conectado a {SERIAL_PORT} @ {SERIAL_BAUD}")
    except Exception as e:
        print("[SERIAL][WARN] No se pudo abrir el serial:", e)

def _fallback_send_with_tee():
    try:
        cmd = 'echo -ne "SALUDO\n" | sudo tee /dev/serial0 > /dev/null'
        rc = subprocess.call(cmd, shell=True)
        if rc == 0:
            print("[SERIAL][FALLBACK] Enviado SALUDO con sudo tee")
        else:
            print(f"[SERIAL][FALLBACK][ERR] rc={rc}")
    except Exception as e:
        print("[SERIAL][FALLBACK][ERR]", e)

def notify_esp32_unlock():
    """Envía 'SALUDO' a la ESP32 por UART."""
    global ser
    try:
        if ser is None or not ser.writable():
            print("[SERIAL] Reabriendo /dev/serial0 …")
            serial_open()
        if ser and ser.writable():
            ser.reset_output_buffer()
            ser.write(b"SALUDO\n")
            ser.flush()
            print("[SERIAL] Enviado SALUDO (pyserial)")
            try:
                resp = ser.readline().decode(errors="ignore").strip()
                if resp:
                    print("[SERIAL] ACK:", resp)
            except Exception:
                pass
            return
    except Exception as e:
        print("[SERIAL][WARN] pyserial falló al enviar SALUDO:", e)
    _fallback_send_with_tee()

def get_qr_bgr(url=ESP32_URL, size_px=300):
    if QR_CACHE.exists():
        qr = cv2.imread(str(QR_CACHE), cv2.IMREAD_COLOR)
        if qr is not None:
            return cv2.resize(qr, (size_px, size_px), interpolation=cv2.INTER_AREA)
    pil_img = qrcode.make(url).convert('L')
    qr_gray = np.array(pil_img, dtype=np.uint8)
    qr_gray = cv2.resize(qr_gray, (size_px, size_px), interpolation=cv2.INTER_NEAREST)
    qr_bgr = cv2.cvtColor(qr_gray, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(QR_CACHE), qr_bgr)
    return qr_bgr

def overlay_qr(frame_bgr, qr_bgr, pos="br", margin=20):
    fh, fw = frame_bgr.shape[:2]
    qh, qw = qr_bgr.shape[:2]
    if pos == "br":
        y2 = fh - margin; y1 = y2 - qh
        x2 = fw - margin; x1 = x2 - qw
    elif pos == "bl":
        y2 = fh - margin; y1 = y2 - qh
        x1 = margin;     x2 = x1 + qw
    elif pos == "tr":
        y1 = margin;     y2 = y1 + qh
        x2 = fw - margin; x1 = x2 - qw
    else:  # 'tl'
        y1 = margin;     y2 = y1 + qh
        x1 = margin;     x2 = x1 + qw
    if x1 < 0 or y1 < 0 or x2 > fw or y2 > fh:
        return frame_bgr
    frame_bgr[y1:y2, x1:x2] = qr_bgr
    cv2.putText(frame_bgr, ESP32_URL, (x1-5, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
    return frame_bgr

# ========= Reset de ciclo al recibir DONE =========
def reset_detection_cycle():
    """Rearma el ciclo de detección luego de terminar la rutina en el brazo."""
    global detection_enabled, listos_already_played, freeze_enabled
    global freeze_frame, last_boxes, curr_person_count, _last_announced_bucket
    global consecutive_ge4, special_img, special_img_until
    detection_enabled = True
    listos_already_played = False
    freeze_enabled = False
    freeze_frame = None
    last_boxes = []
    curr_person_count = 0
    _last_announced_bucket = None
    consecutive_ge4 = 0
    special_img = None
    special_img_until = 0.0
    set_led_count(0)
    disable_button()
    print("[RESET] Detección reactivada. Botones deshabilitados.")

def handle_cmd(line: str):
    line = line.strip()
    print(f"[ACTION] Comando recibido: {line}")
    if line == "DONE":
        reset_detection_cycle()   # ← botón "Salida" en la ESP32 enviará DONE
    elif line == "1":
        print("[ACTION] Boton 1 => ejecutar accion 1")
    elif line == "2":
        print("[ACTION] Boton 2 => ejecutar accion 2")
    elif line == "3":
        print("[ACTION] Boton 3 => ejecutar accion 3")
    elif line == "4":
        print("[ACTION] Boton 4 => ejecutar accion 4")
    else:
        print("[ACTION] Comando no reconocido")

def serial_reader_loop():
    global ser
    if not ser:
        return
    print("[SERIAL] Reader started.")
    buf = b""
    while True:
        try:
            b = ser.read(1)
            if not b:
                continue
            if b in (b'\n', b'\r'):
                line = buf.decode(errors='ignore').strip()
                buf = b""
                if line:
                    print(f"[SERIAL] RX CMD: {line}")
                    handle_cmd(line)
            else:
                buf += b
        except Exception as e:
            print("[SERIAL][ERR]", e)
            time.sleep(0.2)
            
def put_text_with_bg(img, text, org,
                     font=cv2.FONT_HERSHEY_SIMPLEX,
                     scale=0.8,
                     text_color=(0,0,0),      # negro
                     bg_color=(255,255,255),  # blanco
                     thickness=2,
                     padding=6):
    # Tamaño del texto
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    x, y = org

    # Rectángulo de fondo (blanco)
    top_left  = (x - padding, y - th - baseline - padding)
    bot_right = (x + tw + padding, y + baseline + padding//2)
    cv2.rectangle(img, top_left, bot_right, bg_color, -1)

    # Texto encima (negro)
    cv2.putText(img, text, (x, y), font, scale, text_color, thickness, cv2.LINE_AA)

# Preparar serial y QR
serial_open()
threading.Thread(target=serial_reader_loop, daemon=True).start()
QR_ONCE = get_qr_bgr(ESP32_URL, 300)

# ========= Ventana =========
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_EXPANDED)
cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_AUTOSIZE, 0)
cv2.resizeWindow(WINDOW_NAME, 1280, 720)

# ========= Estado =========
frame_counter = 0
curr_person_count = 0
detection_enabled = True
listos_already_played = False
freeze_enabled = False
freeze_frame = None
last_boxes = []
freeze_start_ts = None

# Histeresis para el trigger: 4 frontales consecutivos
REQUIRE_CONSECUTIVE_4 = 1
consecutive_ge4 = 0

# ========= Bucle principal =========
try:
    while True:
        raw = picam2.capture_array()
        raw_bgr = to_bgr(raw)

        # Frame base (congelado o cámara)
        if freeze_enabled and freeze_frame is not None:
            frame_bgr = freeze_frame.copy()
        else:
            frame_bgr = raw_bgr.copy()

        H, W = frame_bgr.shape[:2]
        frame_counter += 1

        # ======= Detección solo si no hay imagen especial en pantalla =======
        special_active = (time.time() < special_img_until) and (special_img is not None)

        if (not special_active) and (not freeze_enabled) and detection_enabled and (frame_counter % PROCESS_EVERY == 0):
            frame_rgb_for_yolo = cv2.cvtColor(raw_bgr, cv2.COLOR_BGR2RGB)
            with torch.inference_mode():
                results = model(frame_rgb_for_yolo, size=YOLO_SIZE)

            det = results.xyxy[0]
            count_facing = 0
            new_boxes = []

            if det is not None and det.shape[0] > 0:
                rows = det.cpu().numpy()
                rows = rows[rows[:,4].argsort()[::-1]]
                rows = rows[:MAX_PERSON_BBOXES]

                for row in rows:
                    x1,y1,x2,y2,conf,cls = row
                    if int(cls) != 0:
                        continue
                    x1,y1,x2,y2 = clip_bbox(x1,y1,x2,y2,W,H)
                    roi = raw_bgr[y1:y2, x1:x2]
                    facing = face_frontal_in_roi(roi)
                    new_boxes.append((x1,y1,x2,y2, facing, float(conf)))
                    if facing:
                        count_facing += 1

            curr_person_count = count_facing
            set_led_count(curr_person_count)  # LEDs según conteo
            last_boxes = new_boxes

            if curr_person_count in (1,2,3):
                maybe_announce(curr_person_count)

            # ======= TRIGGER: SOLO 4 ROSTROS FRONTALES (estable) =======
            if curr_person_count >= 4:
                consecutive_ge4 += 1
            else:
                consecutive_ge4 = 0

            if (consecutive_ge4 >= REQUIRE_CONSECUTIVE_4) and (not listos_already_played):
                print(f"[AUDIO] Trigger por 4 frontales estables: {consecutive_ge4} lecturas seguidas")
                play_listos_force(AUDIO_MAP[4], ALSA_DEVICE)
                listos_already_played = True
                detection_enabled = False
                freeze_enabled = True
                enable_button()

                freeze_frame = raw_bgr.copy()
                for (x1,y1,x2,y2,facing,conf) in last_boxes:
                    if not facing:
                        continue
                    color = (0,255,0)
                    cv2.rectangle(freeze_frame, (x1,y1), (x2,y2), color, 2)
                    label = f"person {conf:.2f} [frontal]"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(freeze_frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
                    cv2.putText(freeze_frame, label, (x1 + 2, y1 - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

                # ENVÍA "SALUDO" A LA ESP32 — la ESP32 moverá el brazo
                notify_esp32_unlock()

                # Muestra QR del hotspot de la ESP32
                freeze_frame = overlay_qr(freeze_frame, QR_ONCE, pos="br", margin=20)
                freeze_start_ts = time.time()

        # ======= Render de imagen especial si está activa =======
        if special_active:
            try:
                # Redimensionar la imagen al tamaño de la ventana
                frame_bgr = cv2.resize(special_img, (W, H), interpolation=cv2.INTER_AREA)
            except Exception as e:
                print("[IMG][ERR] Al redimensionar imagen:", e)
                special_img_until = 0.0  # cancelar

        # ======= Dibujo de cajas cuando no hay imagen especial ni congelado =======
        if (not freeze_enabled) and (not special_active) and DRAW_LAST_BOXES and last_boxes:
            for (x1,y1,x2,y2,facing,conf) in last_boxes:
                color = (0,255,0) if facing else (0,0,255)
                cv2.rectangle(frame_bgr, (x1,y1), (x2,y2), color, 2)
                label = f"person {conf:.2f}" + (" [frontal]" if facing else "")
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame_bgr, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
                cv2.putText(frame_bgr, label, (x1 + 2, y1 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        # ======= HUD =======
        if special_active:
            hud = ""
            cv2.putText(frame_bgr, hud, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)

        else:
            if freeze_enabled:
                # Texto cuando está congelado
                shown_text = f"Personas detectadas: {curr_person_count}"
                put_text_with_bg(frame_bgr, shown_text, (10, 40),
                                scale=1.0, text_color=(0,0,255), bg_color=(255,255,255),
                                thickness=2, padding=8)

                titulo = "Imagen congelada"
                txt2   = "Conectate a la red ARIA-ESP32 y escanea el codigo QR"

                put_text_with_bg(frame_bgr, titulo, (10, 90),
                                scale=0.8, text_color=(0,0,255), bg_color=(255,255,255),
                                thickness=2, padding=6)
                put_text_with_bg(frame_bgr, txt2, (10, 130),
                                scale=0.7, text_color=(0,0,255), bg_color=(255,255,255),
                                thickness=2, padding=6)

            else:
                # Texto cuando está en detección normal (también rojo con fondo blanco)
                shown_text = f"Personas (frontal): {curr_person_count}"
                put_text_with_bg(frame_bgr, shown_text, (10, 40),
                                scale=1.0, text_color=(0,0,255), bg_color=(255,255,255),
                                thickness=2, padding=8)
        # Mostrar
        cv2.imshow(WINDOW_NAME, frame_bgr)

        # Teclas (q/Esc: salir, r: reset detección, t: enviar SALUDO manual)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q') or k == 27:
            break
        if k == ord('r'):
            reset_detection_cycle()
        if k == ord('t'):
            print("[TEST] Enviando SALUDO a ESP32 desde teclado…")
            notify_esp32_unlock()

finally:
    try:
        if 'ser' in globals() and ser:
            ser.close()
    except Exception:
        pass
    try:
        picam2.stop()
    except Exception:
        pass
    try:
        set_led_count(0)
        if _event_registered:
            for p in BTN_PINS.values():
                if GPIO.gpio_function(p) == GPIO.IN:
                    try:
                        GPIO.remove_event_detect(p)
                    except Exception:
                        pass
        GPIO.cleanup()
    except Exception:
        pass
    cv2.destroyAllWindows()