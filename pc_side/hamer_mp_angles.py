"""
live_hamer_v2.py — Async pipeline + MANO angle display for bionic hand
Controls: Q/ESC=quit  S=save  P=print angles
"""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import torch
import numpy as np
import mediapipe as mp
from pathlib import Path
import threading
import queue
import time
import serial   #type: ignore
import serial.tools.list_ports  #type: ignore
from pytorch3d.transforms import matrix_to_axis_angle   # moved to top

# ── HaMeR imports ─────────────────────────────────────────────────────────────
from hamer.models import load_hamer, DEFAULT_CHECKPOINT
from hamer.utils import recursive_to
from hamer.datasets.vitdet_dataset import ViTDetDataset, DEFAULT_MEAN, DEFAULT_STD
from hamer.utils.renderer import Renderer, cam_crop_to_full

# ── Config ────────────────────────────────────────────────────────────────────
CHECKPOINT     = DEFAULT_CHECKPOINT
RESCALE_FACTOR = 2.0
CAMERA_ID      = 0
DISPLAY_WIDTH  = 1280
BBOX_PADDING   = 0.2
SAVE_DIR       = Path('output')
SAVE_DIR.mkdir(exist_ok=True)
RENDER_EVERY_N = 5

# ── Angle tuning — change these first if values look wrong ───────────────────
# Each is 0, 1, or 2 (x, y, z of local MANO joint frame)
CURL_AXIS  = 2    # flexion axis at each finger joint — try 2 if curl stays flat
ABD_AXIS   = 1    # abduction axis at MCP — try 2 if no spread detected
WRIST_AXIS = 1    # global orient axis for wrist flex/extension — try 1 or 2

CURL_OFFSET = [0.0, 0.0, 0.0, 0.0, 15.0]
CURL_SCALE = 1.1  # multiply if curl range feels too small
ABD_SCALE  = 1.5  # multiply if abduction is barely moving

WRIST_SCALE  = 1.0   # increase if wrist feels sluggish, decrease if too twitchy
WRIST_OFFSET = -70.0   # positive shifts toward closed, negative toward open — tune in steps of 10

# ── Serial ────────────────────────────────────────────────────────────────────
SERIAL_PORT   = '/dev/ttyUSB0'   # change to /dev/ttyACM0 if USB0 not found
SERIAL_BAUD   = 115200
SEND_INTERVAL = 0.05             # 20 Hz

# ── MANO joint mapping — bionic hand order ────────────────────────────────────
# Joints per finger: (label, MCP_idx, PIP_idx, DIP_idx)
# MANO order: Index=0-2, Middle=3-5, Pinky=6-8, Ring=9-11, Thumb=12-14
MANO_FINGERS = [
    ('1st',  0,  1,  2),   # Index  → bionic 1st finger
    ('Mid',  3,  4,  5),   # Middle → bionic Middle
    ('Ring', 9,  10, 11),  # Ring   → bionic Ring
    ('Lady', 6,  7,  8),   # Pinky  → bionic Lady finger
    ('Thm',  12, 13, 14),  # Thumb  → bionic Thumb
]

# ── Device ────────────────────────────────────────────────────────────────────
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f"Device: {device}")

# ── Serial init ───────────────────────────────────────────────────────────────
try:
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.01)
    print(f"Serial: {SERIAL_PORT} @ {SERIAL_BAUD}")
except serial.SerialException as e:
    ser = None
    print(f"Serial not connected ({e}) — running display-only")

def send_packet(curls, abds, wrist):
    """Collision clamp then send. Mirrors original MediaPipe packet format."""
    c = curls[:]
    # Thumb (index 4) vs 1st (0) and Middle (1) — clamp if collision risk
    if c[4] > 30.0 and (c[0] > 60.0 or c[1] > 60.0):
        c[4] = 30.0
    pkt = (f"P {c[0]} {c[1]} {c[2]} {c[3]} {c[4]}"
           f" {wrist}"
           f" {abds[0]} {abds[1]} {abds[2]} {abds[3]} {abds[4]}\n")
    ser.write(pkt.encode())

# ── Load HaMeR ────────────────────────────────────────────────────────────────
print("Loading HaMeR...")
model, model_cfg = load_hamer(CHECKPOINT)
model = model.half().to(device).eval()

# ── Renderer ──────────────────────────────────────────────────────────────────
renderer = Renderer(model_cfg, faces=model.mano.faces)

# ── MediaPipe ─────────────────────────────────────────────────────────────────
print("Loading MediaPipe...")
mp_hands = mp.solutions.hands
hands_detector = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.5,
)

# ── Webcam ────────────────────────────────────────────────────────────────────
cap = cv2.VideoCapture(CAMERA_ID)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open camera {CAMERA_ID}")

# ── Shared state ──────────────────────────────────────────────────────────────
detection_queue = queue.Queue(maxsize=1)
stop_event      = threading.Event()


# ── Angle extraction ──────────────────────────────────────────────────────────
def mano_to_angles(hand_pose_mat, global_orient_mat):
    aa = matrix_to_axis_angle(hand_pose_mat)   # [15, 3], radians

    curls, abds = [], []
    for _, mcp, pip, dip in MANO_FINGERS:

        # ── Curl ──────────────────────────────────────────────────────────
        if len(curls) == 4:   # thumb — skip MCP, only PIP+DIP
            flex_rad = (aa[pip, CURL_AXIS] + aa[dip, CURL_AXIS]).item()
            curl = float(np.clip(np.degrees(flex_rad) * CURL_SCALE / 2.0 + CURL_OFFSET[4], 0.0, 90.0))
        else:
            i = len(curls)
            flex_rad = (aa[mcp, CURL_AXIS] + aa[pip, CURL_AXIS] + aa[dip, CURL_AXIS]).item()
            curl = float(np.clip(np.degrees(flex_rad) * CURL_SCALE / 3.0 + CURL_OFFSET[i], 0.0, 90.0))
        curls.append(curl)

        # ── Abduction ─────────────────────────────────────────────────────
        abd = float(np.clip(90.0 + np.degrees(aa[mcp, ABD_AXIS].item()) * ABD_SCALE, 60.0, 120.0))
        abds.append(abd)

    # ── Wrist ─────────────────────────────────────────────────────────────
    go_aa = matrix_to_axis_angle(global_orient_mat.squeeze(0))
    wrist  = float(np.clip(90.0 + np.degrees(go_aa[WRIST_AXIS].item()) * WRIST_SCALE + WRIST_OFFSET, 0.0, 180.0))

    return curls, abds, wrist

# ── Thread A — Capture + MediaPipe ────────────────────────────────────────────
def detection_thread():
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            stop_event.set()
            break

        H, W = frame.shape[:2]
        frame  = cv2.flip(frame, 1)
        rgb    = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_out = hands_detector.process(rgb)

        if not mp_out.multi_hand_landmarks:
            try:
                detection_queue.put_nowait((frame, None, None))
            except queue.Full:
                pass
            continue

        bboxes, is_right = [], []
        for hand_lms, handedness in zip(mp_out.multi_hand_landmarks,
                                         mp_out.multi_handedness):
            xs = [lm.x * W for lm in hand_lms.landmark]
            ys = [lm.y * H for lm in hand_lms.landmark]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
            bw, bh = x2 - x1, y2 - y1
            x1 = max(0, x1 - bw * BBOX_PADDING)
            y1 = max(0, y1 - bh * BBOX_PADDING)
            x2 = min(W, x2 + bw * BBOX_PADDING)
            y2 = min(H, y2 + bh * BBOX_PADDING)
            bboxes.append([x1, y1, x2, y2])
            label = handedness.classification[0].label
            is_right.append(0 if label == 'Right' else 1)

        boxes = np.array(bboxes, dtype=np.float32)
        right = np.array(is_right, dtype=np.int32)
        try:
            detection_queue.put_nowait((frame, boxes, right))
        except queue.Full:
            pass


det_thread = threading.Thread(target=detection_thread, daemon=True)
det_thread.start()
print("Running. Q/ESC=quit  S=save  P=print angles")

# ── Main loop — Thread B ──────────────────────────────────────────────────────
frame_count  = 0
fps_times    = []
last_render  = None   # clean mesh image, no overlays (reused every RENDER_EVERY_N)
last_angles  = None   # (curls, abds, wrist, side_str) — updated every frame
save_counter = 0
last_sent    = 0.0 

while not stop_event.is_set():

    try:
        frame, boxes, right = detection_queue.get(timeout=1.0)
    except queue.Empty:
        continue

    t0 = time.perf_counter()
    frame_count += 1
    H, W = frame.shape[:2]

    # ── No hands ──────────────────────────────────────────────────────────────
    if boxes is None:
        display = frame.copy()
        cv2.putText(display, "No hands", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('HaMeR Live', display)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            stop_event.set()
        continue

    # ── HaMeR inference ───────────────────────────────────────────────────────
    img_size   = frame.shape[:2]
    dataset    = ViTDetDataset(model_cfg, frame, boxes, right,
                               rescale_factor=RESCALE_FACTOR)
    dataloader = torch.utils.data.DataLoader(dataset, batch_size=8,
                                             shuffle=False, num_workers=0)

    all_verts, all_cam_t, all_right_out   = [], [], []
    all_hand_poses, all_global_orients    = [], []
    scaled_focal = None

    for batch in dataloader:
        batch = recursive_to(batch, device)
        with torch.no_grad():
            with torch.amp.autocast('cuda'):
                out = model(batch)

        # Collect MANO rotation matrices (float32 on CPU)
        all_hand_poses.append(out['pred_mano_params']['hand_pose'].cpu().float())       # [B,15,3,3]
        all_global_orients.append(out['pred_mano_params']['global_orient'].cpu().float()) # [B, 1,3,3]

        multiplier      = (2 * batch['right'] - 1)
        pred_cam        = out['pred_cam']
        pred_cam[:, 1] *= multiplier
        box_center      = batch['box_center'].float()
        box_size        = batch['box_size'].float()
        img_size_t      = batch['img_size'].float()
        scaled_focal    = (model_cfg.EXTRA.FOCAL_LENGTH
                           / model_cfg.MODEL.IMAGE_SIZE
                           * img_size_t.max())
        pred_cam_t_full = cam_crop_to_full(
            pred_cam, box_center, box_size, img_size_t, scaled_focal
        ).detach().cpu().numpy()

        all_verts.extend([v.detach().cpu().numpy() for v in out['pred_vertices']])
        all_cam_t.extend(list(pred_cam_t_full))
        all_right_out.extend(batch['right'].cpu().numpy())

    # ── Pick target hand for angle extraction ─────────────────────────────────
    # Prefer right hand (is_right=1); fall back to first detected
    rights     = np.array(all_right_out)
    target_idx = int(np.argmax(rights)) if rights.any() else 0

    hand_poses  = torch.cat(all_hand_poses,    dim=0)   # [N, 15, 3, 3]
    global_oris = torch.cat(all_global_orients, dim=0)  # [N,  1, 3, 3]

    curls, abds, wrist = mano_to_angles(
        hand_poses[target_idx],      # [15, 3, 3]
        global_oris[target_idx],     # [ 1, 3, 3]
    )
    last_angles = (curls, abds, wrist, "R" if rights[target_idx] else "L")

    # ── Serial send (20 Hz, non-blocking gate) ────────────────────────────────
    now = time.time()
    if ser and (now - last_sent >= SEND_INTERVAL):
        send_packet(curls, abds, wrist)
        last_sent = now

    # ── Render every N frames (Option 2) ──────────────────────────────────────
    do_render = (frame_count % RENDER_EVERY_N == 0) or (last_render is None)

    if do_render:
        focal_val = (scaled_focal.item() if scaled_focal.dim() == 0
                     else scaled_focal[0].item())
        cam_view  = renderer.render_rgba_multiple(
            all_verts, cam_t=all_cam_t,
            render_res=img_size[::-1],
            is_right=all_right_out,
            mesh_base_color=(1.0, 1.0, 0.9),
            scene_bg_color=(1, 1, 1),
            focal_length=focal_val,
        )
        rgb_mesh    = cam_view[:, :, :3]
        mask        = cam_view[:, :, 3:4]
        clean_img   = (rgb_mesh * mask + (frame / 255.0) * (1 - mask))
        clean_img   = (clean_img * 255).clip(0, 255).astype(np.uint8)
        last_render = clean_img   # store WITHOUT overlays so reused frames are clean

    out_img = last_render.copy()   # always copy so overlays are drawn fresh

    # ── Angle overlay (always fresh, even on skipped render frames) ───────────
    if last_angles:
        curls, abds, wrist, side = last_angles
        labels = [row[0] for row in MANO_FINGERS]
        for i, lbl in enumerate(labels):
            cv2.putText(out_img,
                        f"{lbl}  curl:{curls[i]:5.1f}  abd:{abds[i]:+6.1f}",
                        (10, 70 + i * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 128), 2)
        cv2.putText(out_img,
                    f"Wrist: {wrist:.1f}  [{side}]",
                    (10, 70 + 5 * 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    # ── FPS overlay ───────────────────────────────────────────────────────────
    t1 = time.perf_counter()
    fps_times.append(t1 - t0)
    if len(fps_times) > 30:
        fps_times.pop(0)
    avg_fps    = 1.0 / (sum(fps_times) / len(fps_times))
    render_tag = "R" if do_render else "-"
    cv2.putText(out_img,
                f"Hands:{len(all_verts)}  FPS:{avg_fps:.1f}  [{render_tag}]",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # ── Resize + display ──────────────────────────────────────────────────────
    h, w = out_img.shape[:2]
    if w > DISPLAY_WIDTH:
        scale   = DISPLAY_WIDTH / w
        out_img = cv2.resize(out_img, (DISPLAY_WIDTH, int(h * scale)))

    cv2.imshow('HaMeR Live', out_img)

    # ── Key handling ──────────────────────────────────────────────────────────
    key = cv2.waitKey(1) & 0xFF

    if key in (ord('q'), 27):
        stop_event.set()

    elif key == ord('s'):
        p = SAVE_DIR / f"live_{save_counter:04d}.jpg"
        cv2.imwrite(str(p), out_img)
        print(f"Saved: {p}")
        save_counter += 1

    elif key == ord('p') and last_angles:
        curls, abds, wrist, side = last_angles
        labels = [row[0] for row in MANO_FINGERS]
        print(f"\n── MANO Angles ({side} hand) ──")
        for lbl, c, a in zip(labels, curls, abds):
            print(f"  {lbl:5s}  curl: {c:6.1f}°   abd: {a:+7.1f}°")
        print(f"  Wrist: {wrist:.1f}°")

# ── Cleanup ───────────────────────────────────────────────────────────────────
cap.release()
if ser:
    ser.close()
hands_detector.close()
cv2.destroyAllWindows()
det_thread.join(timeout=2.0)
print("Done.")
