import cv2
import mediapipe as mp
import serial
import time
import numpy as np
from collections import deque

# ── Serial ─────────────────────────────────────────────────────────
SEND_INTERVAL = 0.05
SIMULATE = False  # set True to run without hardware

class FakeSerial:
    """Stand-in for serial.Serial that just prints what would be sent."""
    def write(self, data):
        print("[SIM] ->", data)
    def readline(self):
        return b""
    def read(self, n=1):
        return b""
    @property
    def in_waiting(self):
        return 0
    def close(self):
        pass

if SIMULATE:
    ser = FakeSerial()
else:
    try:
        ser = serial.Serial('COM6', 115200, timeout=0.01)
    except serial.SerialException as e:
        print(f"Could not open COM6 ({e}); falling back to simulation mode.")
        ser = FakeSerial()

# ── MediaPipe ──────────────────────────────────────────────────────
mp_hands = mp.solutions.hands
mp_draw  = mp.solutions.drawing_utils
hands    = mp_hands.Hands(max_num_hands=1,
                          min_detection_confidence=0.5,
                          min_tracking_confidence=0.5)

# ── Landmark indices: (wrist_base, mcp, pip, dip, tip) ─────────────
FINGER_LM = [
    (0,  5,  6,  7,  8),   # index  → bionic FIRST  (0)
    (0,  9,  10, 11, 12),  # middle → bionic MIDDLE (1)
    (0,  13, 14, 15, 16),  # ring   → bionic RING   (2)
    (0,  17, 18, 19, 20),  # pinky  → bionic LADY   (3)
]

# ── Tuning ─────────────────────────────────────────────────────────
ALPHA                 = 0.4    # EMA for curl
KNUCKLE_ALPHA         = 0.2    # EMA for abduction — smoother, abduction is slower
MAX_ANGLE             = 90.0
LM_BUFFER_SIZE        = 5
WRIST_SENSITIVITY     = 0.8
WRIST_OFFSET          = -60
KNUCKLE_MIN           = 60.0
KNUCKLE_MAX           = 120.0
ABDUCTION_SENSITIVITY = 100.0  # tune: higher = more servo travel per degree of spread

smooth_angle   = [45.0] * 5
smooth_knuckle = [90.0] * 5    # all neutral at start
lm_buffer: deque = deque(maxlen=LM_BUFFER_SIZE)

# ── Curl geometry ──────────────────────────────────────────────────
def joint_angle_deg(a, b, c, lm):
    va = lm[a] - lm[b]
    vc = lm[c] - lm[b]
    cos_a = np.dot(va, vc) / (np.linalg.norm(va) * np.linalg.norm(vc) + 1e-6)
    return float(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0))))

def calc_curl(lm, base, mcp, pip, dip, tip):
    avg = (joint_angle_deg(base, mcp, pip, lm) +
           joint_angle_deg(mcp,  pip, dip, lm) +
           joint_angle_deg(pip,  dip, tip, lm)) / 3.0
    return float(np.clip((avg - 90.0) / 90.0, 0.0, 1.0))

def calc_thumb_curl(lm):
    avg = (joint_angle_deg(1, 2, 3, lm) + joint_angle_deg(2, 3, 4, lm)) / 2.0
    return float(np.clip((avg - 90.0) / 90.0, 0.0, 1.0))

def curl_to_angle(curl):
    return round(curl * MAX_ANGLE, 1)

def calc_wrist_angle(lm):
    normal = np.cross(lm[5] - lm[0], lm[17] - lm[0])
    angle  = float(np.degrees(np.arctan2(normal[0], normal[1])))
    return float(np.clip(90.0 + WRIST_SENSITIVITY * angle + WRIST_OFFSET, 0.0, 180.0))

# ── Abduction geometry ─────────────────────────────────────────────
def calc_abduction_angles(lm):
    """
    Returns [index, middle, ring, pinky, thumb] knuckle angles in 0–180°.
    90° = neutral (fingers together). Spread apart → deviates from 90°.

    Method: project each MCP unit-vector deviation from middle-finger
    onto the palm lateral axis. Multiply by ABDUCTION_SENSITIVITY.

    Thumb: uses CMC(1)→tip(4) direction projected onto lateral axis.
    """
    wrist = lm[0]

    # Palm lateral axis — pinky MCP → index MCP (points toward index side)
    lateral = lm[5] - lm[17]
    lateral /= np.linalg.norm(lateral) + 1e-6

    # Middle finger as the neutral/reference direction
    v_mid = lm[9] - wrist
    v_mid /= np.linalg.norm(v_mid) + 1e-6

    def signed_spread(mcp_idx):
        """Signed lateral deviation of this MCP's direction from middle finger."""
        v = lm[mcp_idx] - wrist
        v /= np.linalg.norm(v) + 1e-6
        return float(np.dot(v - v_mid, lateral))

    # Index is on the positive-lateral side → positive spread → angle > 90°
    # Ring and pinky are on the negative side → negative spread → angle < 90°
    spreads = [
        signed_spread(5),    # index
        0.0,                 # middle = reference, always neutral
        signed_spread(13),   # ring
        signed_spread(17),   # pinky
    ]
    angles = [float(np.clip(90.0 + s * ABDUCTION_SENSITIVITY, KNUCKLE_MIN, KNUCKLE_MAX)) for s in spreads]
    # Thumb abduction: CMC(1) → tip(4) direction, projected onto lateral
    # Negated because thumb spreads opposite to the lateral axis direction
    tv = lm[4] - lm[1]
    tv /= np.linalg.norm(tv) + 1e-6
    thumb_spread = float(np.dot(tv, lateral))
    angles.append(float(np.clip(90.0 - thumb_spread * ABDUCTION_SENSITIVITY, KNUCKLE_MIN, KNUCKLE_MAX)))
    return angles  # [index, middle, ring, pinky, thumb]

def thumb_collision_safe(angles):
    a = angles[:]
    if a[4] > 30.0 and (a[0] < 30.0 or a[1] < 30.0):
        a[4] = 30.0
    return a

# ── Main loop ──────────────────────────────────────────────────────
cap       = cv2.VideoCapture(0)
last_sent = 0.0

while cap.isOpened():
    ok, img = cap.read()
    if not ok:
        continue

    results = hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

    if results.multi_hand_landmarks:
        raw_lm = results.multi_hand_landmarks[0].landmark
        lm_buffer.append(np.array([[l.x, l.y, l.z] for l in raw_lm], dtype=np.float32))

        mp_draw.draw_landmarks(img, results.multi_hand_landmarks[0], mp_hands.HAND_CONNECTIONS)

        if len(lm_buffer) < LM_BUFFER_SIZE:
            cv2.imshow("Bionic Hand Control", img)
            if cv2.waitKey(1) & 0xFF == 27: break
            continue

        lm = np.mean(lm_buffer, axis=0)

        # ── Curl ──────────────────────────────────────────────────
        raw_angles = [curl_to_angle(calc_curl(lm, *f)) for f in FINGER_LM]
        raw_angles.append(curl_to_angle(calc_thumb_curl(lm)))

        for i in range(5):
            smooth_angle[i] = ALPHA * raw_angles[i] + (1 - ALPHA) * smooth_angle[i]

        angles = thumb_collision_safe([round(smooth_angle[i], 1) for i in range(5)])
        wrist  = round(calc_wrist_angle(lm), 1)

        # ── Abduction ─────────────────────────────────────────────
        raw_knuckle = calc_abduction_angles(lm)
        for i in range(5):
            smooth_knuckle[i] = KNUCKLE_ALPHA * raw_knuckle[i] + (1 - KNUCKLE_ALPHA) * smooth_knuckle[i]
        knuckle = [round(smooth_knuckle[i], 1) for i in range(5)]

        # ── Serial send ───────────────────────────────────────────
        now = time.time()
        if now - last_sent >= SEND_INTERVAL:
            pkt = (f"P {angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]}"
                   f" {wrist}"
                   f" {knuckle[0]} {knuckle[1]} {knuckle[2]} {knuckle[3]} {knuckle[4]}\n")
            ser.write(pkt.encode())
            last_sent = now

        # ── Overlay ───────────────────────────────────────────────
        for i, name in enumerate(["1st", "Mid", "Ring", "Lady", "Thm"]):
            cv2.putText(img, f"{name} curl:{angles[i]:.0f} abd:{knuckle[i]:.0f}",
                        (10, 30 + i * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(img, f"Wrist: {wrist:.0f}",
                    (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

    cv2.imshow("Bionic Hand Control", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
