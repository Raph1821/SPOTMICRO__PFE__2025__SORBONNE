# sign_detector.py
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Load the HandLandmarker
base_options = python.BaseOptions(
    model_asset_path='task_models_for_mediapipe/hand_landmarker.task'
)
options = vision.HandLandmarkerOptions(
    base_options=base_options,
    num_hands=1
)
detector = vision.HandLandmarker.create_from_options(options)

HAND_CONNECTIONS = [
    (0,1), (1,2), (2,3), (3,4),
    (0,5), (5,6), (6,7), (7,8),
    (5,9), (9,10), (10,11), (11,12),
    (9,13), (13,14), (14,15), (15,16),
    (13,17), (17,18), (18,19), (19,20)
]


def draw_landmarks(image, landmarks):
    h, w, _ = image.shape

    for start, end in HAND_CONNECTIONS:
        x1, y1 = int(landmarks[start].x * w), int(landmarks[start].y * h)
        x2, y2 = int(landmarks[end].x * w), int(landmarks[end].y * h)
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 255), 2)

    for lm in landmarks:
        cx, cy = int(lm.x * w), int(lm.y * h)
        cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)


def put_label(frame, text):
    cv2.putText(frame, text, (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (0, 255, 0), 3, cv2.LINE_AA)


# === your gesture predicates here (unchanged) ===
# is_open_hand, is_love_hand, is_ok_hand, is_index_pointing,
# index_direction, is_forward_military, is_backward_drink_thumb, etc.
# (reuse your existing implementations)


def get_sign_command(cap):
    ret, frame_flipped = cap.read()
    if not ret:
        return None, None

    frame = cv2.flip(frame_flipped, 1)

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    results = detector.detect(mp_image)

    if not results.hand_landmarks:
        return None, frame

    landmarks = results.hand_landmarks[0]
    draw_landmarks(frame, landmarks)

    # Example mapping with labels
    if is_open_hand(landmarks):
        put_label(frame, "STAND")
        return "stand", frame

    if is_love_hand(landmarks):
        put_label(frame, "IDLE")
        return "idle", frame

    if is_ok_hand(landmarks):
        put_label(frame, "WALK")
        return "walk", frame

    if is_index_pointing(landmarks):
        direction = index_direction(landmarks)
        put_label(frame, direction.upper())
        return direction, frame

    if is_forward_military(landmarks):
        put_label(frame, "FORWARD")
        return "forward", frame

    if is_backward_drink_thumb(landmarks):
        put_label(frame, "BACKWARD")
        return "backward", frame

    return None, frame
