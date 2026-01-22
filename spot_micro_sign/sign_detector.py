import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Load the HandLandmarker
base_options = python.BaseOptions(model_asset_path='task_models_for_mediapipe/hand_landmarker.task')
options = vision.HandLandmarkerOptions(base_options=base_options, num_hands=1)
detector = vision.HandLandmarker.create_from_options(options)


# MediaPipe hand connections (21 points)
HAND_CONNECTIONS = [
    (0,1), (1,2), (2,3), (3,4),        # Thumb
    (0,5), (5,6), (6,7), (7,8),        # Index
    (5,9), (9,10), (10,11), (11,12),   # Middle
    (9,13), (13,14), (14,15), (15,16), # Ring
    (13,17), (17,18), (18,19), (19,20) # Pinky
]

def draw_landmarks(image, landmarks):
    h, w, _ = image.shape

    # Draw connections
    for start, end in HAND_CONNECTIONS:
        x1, y1 = int(landmarks[start].x * w), int(landmarks[start].y * h)
        x2, y2 = int(landmarks[end].x * w), int(landmarks[end].y * h)
        cv2.line(image, (x1, y1), (x2, y2), (0, 255, 255), 2)

    # Draw points
    for lm in landmarks:
        cx, cy = int(lm.x * w), int(lm.y * h)
        cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)


cap = cv2.VideoCapture(0)



def is_open_hand(landmarks):
    # Check if index, middle, ring, pinky are extended
    fingers = []
    for tip, pip in [(8, 6), (12, 10), (16, 14), (20, 18)]:
        fingers.append(landmarks[tip].y < landmarks[pip].y)

    # Thumb check (simple horizontal distance)
    thumb = abs(landmarks[4].x - landmarks[2].x) > 0.05

    return all(fingers) and thumb


def is_love_hand(landmarks):
    # Check if a finger is extended (tip.y < pip.y)
    def ext(tip, pip):
        return landmarks[tip].y < landmarks[pip].y

    # Thumb uses x‑axis separation
    thumb = abs(landmarks[4].x - landmarks[2].x) > 0.05

    index  = ext(8, 6)
    middle = ext(12, 10)
    ring   = ext(16, 14)
    pinky  = ext(20, 18)

    # ASL "I Love You" = thumb + index + pinky extended, middle + ring folded
    return thumb and index and pinky and not middle and not ring

def is_ok_hand(landmarks):
    # Distance between thumb tip (4) and index tip (8)
    thumb_tip = landmarks[4]
    index_tip = landmarks[8]

    # thumb and index close to form cercle
    dist = ((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2) ** 0.5
    circle_close = dist < 0.10 #0.05

    # Middle, ring, pinky extended
    def ext(tip, pip):
        return landmarks[tip].y < landmarks[pip].y

    middle = ext(12, 10)
    ring   = ext(16, 14)
    pinky  = ext(20, 18)

    # Index should be bent (tip below pip)
    index_bent = landmarks[8].y > landmarks[6].y

    return circle_close and middle and ring and pinky and index_bent



def get_sign_command():
    ret, frame_flipped = cap.read()
    frame = cv2.flip(frame_flipped, 1)

    if not ret:
        return None, frame

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb) # Convert the NumPy array to Mediapipe Image format
    results = detector.detect(mp_image)

    if not results.hand_landmarks:
        return None, frame

    landmarks = results.hand_landmarks[0]  # Access the first detected hand

    draw_landmarks(frame, landmarks)


    # Detect open hand → STAND
    if is_open_hand(landmarks):
        return "stand", frame

    # Detect love hand → idle
    if is_love_hand(landmarks):
        return "idle", frame

    # Detect ok hand → walk
    if is_ok_hand(landmarks):
        return "walk", frame

    return None, frame

# debug
''' MAIN
# Main loop to continuously detect hand signs
while cap.isOpened():
    command, frame = get_sign_command()
    if command == "stand":
        print("Hand sign detected: STAND")

    # Display the frame
    if frame is not None:
        cv2.imshow('Hand Sign Detection', frame)
    if cv2.waitKey(5) & 0xFF == 27:  # Escape key
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
'''