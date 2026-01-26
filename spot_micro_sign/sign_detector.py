import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# selecting MODE in prevision of way of command
IMPULSE = False
ADDUP = True

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

# thumb detection not used

def is_thumbpose(landmarks):
    # Thumb extended
    thumb_extended = abs(landmarks[4].x - landmarks[2].x) > 0.05

    # Fingers closed
    def folded(tip, pip):
        return landmarks[tip].y > landmarks[pip].y

    fingers_closed = (
        folded(8,6) and folded(12,10) and folded(16,14) and folded(20,18)
    )

    return thumb_extended and fingers_closed


def thumb_direction(landmarks):
    dx = landmarks[4].x - landmarks[2].x
    dy = landmarks[4].y - landmarks[2].y

    if abs(dx) > abs(dy):
        return "right" if dx > 0 else "left"
    else:
        return "backward" if dy > 0 else "forward"



def is_index_pointing(landmarks):
    # Index extended
    index_ext = landmarks[8].y < landmarks[6].y

    # Other fingers folded
    middle_fold = landmarks[12].y > landmarks[10].y
    ring_fold   = landmarks[16].y > landmarks[14].y
    pinky_fold  = landmarks[20].y > landmarks[18].y

    return index_ext and middle_fold and ring_fold and pinky_fold


def index_direction(landmarks):
    # Vector from index MCP (5) to index TIP (8)
    dx = landmarks[8].x - landmarks[5].x
    dy = landmarks[8].y - landmarks[5].y

    if abs(dx) > abs(dy):
        return "right" if dx > 0 else "left"
    """# if we want index pointing on those direction (PS : backward is clumsy)
    else:
        return "backward" if dy > 0 else "forward"
    """

def is_two_fingers_parallel(landmarks):
    # Index + middle extended
    index_ext  = landmarks[8].y  < landmarks[6].y
    middle_ext = landmarks[12].y < landmarks[10].y

    # Ring + pinky folded
    ring_fold   = landmarks[16].y > landmarks[14].y
    pinky_fold  = landmarks[20].y > landmarks[18].y

    # Fingers must be close together (parallel, not V)
    close_together = abs(landmarks[8].x - landmarks[12].x) < 0.03

    return index_ext and middle_ext and ring_fold and pinky_fold and close_together


def is_forward_direction(landmarks):
    # Midpoint of index + middle tips
    tip_x = (landmarks[8].x + landmarks[12].x) / 2
    tip_z = (landmarks[8].z + landmarks[12].z) / 2

    # Midpoint of index + middle MCP
    mcp_x = (landmarks[5].x + landmarks[9].x) / 2
    mcp_z = (landmarks[5].z + landmarks[9].z) / 2

    dx = tip_x - mcp_x
    dz = tip_z - mcp_z

    forward = dz < -0.05      # toward camera
    diagonal = abs(dx) > 0.03 # slight sideways tilt

    return forward and diagonal


def is_forward_military(landmarks):
    return is_two_fingers_parallel(landmarks) and is_forward_direction(landmarks)


def is_backward_drink_thumb(landmarks):
    # this works well because all fingers are seen + palm by mediapip e
    # 1. All fingers folded (index, middle, ring, pinky)
    index_fold  = landmarks[8].y  > landmarks[6].y
    middle_fold = landmarks[12].y > landmarks[10].y
    ring_fold   = landmarks[16].y > landmarks[14].y
    pinky_fold  = landmarks[20].y > landmarks[18].y

    fingers_folded = index_fold and middle_fold and ring_fold and pinky_fold

    if not fingers_folded: # detection problem if we don't confirm
        return False

    # 2. Thumb extended sideways
    thumb_side = abs(landmarks[4].x - landmarks[2].x) > 0.05

    # 3. Thumb pointing backward (away from camera)
    # thumb tip farther from camera than thumb MCP
    thumb_tip_z = landmarks[4].z
    thumb_mcp_z = landmarks[2].z
    thumb_back = thumb_tip_z > thumb_mcp_z + 0.05

    # 4. Diagonal backward tilt (x component)
    dx = landmarks[4].x - landmarks[2].x
    diagonal = abs(dx) > 0.03

    #print(thumb_side, thumb_back, diagonal) #debug
    return thumb_side and diagonal



def get_sign_command():
    ret, frame_flipped = cap.read()
    frame = cv2.flip(frame_flipped, 1)

    if not ret:
        return None, frame#, None

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb) # Convert the NumPy array to Mediapipe Image format
    results = detector.detect(mp_image)

    if not results.hand_landmarks:
        return None, frame#, None

    landmarks = results.hand_landmarks[0]  # Access the first detected hand

    draw_landmarks(frame, landmarks)


    # Detect open hand → STAND, IMPULSE
    if is_open_hand(landmarks):
        return "stand", frame#, IMPULSE

    # Detect love hand → idle, IMPULSE
    if is_love_hand(landmarks):
        return "idle", frame#, IMPULSE

    # Detect ok hand → walk, IMPULSE
    if is_ok_hand(landmarks):
        return "walk", frame#, IMPULSE

    # Detect ok hand → direction , ADDUP
    if is_index_pointing(landmarks):
        return index_direction(landmarks), frame#, ADDUP

    # FORWARD gesture
    if is_forward_military(landmarks):
        return "forward", frame

    # BACKWARD gesture
    if is_backward_drink_thumb(landmarks):
        return "backward", frame

    ############ UNUSED
    """
    if is_thumbpose(landmarks):
        direction = thumb_direction(landmarks)
        return direction, frame
    """
    #############


    return None, frame#, None


# debug
"""#MAIN
# Main loop to continuously detect hand signs
while cap.isOpened():
    command, frame, mode = get_sign_command()
    if command == "forward":
        print("Hand sign detected: forward")

    # Display the frame
    if frame is not None:
        cv2.imshow('Hand Sign Detection', frame)
    if cv2.waitKey(5) & 0xFF == 27:  # Escape key
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
"""