# sign_pipe.py
import time
import socket
import cv2

import V6camera_stream          # gives us cap + latest_frame + Flask server
from V6sign_detector import get_sign_command

# === ROBOT SOCKET ===
ROBOT_IP = "172.26.52.254"
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_to_robot(command: str):
    mapping = {
        "stand": "STAND",
        "idle": "IDLE",
        "walk": "WALK",
        "forward": "FORWARD",
        "backward": "BACKWARD",
        "rotate_left": "ROTATE_LEFT",
        "rotate_right": "ROTATE_RIGHT",
    }

    msg = mapping.get(command)
    if msg is None:
        print("Unknown command:", command)
        return

    sock.sendto(msg.encode(), (ROBOT_IP, PORT))
    print(f"[SENT] {msg} → {ROBOT_IP}:{PORT}")


# === ACCUMULATOR LOGIC ===
INCREMENT = ["forward", "backward", "rotate_left", "rotate_right"]
ADDUP = 10

accumulators = {
    "stand": 0,
    "idle": 0,
    "walk": 0,
    "forward": 0,
    "backward": 0,
    "rotate_left": 0,
    "rotate_right": 0
}

THRESHOLD = 18
DECAY = 1
COOLDOWN_TIME = 2

last_fired = None
last_fire_time = 0


def fire_sign_command():
    global last_fired, last_fire_time

    command, frame = get_sign_command(V6camera_stream.cap)

    if frame is None:
        return None, None

    if time.time() - last_fire_time < COOLDOWN_TIME:
        return None, frame

    for key in accumulators:
        accumulators[key] = max(0, accumulators[key] - DECAY)

    if command is None:
        return None, frame

    accumulators[command] += 2
    print(accumulators[command])

    for gesture, value in accumulators.items():
        if value >= THRESHOLD and gesture != last_fired:
            print(f"[FIRE]{gesture.upper()}")
            last_fired = gesture
            last_fire_time = time.time()
            for k in accumulators:
                accumulators[k] = 0
            return gesture, frame

    if command in INCREMENT and command == last_fired and accumulators.get(command, 0) != 0:
        if accumulators[command] % ADDUP == 0:
            print(f"[FIRE]{command.upper()}")
            return command, frame

    return None, frame


# === MAIN LOOP ===
if __name__ == "__main__":
    print("Running sign_pipe.py test loop... Press ESC to exit.")

    while V6camera_stream.cap.isOpened():
        command, frame = fire_sign_command()

        if frame is None:
            print("No frame received")
            continue

        # Update MJPEG stream
        V6camera_stream.latest_frame = frame.copy()

        # Send to robot if fired
        if command:
            send_to_robot(command)
            # robot_reply(command), speak(reply) can be added here

        # Local display
        cv2.imshow("Hand Sign Detection", frame)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    V6camera_stream.cap.release()
    cv2.destroyAllWindows()
