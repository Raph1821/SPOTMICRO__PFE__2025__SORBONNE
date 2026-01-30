# sign_pipe.py
import time
import socket
import cv2

from robot_responses import robot_reply
from robot_tts import speak


import V6camera_stream # gives us cap + latest_frame + Flask server
from V6sign_detector import get_sign_command # sign

#if debug that away "
###################n
# SOCKET SENDER
ROBOT_IP = "172.26.52.254"   # CHANGE with robot/WSL IP
PORT = 5005  # sends at port 5005 here

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



####################
# BIOINSPIRED SAMPLER WRAPPER FOR sign_detector

INCREMENT = ["forward", "backward", "rotate_left", "rotate_right"]
ADDUP = 10


# Neuron-like accumulators
accumulators = {
    "stand": 0,
    "idle": 0,
    "walk": 0,
    "forward": 0,
    "backward": 0,
    "rotate_left": 0,
    "rotate_right": 0
}

THRESHOLD = 18 # frames needed to fire
DECAY = 1 # decay per frame
COOLDOWN_TIME = 2 # seconds cooldown

last_fired = None
last_fire_time = 0



# label for return frames
def put_label(frame, text):
    cv2.putText(frame, text, (30, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (0, 0, 255), 3, cv2.LINE_AA)




def fire_sign_command():
    global last_fired, last_fire_time

    # Get raw gesture from sign_detector
    command, frame = get_sign_command(V6camera_stream.cap)

    # Cooldown: ignore new gestures for a short time
    # internal and in parallel, so the external time.sleep(x) in main will be effective for display
    # however let's keep this one so it works when called by publisher function
    if frame is None:
        # put_label(frame, "Sending...")
        # time.sleep(2)
        return None, None

    if time.time() - last_fire_time < COOLDOWN_TIME:
        return None, frame

    # Decay all accumulators
    for key in accumulators:
        accumulators[key] = max(0, accumulators[key] - DECAY)

    # If no gesture detected return nothing
    if command is None:
        return None, frame

    # Increase accumulator for detected gesture
    accumulators[command] += 2
    print(accumulators[command])

    # Check if any accumulator crosses the threshold and fire (unless it's the last fired)
    for gesture, value in accumulators.items():
        if value >= THRESHOLD and gesture != last_fired:
            print(f"[FIRE]{gesture.upper()}")
            last_fired = gesture
            last_fire_time = time.time()
            for k in accumulators:
                accumulators[k] = 0
            return gesture, frame

    # Handle special case when the same command is being incremented.
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
            put_label(frame, "Sending...")
            cv2.imshow('Hand Sign Detection', frame)

            ### voc chatbot
            reply = robot_reply(command)
            print("🐶🤖 Spotmini :", reply)
            speak(reply)
            ###


        # Local display
        if frame is not None:
            cv2.imshow("Hand Sign Detection", frame)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    V6camera_stream.cap.release()
    cv2.destroyAllWindows()
