import time
import cv2
import socket
from V4sign_detector import get_sign_command, cap

#if debug that away "
###################n
# SOCKET SENDER
ROBOT_IP = "172.26.52.254"   # <- CHANGE  with robot/WSL IP
PORT = 5005 # sends at port 5005 here

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_to_robot(command: str):
    if command == "stand":
        msg = "STAND"
        print(msg, " sent")

    elif command == "idle":
        msg = "IDLE"

    elif command == "walk":
        msg = "WALK"

    elif command == "forward":
        msg = "FORWARD"

    elif command == "backward":
        msg = "BACKWARD"

    elif command == "rotate_left":
        msg = "ROTATE_LEFT"

    elif command == "rotate_right":
        msg = "ROTATE_RIGHT"


    else:
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

# Parameters
THRESHOLD = 18 #5          # frames needed to fire
DECAY = 1              # decay per frame
COOLDOWN_TIME = 2    # seconds of refractory period

last_fired = None
last_fire_time = 0


def fire_sign_command():
    global last_fired, last_fire_time

    # Get raw gesture from sign_detector
    command, frame = get_sign_command() #ADD MODE EVENTUALLY

    # Cooldown: ignore new gestures for a short time
    # internal and in parallel, so the external time.sleep(x) in main will be effective for display
    # however let's keep this one so it works when called by publisher function
    if time.time() - last_fire_time < COOLDOWN_TIME:
        return None, frame

    # Decay all accumulators
    for key in accumulators:
        accumulators[key] = max(0, accumulators[key] - DECAY)

    # If no gesture detected → return nothing
    if command is None:
        return None, frame


    # Increase accumulator for detected gesture
    accumulators[command] += 2  #1 # boost for detected gesture (+2 because decay is -1 every frame)
    print(accumulators[command]) #debug accumulation

    # Check if any accumulator crosses threshold
    # After updating accumulators elsewhere...

    # Check if any accumulator crosses the threshold and fire (unless it's the last fired)
    for gesture, value in accumulators.items():
        if value >= THRESHOLD and gesture != last_fired:
            print(f"[FIRE]{gesture.upper()}")  # (acc={value})
            last_fired = gesture
            last_fire_time = time.time()

            # Reset accumulators after firing
            for k in accumulators:
                accumulators[k] = 0

            return gesture, frame

    # Handle special case when the same command is being incremented.
    if command in INCREMENT and command == last_fired and accumulators.get(command, 0) != 0:
        if accumulators[command] % ADDUP == 0:
            print(f"[FIRE]{command.upper()}")  # (acc={value})
            time.sleep(2)

    return None, frame


if __name__ == "__main__":
    print("Running sign_pipe.py test loop... Press ESC to exit.")

    while cap.isOpened():
        command, frame = fire_sign_command()

        if command :
            send_to_robot(command)
            time.sleep(2) # cooldown not obligatory
            command = None

        # still works but now it will say "no Sign detected" if no fire
        """
        if command == "stand":
            print("Hand sign detected: STAND")

        elif command == "idle":
            print("Hand sign detected: IDLE")

        elif command == "walk":
            print("Hand sign detected: WALK")

        else:
            print("NO sign detected")
        """

        if frame is not None:
            cv2.imshow('Hand Sign Detection', frame)

        if cv2.waitKey(5) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

