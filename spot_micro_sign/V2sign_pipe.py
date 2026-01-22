import time
import cv2
from sign_detector import get_sign_command, cap

# Neuron-like accumulators
accumulators = {
    "stand": 0,
    "idle": 0,
    "walk": 0
}

# Parameters (tunable)
THRESHOLD = 12 #5          # frames needed to fire
DECAY = 1              # decay per frame
COOLDOWN_TIME = 2    # seconds of refractory period

last_fired = None
last_fire_time = 0


def fire_sign_command():
    global last_fired, last_fire_time

    # Get raw gesture from sign_detector
    command, frame = get_sign_command()

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
    for gesture, value in accumulators.items():
        if value >= THRESHOLD and gesture != last_fired:
            # FIRE the gesture
            print(f"[FIRE]{gesture.upper()}") # (acc={value})")

            last_fired = gesture
            last_fire_time = time.time()

            # Reset accumulators after firing
            for k in accumulators:
                accumulators[k] = 0

            return gesture, frame

    return None, frame


if __name__ == "__main__":
    print("Running sign_pipe.py test loop... Press ESC to exit.")

    while cap.isOpened():
        command, frame = fire_sign_command()
        if command :
            time.sleep(2) # cooldown not obligatory

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

