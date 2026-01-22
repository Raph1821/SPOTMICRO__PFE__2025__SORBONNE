import cv2
from sign_detector import get_sign_command, cap

while cap.isOpened():
    command, frame = get_sign_command()

    if command == "stand":
        print("Hand sign detected: STAND")

    elif command == "idle":
        print("Hand sign detected: IDLE")


    elif command == "walk":
        print("Hand sign detected: WALK")

    else:
        print("NO sign detected")

    if frame is not None:
        cv2.imshow('Hand Sign Detection', frame)

    if cv2.waitKey(5) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
