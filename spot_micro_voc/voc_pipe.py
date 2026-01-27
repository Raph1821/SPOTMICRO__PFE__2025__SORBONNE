import time
import socket
from voc_detector import ecouter_commandes  # nouveau module vocal

'''
    USE sign_ros_call.py VARIANT TO CONVERT INFO INTO ROS COMMAND, SHARED PIPELINE FILE

'''


###################
# SOCKET SENDER
###################
ROBOT_IP = "172.26.52.254"   # <- CHANGE avec l’IP du robot/WSL
PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_to_robot(command: str):
    if command == "stand":
        msg = "STAND"
        print(msg, "sent")

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


if __name__ == "__main__":
    print("=== SYSTEME DE COMMANDE VOCALE FR ===")
    print("Dites : 'avance', 'recule', 'gauche', 'droite', 'marche', 'immobile', etc.\n")

    while True:
        command = ecouter_commandes()

        if command:
            send_to_robot(command)

        time.sleep(0.1)
