import threading
import time
import rospy
from std_msgs.msg import String
from sign_detector import get_sign_command, cap

class GestureObserver:
    def __init__(self, target="stand", threshold=10, cooldown=3):
        self.target = target
        self.threshold = threshold
        self.cooldown = cooldown

        self.count = 0
        self.cooldown_until = 0
        self.running = False

        # ROS publisher
        self.pub = rospy.Publisher("/gesture_command", String, queue_size=10)

    def process_gesture(self, gesture):
        now = time.time()

        # Cooldown active → ignore everything
        if now < self.cooldown_until:
            return

        # Count consecutive detections
        if gesture == self.target:
            self.count += 1
        else:
            self.count = 0

        # Trigger event
        if self.count >= self.threshold:
            self.count = 0
            self.cooldown_until = now + self.cooldown

            # Publish ROS command
            self.pub.publish(f"{self.target}_triggered")
            print(f"[ROS] Published: {self.target}_triggered")

    def run(self):
        self.running = True
        print("[Observer] Started listening for gestures...")

        while self.running and cap.isOpened():
            gesture, frame = get_sign_command()
            self.process_gesture(gesture)

            # No blocking — stays async
            time.sleep(0.01)

    def stop(self):
        self.running = False
        print("[Observer] Stopped.")


def start_observer():
    rospy.init_node("gesture_observer", anonymous=True)
    observer = GestureObserver()

    # Run observer in background thread
    thread = threading.Thread(target=observer.run, daemon=True)
    thread.start()

    return observer
