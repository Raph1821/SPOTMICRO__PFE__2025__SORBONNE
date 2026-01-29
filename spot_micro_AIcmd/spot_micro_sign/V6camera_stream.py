# V6camera_stream.py
import cv2
import threading
from flask import Flask, Response

###############################################
USE_PHONE_CAMERA = False
PHONE_STREAM_URL =  "http://admin:vanre@192.168.43.28:8081/video" # check this from time to time because it changes
###############################################

# Shared objects
latest_frame = None
cap = None

# Init camera
if USE_PHONE_CAMERA:
    cap = cv2.VideoCapture(PHONE_STREAM_URL)
else:
    cap = cv2.VideoCapture(0)

# Flask app
app = Flask(__name__)


def mjpeg_stream():
    global latest_frame
    while True:
        if latest_frame is None:
            continue

        # Optional flip for phone viewing
        #processed = cv2.flip(latest_frame, 1)

        ret, jpeg = cv2.imencode('.jpg', latest_frame)
        frame_bytes = jpeg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


@app.route('/processed')
def processed():
    return Response(mjpeg_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/snapshot')
def snapshot():
    global latest_frame
    if latest_frame is None:
        return "No frame yet", 503

    ret, jpeg = cv2.imencode('.jpg', latest_frame)
    return Response(jpeg.tobytes(), mimetype='image/jpeg')


def start_server():
    app.run(host="0.0.0.0", port=8080, debug=False, threaded=True)


# Start Flask in background as soon as this module is imported
threading.Thread(target=start_server, daemon=True).start()
