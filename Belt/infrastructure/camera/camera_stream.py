import cv2
import threading


class CameraStream:

    def __init__(self, camera_index=0):

        self.camera_index = camera_index

        self.cap = None

        self.running = False

        self.thread = None

        self.frame_callback = None

    # =====================================================
    # START
    # =====================================================

    def start(self):

        if self.running:
            return

        self.cap = cv2.VideoCapture(
            self.camera_index
        )

        if not self.cap.isOpened():

            raise Exception("Cannot open camera")

        self.running = True

        self.thread = threading.Thread(
            target=self.loop,
            daemon=True
        )

        self.thread.start()

    # =====================================================
    # STOP
    # =====================================================

    def stop(self):

        self.running = False

        if self.thread:

            self.thread.join(timeout=1)

        if self.cap:

            self.cap.release()

    # =====================================================
    # LOOP
    # =====================================================

    def loop(self):

        while self.running:

            ret, frame = self.cap.read()

            if not ret:
                continue

            frame = cv2.resize(
                frame,
                (640, 480)
            )

            rgb = cv2.cvtColor(
                frame,
                cv2.COLOR_BGR2RGB
            )

            # emit frame
            if self.frame_callback:

                self.frame_callback(rgb)