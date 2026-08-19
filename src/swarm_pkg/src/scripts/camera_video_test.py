#!/usr/bin/env python3
"""Live camera view with Canny edge detection (no ROS2)."""
import cv2


def main():
    cap = cv2.VideoCapture(0)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            processed = cv2.Canny(frame, 100, 200)
            processed = cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)

            cv2.imshow("wrist_camera raw", frame)
            cv2.imshow("wrist_camera processed", processed)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        cap.release()


if __name__ == "__main__":
    main()
