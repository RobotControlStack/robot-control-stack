import cv2
'''
Simple script for opening a webcam using OpenCV and displaying the video feed.
'''
def open_webcam():
    # Open the default webcam (0).
    # If you have multiple cameras, you can try 1, 2, etc.
    cap = cv2.VideoCapture("/dev/video8")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Loop to continuously get frames
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        # Show the frame in a window
        cv2.imshow("Webcam", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    open_webcam()
