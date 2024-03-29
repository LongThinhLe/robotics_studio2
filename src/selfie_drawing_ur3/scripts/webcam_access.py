import cv2

def main():
    # Access the webcam
    cap = cv2.VideoCapture(0)

    # Check if the webcam is opened successfully
    if not cap.isOpened():
        print("Error: Could not access the webcam.")
        return

    # Loop to continuously capture frames from the webcam
    while True:
        # Capture a frame
        ret, frame = cap.read()

        # Check if the frame is captured successfully
        if not ret:
            print("Error: Failed to capture frame.")
            break

        # Display the frame
        cv2.imshow('Webcam Feed', frame)

        # Check for 'q' key press to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the webcam and close all OpenCV windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
