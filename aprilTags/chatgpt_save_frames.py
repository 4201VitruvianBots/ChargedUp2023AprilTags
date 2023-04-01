import pathlib
import cv2
import os
import time

def save_frame_every_second(output_folder: pathlib.Path, capture_duration:float=60):
    # Ensure the output folder exists
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

    # Open the camera (0 is the default camera index)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Camera not found.")
        return

    start_time = time.time()
    frame_counter = 0

    while int(time.time() - start_time) < capture_duration:
        # Read a frame from the camera
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        current_time = int(time.time())
        if current_time % 1 == 0 and frame_counter != current_time:
            # Save the frame to the output folder
            output_path = output_folder / f"frame_{frame_counter}.png"
            cv2.imwrite(output_path, frame)
            print(f"Saved frame {frame_counter} to {output_path}")
            frame_counter += 1

        # Sleep for a short while to avoid high CPU usage
        time.sleep(0.1)

    # Release the camera resources
    cap.release()

if __name__ == "__main__":
    output_folder = pathlib.Path("captured_frames")
    capture_duration = 60  # Capture images for 60 seconds
    save_frame_every_second(output_folder, capture_duration)
