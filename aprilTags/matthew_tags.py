import pathlib
import cv2
import os
import time

import robotpy_apriltag

from icecream import ic

import matplotlib.pyplot as plt

def save_frame_every_second(output_folder: pathlib.Path, capture_duration: float = 60):
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
        image_file_name = input('image filename:')

        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture frame.")
            break

        current_time = int(time.time())
        if current_time % 1 == 0 and frame_counter != current_time:
            # Save the frame to the output folder
            output_path = output_folder / f"{image_file_name}.png"
            cv2.imwrite(str(output_path), frame)
            print(f"Saved frame {frame_counter} to {output_path}")
            frame_counter += 1

        # Sleep for a short while to avoid high CPU usage
        time.sleep(0.1)

    # Release the camera resources
    cap.release()


def loop_save_frames_with_text():
    output_folder = pathlib.Path("captured_frames")
    capture_duration = 60  # Capture images for 60 seconds
    save_frame_every_second(output_folder, capture_duration)


def show_frame():
    frame_directory = pathlib.Path('captured_frames')
    for image_file in frame_directory.glob('*.png'):
        image = cv2.imread(str(image_file))
        image = image[:, :, 0]
        detector = robotpy_apriltag.AprilTagDetector()

        # detector config
        detector_config = robotpy_apriltag.AprilTagDetector.Config()
        detector_config.refineEdges = 1.0
        detector_config.quadDecimate = 1.0
        detector_config.numThreads = 3
        detector_config.quadSigma = 1.0
        detector_config.decodeSharpening = 0.25
        detector.setConfig(detector_config)

        # print detections
        detector.addFamily("tag16h5", 3)
        detections = detector.detect(image)

        # calls tag filter
        tag_filter(detections)

        # cycle images
        plt.imshow(image, cmap='gray', vmin=0, vmax=image.max())
        plt.show()
        # cv2.imshow(str(image_file), image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()


def tag_filter(detections):
    # cycles through every detected tag and prints detection corners
    # 3 - 0 right side; 2 - 1 left side; right side - left side = if tag is usable
    tag_threshold = 10

    if len(detections) > 0:
        for tag in detections:
            tag_corners = [tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)]

            right_side = point_dist(tag_corners[3], tag_corners[0])
            left_side = point_dist(tag_corners[2], tag_corners[1])

            side_diff = abs(right_side - left_side)

            if side_diff < tag_threshold:
                continue
            else:
                ic(tag_corners)
                ic(side_diff)
                print(tag)
                continue


def point_dist(p1: robotpy_apriltag.AprilTagDetection.Point, p2: robotpy_apriltag.AprilTagDetection.Point):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


if __name__ == "__main__":
    show_frame()
