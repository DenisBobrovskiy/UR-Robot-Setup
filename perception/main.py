from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import mediapipe as mp
import numpy as np
import cv2
import time

def draw_landmarks_on_image(rgb_image, detection_result):
  pose_landmarks_list = detection_result.pose_landmarks
  annotated_image = np.copy(rgb_image)

  # Loop through the detected poses to visualize.
  for idx in range(len(pose_landmarks_list)):
    pose_landmarks = pose_landmarks_list[idx]

    # Draw the pose landmarks.
    pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
    pose_landmarks_proto.landmark.extend([
      landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
    ])
    solutions.drawing_utils.draw_landmarks(
      annotated_image,
      pose_landmarks_proto,
      solutions.pose.POSE_CONNECTIONS,
      solutions.drawing_styles.get_default_pose_landmarks_style())
  return annotated_image

def main():
  # Open the default camera
  cam = cv2.VideoCapture(0)

  base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
  options = vision.PoseLandmarkerOptions(
      base_options=base_options,
      output_segmentation_masks=True,
      running_mode=vision.RunningMode.VIDEO)
  detector = vision.PoseLandmarker.create_from_options(options)

  while True:
  # for i in range(10): # process just one frame
      ret, frame = cam.read()

      # Load the frame into mediapipe
      image = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.asarray(frame))
      
      # Run inference.
      detection_result = detector.detect_for_video(image, int(time.time() * 1000))

      # Display the captured frame
      # cv2.imshow('Camera', frame)
      annotated_image = draw_landmarks_on_image(image.numpy_view(), detection_result)
      cv2.imshow("Camera", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
      cv2.waitKey(1)

      # Press 'q' to exit the loop
      if cv2.waitKey(1) == ord('q'):
          break

  # Release the capture and writer objects
  cam.release()
  cv2.destroyAllWindows()

if __name__ == '__main__':
  main()