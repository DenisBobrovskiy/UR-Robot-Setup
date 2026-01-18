#!/usr/bin/env python3
"""
Maps MediaPipe pose landmarks to UR robot arm joint positions.
Tracks human arm movement and mirrors it on the robot arm.
"""
import asyncio
import numpy as np
import cv2
import time
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import mediapipe as mp
from client_core import URRobotClient
from typing import Optional, List, Tuple


class PoseToRobotMapper:
    """Maps human pose landmarks to robot joint angles."""
    
    def __init__(self):
        # MediaPipe landmark indices for right arm
        self.SHOULDER = 12  # Right shoulder
        self.ELBOW = 14     # Right elbow
        self.WRIST = 16     # Right wrist
        self.HIP = 24       # Right hip (for reference)
        
        # Joint angle limits (in degrees from image)
        self.joint_limits = {
            'shoulder_pan': (144, 144),      # Rotation around vertical axis
            'shoulder_lift': (-95, -95),     # Lift up/down
            'elbow': (-107, -107),           # Elbow bend
            'wrist_1': (-32, -32),           # Wrist pitch
            'wrist_2': (-251, -251),         # Wrist roll
            'wrist_3': (-1, -1)              # Wrist rotation
        }
        
        # Smoothing buffer
        self.angle_history = []
        self.history_size = 5
    
    def calculate_angle_3d(self, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray) -> float:
        """Calculate angle at p2 given three 3D points."""
        v1 = p1 - p2
        v2 = p3 - p2
        
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        return np.degrees(angle)
    
    def calculate_rotation_angle(self, p1: np.ndarray, p2: np.ndarray, reference_axis: str = 'y') -> float:
        """Calculate rotation angle in a plane."""
        vector = p2 - p1
        
        if reference_axis == 'y':
            # Rotation around Y axis (shoulder pan)
            angle = np.arctan2(vector[0], vector[2])
        elif reference_axis == 'x':
            # Rotation around X axis
            angle = np.arctan2(vector[1], vector[2])
        else:
            angle = 0
        
        return np.degrees(angle)
    
    def landmarks_to_joint_angles(self, landmarks) -> Optional[List[float]]:
        """Convert MediaPipe landmarks to robot joint angles."""
        if not landmarks or len(landmarks) < 25:
            return None
        
        # Extract 3D positions
        shoulder = np.array([landmarks[self.SHOULDER].x, 
                           landmarks[self.SHOULDER].y, 
                           landmarks[self.SHOULDER].z])
        elbow = np.array([landmarks[self.ELBOW].x, 
                        landmarks[self.ELBOW].y, 
                        landmarks[self.ELBOW].z])
        wrist = np.array([landmarks[self.WRIST].x, 
                        landmarks[self.WRIST].y, 
                        landmarks[self.WRIST].z])
        hip = np.array([landmarks[self.HIP].x, 
                      landmarks[self.HIP].y, 
                      landmarks[self.HIP].z])
        
        # Calculate angles
        # Shoulder pan: horizontal rotation (left-right)
        shoulder_pan = self.calculate_rotation_angle(hip, shoulder, 'y') * 2
        shoulder_pan = np.clip(shoulder_pan, -180, 180)
        
        # Shoulder lift: vertical angle (up-down)
        torso_to_shoulder = shoulder - hip
        shoulder_to_elbow = elbow - shoulder
        shoulder_lift = -np.degrees(np.arctan2(shoulder_to_elbow[1], 
                                               np.linalg.norm(shoulder_to_elbow[[0, 2]])))
        shoulder_lift = np.clip(shoulder_lift, -180, 0)
        
        # Elbow: bend angle
        elbow_angle = self.calculate_angle_3d(shoulder, elbow, wrist)
        elbow_joint = -(180 - elbow_angle)  # Convert to robot convention
        elbow_joint = np.clip(elbow_joint, -180, 0)
        
        # Wrist angles (simplified - based on wrist orientation)
        elbow_to_wrist = wrist - elbow
        wrist_1 = -np.degrees(np.arctan2(elbow_to_wrist[1], 
                                        np.linalg.norm(elbow_to_wrist[[0, 2]])))
        wrist_1 = np.clip(wrist_1, -180, 180)
        
        # Wrist 2 & 3: simplified roll and rotation
        wrist_2 = 0  # Could be calculated from hand landmarks if available
        wrist_3 = 0
        
        # Convert to radians for robot
        angles = [
            np.radians(shoulder_pan),
            np.radians(shoulder_lift),
            np.radians(elbow_joint),
            np.radians(wrist_1),
            np.radians(wrist_2),
            np.radians(wrist_3)
        ]
        
        # Smooth angles
        self.angle_history.append(angles)
        if len(self.angle_history) > self.history_size:
            self.angle_history.pop(0)
        
        smoothed = np.mean(self.angle_history, axis=0)
        return smoothed.tolist()


class PoseRobotController:
    """Main controller combining pose detection and robot control."""
    
    def __init__(self, robot_host: str = "localhost", robot_port: int = 8765):
        self.robot_host = robot_host
        self.robot_port = robot_port
        self.mapper = PoseToRobotMapper()
        self.robot: Optional[URRobotClient] = None
        self.is_running = False
        self.control_enabled = False
        self.last_move_time = 0
        self.move_interval = 0.1  # Send commands every 100ms
        
        # Safety limits
        self.initial_position: Optional[List[float]] = None
        self.max_deviation_deg = 6.0  # Maximum deviation in degrees
        self.safety_limit_active = True
    
    def draw_landmarks_on_image(self, rgb_image, detection_result):
        """Draw pose landmarks on image."""
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z) 
                for lm in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                pose_landmarks_proto,
                solutions.pose.POSE_CONNECTIONS,
                solutions.drawing_styles.get_default_pose_landmarks_style())
        
        return annotated_image
    
    def apply_safety_limits(self, target_angles: List[float]) -> Tuple[List[float], bool]:
        """
        Apply safety limits to target angles.
        Returns: (limited_angles, is_within_limits)
        """
        if not self.initial_position or not self.safety_limit_active:
            return target_angles, True
        
        limited_angles = []
        all_within_limits = True
        max_deviation_rad = np.radians(self.max_deviation_deg)
        
        for i, (target, initial) in enumerate(zip(target_angles, self.initial_position)):
            deviation = target - initial
            
            # Clamp deviation to max allowed
            if abs(deviation) > max_deviation_rad:
                all_within_limits = False
                clamped_deviation = np.clip(deviation, -max_deviation_rad, max_deviation_rad)
                limited_angle = initial + clamped_deviation
            else:
                limited_angle = target
            
            limited_angles.append(limited_angle)
        
        return limited_angles, all_within_limits
    
    async def run(self):
        """Main control loop."""
        # Initialize camera
        cam = cv2.VideoCapture(0)
        if not cam.isOpened():
            print("Error: Could not open camera")
            return
        
        # Initialize MediaPipe
        base_options = python.BaseOptions(model_asset_path='pose_landmarker.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=False,
            running_mode=vision.RunningMode.VIDEO)
        detector = vision.PoseLandmarker.create_from_options(options)
        
        # Connect to robot
        try:
            self.robot = URRobotClient(self.robot_host, self.robot_port)
            await self.robot.connect()
            print(f"Connected to robot at {self.robot_host}:{self.robot_port}")
            await self.robot.wait_for_state()
            
            # Record initial position for safety limits
            if self.robot.positions:
                self.initial_position = self.robot.positions.copy()
                print(f"Initial position recorded: {[np.degrees(p) for p in self.initial_position]}")
                print(f"Safety limit: ±{self.max_deviation_deg}° from initial position")
            
            print("Robot ready!")
        except Exception as e:
            print(f"Could not connect to robot: {e}")
            print("Running in visualization-only mode")
        
        self.is_running = True
        frame_count = 0
        
        print("\nControls:")
        print("  SPACE - Toggle robot control on/off")
        print("  'h' - Move robot to home position")
        print("  'r' - Reset initial position (current pose becomes new reference)")
        print("  's' - Toggle safety limits on/off")
        print("  'q' - Quit")
        print("\nRobot control is OFF by default. Press SPACE to enable.\n")
        
        try:
            while self.is_running:
                ret, frame = cam.read()
                if not ret:
                    break
                
                frame_count += 1
                
                # Process with MediaPipe
                image = mp.Image(image_format=mp.ImageFormat.SRGB, 
                               data=np.asarray(frame))
                detection_result = detector.detect_for_video(
                    image, int(time.time() * 1000))
                
                # Draw landmarks
                annotated_image = self.draw_landmarks_on_image(
                    image.numpy_view(), detection_result)
                
                # Convert to joint angles and control robot
                if detection_result.pose_landmarks and len(detection_result.pose_landmarks) > 0:
                    landmarks = detection_result.pose_landmarks[0]
                    joint_angles = self.mapper.landmarks_to_joint_angles(landmarks)
                    
                    # Display angles on screen
                    if joint_angles:
                        y_offset = 30
                        joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 
                                     'wrist_1', 'wrist_2', 'wrist_3']
                        
                        # Apply safety limits and check
                        limited_angles, within_limits = self.apply_safety_limits(joint_angles)
                        
                        for i, (name, angle, limited) in enumerate(zip(joint_names, joint_angles, limited_angles)):
                            deviation = 0
                            if self.initial_position:
                                deviation = np.degrees(angle - self.initial_position[i])
                            
                            # Color code based on safety limit
                            color = (0, 255, 0) if abs(deviation) < self.max_deviation_deg else (0, 165, 255)
                            text = f"{name}: {np.degrees(limited):.1f}° (Δ{deviation:+.1f}°)"
                            cv2.putText(annotated_image, text, (10, y_offset + i*25),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        # Show safety limit warning if exceeded
                        if not within_limits and self.safety_limit_active:
                            cv2.putText(annotated_image, "SAFETY LIMIT ACTIVE", 
                                      (10, y_offset + 170),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                        
                        # Send to robot if enabled
                        if self.control_enabled and self.robot and self.robot.is_ready:
                            current_time = time.time()
                            if current_time - self.last_move_time > self.move_interval:
                                try:
                                    await self.robot.move_joints(limited_angles, duration=0.2)
                                    self.last_move_time = current_time
                                except Exception as e:
                                    print(f"Robot move error: {e}")
                
                # Display status
                status_text = "CONTROLLING ROBOT" if self.control_enabled else "PREVIEW ONLY"
                color = (0, 255, 0) if self.control_enabled else (0, 0, 255)
                cv2.putText(annotated_image, status_text, (10, annotated_image.shape[0] - 50),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
                
                # Display safety status
                safety_status = f"Safety: {'ON' if self.safety_limit_active else 'OFF'} (±{self.max_deviation_deg}°)"
                safety_color = (0, 255, 0) if self.safety_limit_active else (0, 0, 255)
                cv2.putText(annotated_image, safety_status, (10, annotated_image.shape[0] - 20),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, safety_color, 2)
                
                # Show frame
                cv2.imshow("Pose Robot Control", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
                
                # Handle keypresses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):
                    self.control_enabled = not self.control_enabled
                    status = "enabled" if self.control_enabled else "disabled"
                    print(f"Robot control {status}")
                elif key == ord('h') and self.robot:
                    print("Moving to home position...")
                    try:
                        await self.robot.move_to_named("home", duration=3.0)
                        # Update initial position after moving home
                        await asyncio.sleep(3.5)
                        if self.robot.positions:
                            self.initial_position = self.robot.positions.copy()
                            print("Initial position updated to home")
                    except Exception as e:
                        print(f"Home move error: {e}")
                elif key == ord('r') and self.robot:
                    # Reset initial position to current position
                    if self.robot.positions:
                        self.initial_position = self.robot.positions.copy()
                        print(f"Initial position reset to: {[np.degrees(p) for p in self.initial_position]}")
                elif key == ord('s'):
                    # Toggle safety limits
                    self.safety_limit_active = not self.safety_limit_active
                    status = "enabled" if self.safety_limit_active else "disabled"
                    print(f"Safety limits {status}")
                    if not self.safety_limit_active:
                        print("WARNING: Robot can now move freely!")
                
                # Allow other async tasks to run
                await asyncio.sleep(0.001)
        
        finally:
            # Cleanup
            cam.release()
            cv2.destroyAllWindows()
            if self.robot:
                await self.robot.disconnect()
            print("Shutdown complete")


async def main():
    """Entry point."""
    import sys
    
    # Parse command line args
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
    
    controller = PoseRobotController(host, port)
    await controller.run()


if __name__ == '__main__':
    asyncio.run(main())