#!/usr/bin/env python3
"""
Maps MediaPipe pose landmarks to UR robot arm joint positions.
Uses RELATIVE movements from home position, like the dance demo.
Max deviation: ±10 degrees from starting position.
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
    """Maps human pose CHANGES to small robot joint deltas."""
    
    def __init__(self):
        # MediaPipe landmark indices for right arm
        self.SHOULDER = 12
        self.ELBOW = 14
        self.WRIST = 16
        self.HIP = 24
        
        # Reference pose (calibrated when user presses 'c')
        self.reference_pose: Optional[dict] = None
        
        # Smoothing
        self.angle_history = []
        self.history_size = 5
        
        # Sensitivity: how much robot moves per degree of human movement
        # Lower = less sensitive, more controlled
        self.sensitivity = 0.3
    
    def extract_pose_angles(self, landmarks) -> Optional[dict]:
        """Extract key angles from pose landmarks."""
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
        
        # Calculate human pose angles
        # Shoulder rotation (left-right lean)
        shoulder_vec = shoulder - hip
        shoulder_pan = np.degrees(np.arctan2(shoulder_vec[0], shoulder_vec[2] + 1e-6))
        
        # Arm lift (up-down)
        arm_vec = elbow - shoulder
        shoulder_lift = np.degrees(np.arctan2(-arm_vec[1], np.linalg.norm(arm_vec[[0, 2]]) + 1e-6))
        
        # Elbow bend
        v1 = shoulder - elbow
        v2 = wrist - elbow
        cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
        elbow_angle = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0)))
        
        # Wrist angle (forearm direction)
        forearm_vec = wrist - elbow
        wrist_angle = np.degrees(np.arctan2(-forearm_vec[1], np.linalg.norm(forearm_vec[[0, 2]]) + 1e-6))
        
        return {
            'shoulder_pan': shoulder_pan,
            'shoulder_lift': shoulder_lift,
            'elbow': elbow_angle,
            'wrist': wrist_angle,
        }
    
    def calibrate(self, landmarks):
        """Set current pose as reference (neutral position)."""
        self.reference_pose = self.extract_pose_angles(landmarks)
        self.angle_history = []
        return self.reference_pose is not None
    
    def pose_to_deltas(self, landmarks, max_delta_deg: float = 10.0) -> Optional[List[float]]:
        """
        Convert pose to joint DELTAS (not absolute positions).
        Returns deltas in radians, clamped to max_delta_deg.
        """
        if self.reference_pose is None:
            return None
        
        current_pose = self.extract_pose_angles(landmarks)
        if current_pose is None:
            return None
        
        # Calculate differences from reference pose
        diff_shoulder_pan = current_pose['shoulder_pan'] - self.reference_pose['shoulder_pan']
        diff_shoulder_lift = current_pose['shoulder_lift'] - self.reference_pose['shoulder_lift']
        diff_elbow = current_pose['elbow'] - self.reference_pose['elbow']
        diff_wrist = current_pose['wrist'] - self.reference_pose['wrist']
        
        # Map to robot joints with sensitivity scaling
        # Clamp each to max_delta_deg
        def clamp_and_scale(diff, scale=1.0):
            scaled = diff * self.sensitivity * scale
            return np.clip(scaled, -max_delta_deg, max_delta_deg)
        
        deltas_deg = [
            clamp_and_scale(diff_shoulder_pan, 1.0),    # shoulder_pan
            clamp_and_scale(diff_shoulder_lift, 0.8),   # shoulder_lift
            clamp_and_scale(diff_elbow, 0.5),           # elbow (less sensitive)
            clamp_and_scale(diff_wrist, 0.3),           # wrist_1 (even less)
            0.0,                                         # wrist_2 (locked)
            clamp_and_scale(diff_shoulder_pan, 0.5),    # wrist_3 (follows pan slightly)
        ]
        
        # Smooth the deltas
        self.angle_history.append(deltas_deg)
        if len(self.angle_history) > self.history_size:
            self.angle_history.pop(0)
        
        smoothed_deg = np.mean(self.angle_history, axis=0)
        
        # Convert to radians
        smoothed_rad = [np.radians(d) for d in smoothed_deg]
        return smoothed_rad


class PoseRobotController:
    """Main controller - moves robot relative to home position."""
    
    def __init__(self, robot_host: str = "localhost", robot_port: int = 8765):
        self.robot_host = robot_host
        self.robot_port = robot_port
        self.mapper = PoseToRobotMapper()
        self.robot: Optional[URRobotClient] = None
        self.is_running = False
        self.control_enabled = False
        self.last_move_time = 0
        self.move_interval = 0.15  # 150ms between commands
        
        # Home position (will be fetched from robot)
        self.home_position: Optional[List[float]] = None
        
        # Safety: max deviation from home (10 degrees)
        self.max_deviation_deg = 10.0
    
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
            
            if not await self.robot.wait_for_state(timeout=5.0):
                print("Failed to get robot state")
                return
            
            # Get home position from robot
            self.home_position = self.robot.positions.copy()
            print(f"Home position: {[f'{np.degrees(p):.1f}°' for p in self.home_position]}")
            print(f"Max deviation: ±{self.max_deviation_deg}°")
            print("Robot ready!")
            
        except Exception as e:
            print(f"Could not connect to robot: {e}")
            print("Running in visualization-only mode")
        
        self.is_running = True
        
        print("\n" + "="*50)
        print("CONTROLS:")
        print("  'c'   - CALIBRATE pose (stand in neutral position first!)")
        print("  SPACE - Toggle robot control ON/OFF")
        print("  'h'   - Move robot to home position")
        print("  '+/-' - Adjust sensitivity")
        print("  'q'   - Quit")
        print("="*50)
        print("\n⚠️  CALIBRATE FIRST! Stand neutral, then press 'c'\n")
        
        try:
            while self.is_running:
                ret, frame = cam.read()
                if not ret:
                    break
                
                # Process with MediaPipe
                image = mp.Image(image_format=mp.ImageFormat.SRGB, 
                               data=np.asarray(frame))
                detection_result = detector.detect_for_video(
                    image, int(time.time() * 1000))
                
                # Draw landmarks
                annotated_image = self.draw_landmarks_on_image(
                    image.numpy_view(), detection_result)
                
                # Process pose if detected
                if detection_result.pose_landmarks and len(detection_result.pose_landmarks) > 0:
                    landmarks = detection_result.pose_landmarks[0]
                    
                    # Get deltas from pose
                    deltas = self.mapper.pose_to_deltas(landmarks, self.max_deviation_deg)
                    
                    # Display info
                    y_offset = 30
                    
                    if self.mapper.reference_pose is None:
                        cv2.putText(annotated_image, "NOT CALIBRATED - Press 'c'", 
                                  (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    elif deltas:
                        joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow', 
                                     'wrist_1', 'wrist_2', 'wrist_3']
                        
                        for i, (name, delta) in enumerate(zip(joint_names, deltas)):
                            delta_deg = np.degrees(delta)
                            # Color based on magnitude
                            intensity = min(abs(delta_deg) / self.max_deviation_deg, 1.0)
                            color = (0, int(255 * (1 - intensity)), int(255 * intensity))
                            
                            text = f"{name}: {delta_deg:+.1f}°"
                            cv2.putText(annotated_image, text, (10, y_offset + i*25),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                        
                        # Send to robot
                        if self.control_enabled and self.robot and self.home_position:
                            current_time = time.time()
                            if current_time - self.last_move_time > self.move_interval:
                                try:
                                    # Calculate target = home + delta
                                    target = [h + d for h, d in zip(self.home_position, deltas)]
                                    await self.robot.move_joints(target, duration=0.2)
                                    self.last_move_time = current_time
                                except Exception as e:
                                    print(f"Move error: {e}")
                
                # Status display
                status_y = annotated_image.shape[0] - 80
                
                # Calibration status
                cal_status = "CALIBRATED" if self.mapper.reference_pose else "NOT CALIBRATED"
                cal_color = (0, 255, 0) if self.mapper.reference_pose else (0, 0, 255)
                cv2.putText(annotated_image, f"Pose: {cal_status}", (10, status_y),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, cal_color, 2)
                
                # Control status
                ctrl_status = "CONTROLLING ROBOT" if self.control_enabled else "PREVIEW ONLY"
                ctrl_color = (0, 255, 0) if self.control_enabled else (0, 165, 255)
                cv2.putText(annotated_image, ctrl_status, (10, status_y + 25),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, ctrl_color, 2)
                
                # Sensitivity
                cv2.putText(annotated_image, f"Sensitivity: {self.mapper.sensitivity:.1f}", 
                          (10, status_y + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Show frame
                cv2.imshow("Pose Robot Control", cv2.cvtColor(annotated_image, cv2.COLOR_RGB2BGR))
                
                # Handle keypresses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                    
                elif key == ord('c'):
                    # Calibrate
                    if detection_result.pose_landmarks:
                        landmarks = detection_result.pose_landmarks[0]
                        if self.mapper.calibrate(landmarks):
                            print("✓ Pose calibrated! Move your arm to control robot.")
                        else:
                            print("✗ Calibration failed - make sure you're visible")
                    
                elif key == ord(' '):
                    if self.mapper.reference_pose is None:
                        print("⚠️  Calibrate first! Press 'c' while in neutral pose")
                    else:
                        self.control_enabled = not self.control_enabled
                        print(f"Robot control {'ENABLED' if self.control_enabled else 'DISABLED'}")
                    
                elif key == ord('h') and self.robot:
                    print("Moving to home...")
                    self.control_enabled = False
                    try:
                        await self.robot.move_to_named("home", duration=2.0)
                        await asyncio.sleep(2.5)
                        # Update home position
                        if self.robot.positions:
                            self.home_position = self.robot.positions.copy()
                        print("At home position")
                    except Exception as e:
                        print(f"Home error: {e}")
                
                elif key == ord('+') or key == ord('='):
                    self.mapper.sensitivity = min(1.0, self.mapper.sensitivity + 0.1)
                    print(f"Sensitivity: {self.mapper.sensitivity:.1f}")
                    
                elif key == ord('-'):
                    self.mapper.sensitivity = max(0.1, self.mapper.sensitivity - 0.1)
                    print(f"Sensitivity: {self.mapper.sensitivity:.1f}")
                
                await asyncio.sleep(0.001)
        
        finally:
            cam.release()
            cv2.destroyAllWindows()
            
            # Return to home on exit
            if self.robot and self.home_position:
                print("Returning to home...")
                try:
                    await self.robot.move_to_named("home", duration=2.0)
                    await asyncio.sleep(2.5)
                except:
                    pass
                await self.robot.disconnect()
            
            print("Shutdown complete")


async def main():
    """Entry point."""
    import sys
    
    host = sys.argv[1] if len(sys.argv) > 1 else "localhost"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8765
    
    controller = PoseRobotController(host, port)
    await controller.run()


if __name__ == '__main__':
    asyncio.run(main())