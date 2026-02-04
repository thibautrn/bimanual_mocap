import time
import numpy as np
import cv2
import pyzed.sl as sl
from datetime import datetime

# ============================
# COLOR RANGES (tweakable)
# ============================
LOWER_ORANGE = np.array([10, 150, 100], dtype=np.uint8)
UPPER_ORANGE = np.array([20, 255, 255], dtype=np.uint8)

LOWER_BLUE = np.array([90, 120, 80], dtype=np.uint8)
UPPER_BLUE = np.array([130, 255, 255], dtype=np.uint8)

MIN_BALL_AREA = 300
MIN_CUP_AREA = 3000

# ============================
# CAMERA CONFIGURATION (meters & degrees)
# ============================
CAMERA_HEIGHT = 1.21
CAMERA_TILT_ANGLE = -23.0

# ============================
# DETECTION SETTINGS
# ============================
TIME_THRESHOLD = 2.0

X_TOLERANCE = 0.08
Y_TOLERANCE = 0.30
Z_TOLERANCE = 0.15


def open_zed():
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.ULTRA
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init.sdk_verbose = False

    for i in range(5):
        err = zed.open(init)
        if err == sl.ERROR_CODE.SUCCESS:
            print("✅ ZED opened")
            return zed
        print(f"⚠️ ZED open failed ({err}), retry {i+1}/5...")
        time.sleep(1)
    raise RuntimeError("❌ Failed to open ZED after retries")


def get_largest_blob(mask, min_area=300):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    return cx, cy, area, c


def camera_to_ground_frame_with_tilt(X_cam, Y_cam, Z_cam, camera_height, tilt_angle_deg):
    theta = np.radians(tilt_angle_deg)
    x = X_cam
    y = Y_cam * np.sin(theta) + Z_cam * np.cos(theta)
    z = camera_height + (Y_cam * np.cos(theta) - Z_cam * np.sin(theta))
    return x, y, z


def check_ball_in_cup(ball_pos, cup_pos, x_tol=0.08, y_tol=0.10, z_tol=0.15):
    if ball_pos is None or cup_pos is None:
        return False
    
    bx, by, bz = ball_pos
    cx, cy, cz = cup_pos
    
    x_diff = abs(bx - cx)
    y_diff = abs(by - cy)
    z_diff = abs(bz - cz)
    
    x_pass = x_diff < x_tol
    y_pass = y_diff < y_tol
    z_pass = z_diff < z_tol
    
    all_pass = x_pass and y_pass and z_pass
    
    return all_pass


def compute_reward(stats):
    """
    Compute reward based on trajectory statistics.
    
    Reward Structure:
    1. Base proximity reward (3D distance) = 0-500 points
    2. Height achievement bonus = +200 points if ball goes above cup
    3. Collision jackpot = +1000 points if ball in cup
    """
    
    min_distance = stats['min_distance_to_cup']
    ball_in_cup = stats['ball_in_cup']
    ball_went_above = stats['ball_went_above_cup']
    
    # 1. BASE PROXIMITY REWARD (0-500 points)
    MAX_DISTANCE_FOR_REWARD = 0.5  # 50cm
    if min_distance <= MAX_DISTANCE_FOR_REWARD:
        distance_reward = 500.0 * (1.0 - min_distance / MAX_DISTANCE_FOR_REWARD)
    else:
        distance_reward = 0.0
    
    # 2. HEIGHT ACHIEVEMENT BONUS (+200 points)
    height_bonus = 200.0 if ball_went_above else 0.0
    
    # 3. COLLISION JACKPOT (+1000 points)
    collision_reward = 1000.0 if ball_in_cup else 0.0
    
    # TOTAL
    total_reward = distance_reward + height_bonus + collision_reward
    
    return {
        'total_reward': total_reward,
        'distance_reward': distance_reward,
        'height_bonus': height_bonus,
        'collision_reward': collision_reward
    }



def print_reward_summary(stats, reward):
    """Print detailed reward breakdown."""
    print("\n" + "="*70)
    print("🎯 REWARD SUMMARY")
    print("="*70)
    
    status = "🎯" if stats['ball_in_cup'] else ("⬆" if stats['ball_went_above_cup'] else "○")
    
    print(f"\n{status} TOTAL REWARD: {reward['total_reward']:.1f} points\n")
    
    print("Statistics:")
    print(f"  • Ball in cup: {stats['ball_in_cup']} (+{reward['collision_reward']:.0f} pts)")
    print(f"  • Ball went above cup: {stats['ball_went_above_cup']} (+{reward['height_bonus']:.0f} pts)")
    print(f"  • Min distance to cup: {stats['min_distance_to_cup']*100:.1f}cm")
    if stats['ball_went_above_cup']:
        print(f"  • Min distance when above: {stats['min_distance_when_above']*100:.1f}cm")
    print(f"  • Max height reached: {stats['max_height']:.3f}m")
    print(f"  • Time in cup: {stats['time_in_cup']:.1f}s")
    
    print("\nReward Breakdown:")
    print(f"  1. Distance reward:         {reward['distance_reward']:6.1f} pts")
    print(f"  2. Height bonus:            {reward['height_bonus']:6.1f} pts")
    print(f"  3. Collision jackpot:       {reward['collision_reward']:6.1f} pts")
    print(f"     {'─'*40}")
    print(f"     TOTAL:                   {reward['total_reward']:6.1f} pts")
    
    print("="*70 + "\n")


def main():
    zed = open_zed()

    runtime = sl.RuntimeParameters()
    left = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    cv2.namedWindow("Ball and Cup Tracking", cv2.WINDOW_NORMAL)

    print("🎯 Ball in Cup Detection with Reward System")
    print("=" * 60)
    print("Press 'SPACE' to compute reward and reset")
    print("Press 'q' to quit")
    print("=" * 60 + "\n")

    # Tracking statistics for current attempt
    min_distance_to_cup = float('inf')
    min_distance_when_above = float('inf')
    ball_went_above_cup = False
    max_height = 0.0
    ball_in_cup = False
    time_in_cup = 0.0
    in_cup_start_time = None
    score_time = None
    
    # Reward display
    show_reward = False
    reward_display_start = None
    reward_to_display = None
    stats_to_display = None

    while True:
        current_time = time.time()
        
        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(left, sl.VIEW.LEFT)
            img_bgra = left.get_data()
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            depth_map = depth.get_data()
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

            frame = cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- BALL DETECTION ---
            ball_mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
            kernel = np.ones((5, 5), np.uint8)
            ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            ball_mask = cv2.morphologyEx(ball_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            ball_result = get_largest_blob(ball_mask, MIN_BALL_AREA)

            # --- CUP DETECTION ---
            cup_mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            cup_mask = cv2.morphologyEx(cup_mask, cv2.MORPH_OPEN, kernel, iterations=1)
            cup_mask = cv2.morphologyEx(cup_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            cup_result = get_largest_blob(cup_mask, MIN_CUP_AREA)

            # Process BALL
            ball_position = None
            if ball_result is not None:
                ball_cx, ball_cy, ball_area, ball_contour = ball_result

                err, point = point_cloud.get_value(ball_cx, ball_cy)
                X, Y, Z = point[0], point[1], point[2]

                color = (0, 255, 0) if not ball_in_cup else (0, 255, 255)
                cv2.drawContours(frame, [ball_contour], -1, color, 3)
                cv2.circle(frame, (ball_cx, ball_cy), 8, color, -1)

                if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                    ball_x, ball_y, ball_z = camera_to_ground_frame_with_tilt(
                        X, Y, Z, CAMERA_HEIGHT, CAMERA_TILT_ANGLE
                    )
                    ball_position = (ball_x, ball_y, ball_z)
                    
                    if ball_z > max_height:
                        max_height = ball_z
                    
                    txt_ball = f"Ball z={ball_z:+.2f}m"
                    cv2.putText(frame, txt_ball, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Process CUP
            cup_position = None
            cup_top_z = None
            if cup_result is not None:
                cup_cx, cup_cy, cup_area, cup_contour = cup_result

                err, point = point_cloud.get_value(cup_cx, cup_cy)
                X, Y, Z = point[0], point[1], point[2]

                cv2.drawContours(frame, [cup_contour], -1, (255, 0, 255), 2)
                cv2.circle(frame, (cup_cx, cup_cy), 6, (255, 0, 255), -1)

                if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                    cup_x, cup_y, cup_z = camera_to_ground_frame_with_tilt(
                        X, Y, Z, CAMERA_HEIGHT, CAMERA_TILT_ANGLE
                    )
                    cup_position = (cup_x, cup_y, cup_z)
                    
                    cup_top_z = cup_z + 0.07
                    
                    txt_cup = f"Cup center z={cup_z:+.2f}m"
                    cv2.putText(frame, txt_cup, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    
                    txt_top = f"Cup top z={cup_top_z:+.2f}m (+7cm)"
                    cv2.putText(frame, txt_top, (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    cv2.circle(frame, (cup_cx, cup_cy - 30), 8, (0, 255, 255), -1)

            # --- UPDATE STATISTICS ---
            if ball_position is not None and cup_position is not None:
                ball_z = ball_position[2]
                cup_z = cup_position[2]
                
                distance_3d = np.linalg.norm(np.array(ball_position) - np.array(cup_position))
                
                if distance_3d < min_distance_to_cup:
                    min_distance_to_cup = distance_3d
                
                is_above_cup = (cup_top_z is not None and ball_z > cup_top_z)
                
                if is_above_cup:
                    ball_went_above_cup = True
                    
                    if distance_3d < min_distance_when_above:
                        min_distance_when_above = distance_3d
                    
                    height_diff = (ball_z - cup_top_z) * 100
                    cv2.putText(frame, "BALL ABOVE CUP!", (frame.shape[1]//2 - 150, 120),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)
                    cv2.putText(frame, f"+{height_diff:.1f}cm", (frame.shape[1]//2 - 150, 160),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                is_in_cup = check_ball_in_cup(ball_position, cup_position, X_TOLERANCE, Y_TOLERANCE, Z_TOLERANCE)
                
                if is_in_cup and not ball_in_cup:
                    if in_cup_start_time is None:
                        in_cup_start_time = current_time
                        print("🎯 Ball meets ALL 3D criteria - timer started...")
                    
                    elapsed_time = current_time - in_cup_start_time
                    
                    if elapsed_time >= TIME_THRESHOLD:
                        ball_in_cup = True
                        score_time = current_time
                        print(f"\n🎉 GOAL! Ball is IN THE CUP!\n")
                    else:
                        remaining = TIME_THRESHOLD - elapsed_time
                        txt_countdown = f"In cup... {remaining:.1f}s remaining"
                        cv2.putText(frame, txt_countdown, (10, 230),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                elif not is_in_cup and not ball_in_cup:
                    if in_cup_start_time is not None:
                        print("⚠️ Ball no longer meets criteria - timer reset")
                    in_cup_start_time = None
                
                if ball_in_cup and score_time is not None:
                    time_in_cup = current_time - score_time
                
                if ball_in_cup:
                    txt_scored = "*** IN THE CUP ***"
                    text_size = cv2.getTextSize(txt_scored, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)[0]
                    text_x = (frame.shape[1] - text_size[0]) // 2
                    text_y = 260
                    
                    cv2.rectangle(frame, 
                                 (text_x - 10, text_y - text_size[1] - 10),
                                 (text_x + text_size[0] + 10, text_y + 10),
                                 (0, 255, 0), -1)
                    
                    cv2.putText(frame, txt_scored, (text_x, text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)
                    
                    txt_time = f"In cup for {time_in_cup:.1f}s"
                    cv2.putText(frame, txt_time, (text_x, text_y + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Display current stats (if not showing reward)
            if not show_reward:
                cv2.putText(frame, f"Min dist: {min_distance_to_cup*100:.1f}cm", (10, 120),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Max height: {max_height:.2f}m", (10, 145),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, f"Above cup: {ball_went_above_cup}", (10, 170),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.putText(frame, "Press SPACE for reward", (10, frame.shape[0] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # --- DISPLAY REWARD ON SCREEN ---
            if show_reward:
                # Check if 2 seconds elapsed
                if current_time - reward_display_start > 2.0:
                    show_reward = False
                else:
                    # Draw semi-transparent overlay
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (50, 100), (frame.shape[1] - 50, 450), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
                    
                    status = "🎯" if stats_to_display['ball_in_cup'] else ("⬆" if stats_to_display['ball_went_above_cup'] else "○")
                    
                    y_pos = 140
                    cv2.putText(frame, "=== REWARD SUMMARY ===", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                    
                    y_pos += 50
                    cv2.putText(frame, f"TOTAL: {reward_to_display['total_reward']:.0f} pts", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                    
                    y_pos += 50
                    cv2.putText(frame, f"Distance reward: {reward_to_display['distance_reward']:.0f} pts", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    y_pos += 35
                    cv2.putText(frame, f"Height bonus: +{reward_to_display['height_bonus']:.0f} pts", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    y_pos += 35
                    cv2.putText(frame, f"Collision jackpot: +{reward_to_display['collision_reward']:.0f} pts", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    y_pos += 50
                    cv2.putText(frame, f"Min distance: {stats_to_display['min_distance_to_cup']*100:.1f}cm", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                    
                    y_pos += 30
                    cv2.putText(frame, f"Max height: {stats_to_display['max_height']:.2f}m", (70, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # Show masks
            h, w = frame.shape[:2]
            
            ball_mask_bgr = cv2.cvtColor(ball_mask, cv2.COLOR_GRAY2BGR)
            ball_small = cv2.resize(ball_mask_bgr, (w // 4, h // 4))
            frame[0:ball_small.shape[0], w - ball_small.shape[1]:w] = ball_small

            cup_mask_bgr = cv2.cvtColor(cup_mask, cv2.COLOR_GRAY2BGR)
            cup_small = cv2.resize(cup_mask_bgr, (w // 4, h // 4))
            y_offset = h - cup_small.shape[0]
            frame[y_offset:h, w - cup_small.shape[1]:w] = cup_small

            cv2.imshow("Ball and Cup Tracking", frame)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        
        elif key == ord(' '):  # SPACE to compute reward
            # Collect stats
            stats = {
                'min_distance_to_cup': min_distance_to_cup,
                'min_distance_when_above': min_distance_when_above,
                'ball_went_above_cup': ball_went_above_cup,
                'max_height': max_height,
                'ball_in_cup': ball_in_cup,
                'time_in_cup': time_in_cup
            }
            
            # Compute reward
            reward = compute_reward(stats)
            
            # Print summary to console
            print_reward_summary(stats, reward)
            
            # Show on screen for 2 seconds
            show_reward = True
            reward_display_start = current_time
            reward_to_display = reward
            stats_to_display = stats
            
            # Reset for next attempt
            min_distance_to_cup = float('inf')
            min_distance_when_above = float('inf')
            ball_went_above_cup = False
            max_height = 0.0
            ball_in_cup = False
            time_in_cup = 0.0
            in_cup_start_time = None
            score_time = None
            
            print("🔄 Statistics reset. Ready for next attempt!\n")

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()