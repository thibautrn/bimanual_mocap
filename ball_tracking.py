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

LOWER_BLUE = np.array([90, 120, 80], dtype=np.uint8)  # Increased S from 80 to 120, V from 40 to 80
UPPER_BLUE = np.array([130, 255, 255], dtype=np.uint8)

MIN_BALL_AREA = 300
MIN_CUP_AREA = 3000

# ============================
# CAMERA CONFIGURATION (meters & degrees)
# ============================
CAMERA_HEIGHT = 1.21
CAMERA_TILT_ANGLE = -23.0

# ============================
# DETECTION SETTINGS - ADJUST THESE!
# ============================
TIME_THRESHOLD = 2.0

X_TOLERANCE = 0.08
Y_TOLERANCE = 0.30
Z_TOLERANCE = 0.15

# ============================
# RECORDING SETTINGS
# ============================
RECORDING_HZ = 40
RECORD_INTERVAL = 1.0 / RECORDING_HZ


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


def get_highest_point(contour):
    """Get the highest point in the contour (lowest y pixel value)."""
    if contour is None or len(contour) == 0:
        return None
    # In image coords, y=0 is top, so minimum y = highest point
    min_y_idx = contour[:, :, 1].argmin()
    return tuple(contour[min_y_idx][0])


def camera_to_ground_frame_with_tilt(X_cam, Y_cam, Z_cam, camera_height, tilt_angle_deg):
    theta = np.radians(tilt_angle_deg)
    x = X_cam
    y = Y_cam * np.sin(theta) + Z_cam * np.cos(theta)
    z = camera_height + (Y_cam * np.cos(theta) - Z_cam * np.sin(theta))
    return x, y, z


def check_ball_in_cup(ball_pos, cup_pos, x_tol=0.08, y_tol=0.10, z_tol=0.15):
    if ball_pos is None or cup_pos is None:
        return False, {"error": "Missing position"}
    
    bx, by, bz = ball_pos
    cx, cy, cz = cup_pos
    
    x_diff = abs(bx - cx)
    y_diff = abs(by - cy)
    z_diff = abs(bz - cz)
    
    x_pass = x_diff < x_tol
    y_pass = y_diff < y_tol
    z_pass = z_diff < z_tol
    
    all_pass = x_pass and y_pass and z_pass
    
    debug_info = {
        'x_diff': x_diff,
        'x_pass': x_pass,
        'y_diff': y_diff,
        'y_pass': y_pass,
        'z_diff': z_diff,
        'z_pass': z_pass,
        'all_pass': all_pass
    }
    
    return all_pass, debug_info


def main():
    zed = open_zed()

    runtime = sl.RuntimeParameters()
    left = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    cv2.namedWindow("Ball and Cup Tracking", cv2.WINDOW_NORMAL)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_file = f"tracking_data_{timestamp}.txt"
    
    print("🎯 Ball in Cup Detection - 3D Position Based")
    print("=" * 60)
    print(f"Camera height: {CAMERA_HEIGHT}m")
    print(f"Camera tilt: {CAMERA_TILT_ANGLE}°")
    print(f"\n📝 Recording to: {output_file} @ {RECORDING_HZ}Hz")
    print("\nPress 'q' to quit, 'r' to reset score, 'd' to toggle debug")
    print("=" * 60 + "\n")

    f = open(output_file, 'w')
    f.write("timestamp,ball_x,ball_y,ball_z,cup_x,cup_y,cup_z\n")

    in_cup_start_time = None
    ball_is_in_cup = False
    score_time = None
    debug_mode = True
    last_record_time = 0
    record_count = 0

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

                color = (0, 255, 0) if not ball_is_in_cup else (0, 255, 255)
                cv2.drawContours(frame, [ball_contour], -1, color, 3)
                cv2.circle(frame, (ball_cx, ball_cy), 8, color, -1)

                if np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                    ball_x, ball_y, ball_z = camera_to_ground_frame_with_tilt(
                        X, Y, Z, CAMERA_HEIGHT, CAMERA_TILT_ANGLE
                    )
                    ball_position = (ball_x, ball_y, ball_z)
                    
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
                    
                    # Cup top = center + 7cm
                    cup_top_z = cup_z + 0.07
                    
                    txt_cup = f"Cup center z={cup_z:+.2f}m"
                    cv2.putText(frame, txt_cup, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                    
                    txt_top = f"Cup top z={cup_top_z:+.2f}m (+7cm)"
                    cv2.putText(frame, txt_top, (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    
                    # Draw yellow dot above cup center to show where we're checking
                    cv2.circle(frame, (cup_cx, cup_cy - 30), 8, (0, 255, 255), -1)

            # --- CHECK IF BALL IS ABOVE CUP TOP ---
            if ball_position is not None and cup_top_z is not None:
                ball_z = ball_position[2]
                
                if ball_z > cup_top_z:
                    height_diff = (ball_z - cup_top_z) * 100
                    cv2.putText(frame, "BALL ABOVE CUP!", (frame.shape[1]//2 - 150, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)
                    cv2.putText(frame, f"+{height_diff:.1f}cm", (frame.shape[1]//2 - 150, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            # --- RECORD DATA AT 40Hz ---
            if current_time - last_record_time >= RECORD_INTERVAL:
                ball_str = f"{ball_position[0]:.4f},{ball_position[1]:.4f},{ball_position[2]:.4f}" if ball_position else ",,"
                cup_str = f"{cup_position[0]:.4f},{cup_position[1]:.4f},{cup_position[2]:.4f}" if cup_position else ",,"
                
                f.write(f"{current_time:.4f},{ball_str},{cup_str}\n")
                f.flush()
                
                record_count += 1
                last_record_time = current_time

            # --- CHECK IF BALL IS IN CUP (3D) ---
            is_in_cup = False
            criteria = {}
            
            if ball_position is not None and cup_position is not None:
                is_in_cup, criteria = check_ball_in_cup(
                    ball_position, cup_position, 
                    X_TOLERANCE, Y_TOLERANCE, Z_TOLERANCE
                )
                
                if debug_mode:
                    y_pos = 120
                    cv2.putText(frame, "=== 3D CHECK ===", (10, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    y_pos += 22
                    
                    x_color = (0, 255, 0) if criteria['x_pass'] else (0, 0, 255)
                    cv2.putText(frame, f"X: {criteria['x_diff']*100:.1f}cm < {X_TOLERANCE*100:.0f}cm = {criteria['x_pass']}", 
                               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, x_color, 1)
                    y_pos += 20
                    
                    y_color = (0, 255, 0) if criteria['y_pass'] else (0, 0, 255)
                    cv2.putText(frame, f"Y: {criteria['y_diff']*100:.1f}cm < {Y_TOLERANCE*100:.0f}cm = {criteria['y_pass']}", 
                               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, y_color, 1)
                    y_pos += 20
                    
                    z_color = (0, 255, 0) if criteria['z_pass'] else (0, 0, 255)
                    cv2.putText(frame, f"Z: {criteria['z_diff']*100:.1f}cm < {Z_TOLERANCE*100:.0f}cm = {criteria['z_pass']}", 
                               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.5, z_color, 1)
                    y_pos += 20
                    
                    all_color = (0, 255, 0) if criteria['all_pass'] else (0, 0, 255)
                    cv2.putText(frame, f"ALL PASS: {criteria['all_pass']}", 
                               (10, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 0.6, all_color, 2)
                
                bx, by, bz = ball_position
                cx, cy, cz = cup_position
                distance_3d = np.sqrt((bx - cx)**2 + (by - cy)**2 + (bz - cz)**2)
                
                txt_dist = f"3D Distance: {distance_3d*100:.1f}cm"
                cv2.putText(frame, txt_dist, (10, 230),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # --- SCORING LOGIC ---
            if is_in_cup and not ball_is_in_cup:
                if in_cup_start_time is None:
                    in_cup_start_time = current_time
                    print("🎯 Ball meets ALL 3D criteria - timer started...")
                
                elapsed_time = current_time - in_cup_start_time
                
                if elapsed_time >= TIME_THRESHOLD:
                    ball_is_in_cup = True
                    score_time = current_time
                    print(f"\n🎉 GOAL! Ball is IN THE CUP!\n")
                else:
                    remaining = TIME_THRESHOLD - elapsed_time
                    txt_countdown = f"In cup... {remaining:.1f}s remaining"
                    cv2.putText(frame, txt_countdown, (10, 260),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            elif not is_in_cup and not ball_is_in_cup:
                if in_cup_start_time is not None:
                    print("⚠️ Ball no longer meets criteria - timer reset")
                in_cup_start_time = None

            if ball_is_in_cup:
                txt_scored = "*** IN THE CUP ***"
                text_size = cv2.getTextSize(txt_scored, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)[0]
                text_x = (frame.shape[1] - text_size[0]) // 2
                text_y = 290
                
                cv2.rectangle(frame, 
                             (text_x - 10, text_y - text_size[1] - 10),
                             (text_x + text_size[0] + 10, text_y + 10),
                             (0, 255, 0), -1)
                
                cv2.putText(frame, txt_scored, (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 0), 3)

            cv2.putText(frame, f"Recording: {record_count} samples @ {RECORDING_HZ}Hz", 
                       (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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
        elif key == ord('r'):
            ball_is_in_cup = False
            in_cup_start_time = None
            score_time = None
            print("\n🔄 Score reset!\n")
        elif key == ord('d'):
            debug_mode = not debug_mode
            print(f"\n{'🐛 Debug ON' if debug_mode else '▶️ Debug OFF'}\n")

    f.close()
    zed.close()
    cv2.destroyAllWindows()
    
    print(f"\n✅ Data saved: {output_file}")
    print(f"   Total samples: {record_count}")
    print(f"   Duration: {record_count / RECORDING_HZ:.1f}s")


if __name__ == "__main__":
    main()