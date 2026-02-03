import time
import numpy as np
import cv2
import pyzed.sl as sl

# ============================
# COLOR RANGES (tweakable)
# ============================
LOWER_ORANGE = np.array([10, 150, 100], dtype=np.uint8)
UPPER_ORANGE = np.array([20, 255, 255], dtype=np.uint8)

LOWER_BLUE = np.array([90, 80, 40], dtype=np.uint8)
UPPER_BLUE = np.array([130, 255, 255], dtype=np.uint8)

MIN_BALL_AREA = 300
MIN_CUP_AREA = 500

# ============================
# CAMERA CONFIGURATION
# ============================
CAMERA_HEIGHT_FALLBACK = 1.21  # Used if ground detection fails
CAMERA_TILT_ANGLE = -23.0

# ============================
# DETECTION SETTINGS
# ============================
TIME_THRESHOLD = 2.0

X_TOLERANCE = 0.08
Y_TOLERANCE = 0.10
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


def detect_ground_plane(zed, point_cloud, frame_shape):
    """
    Detect ground plane by sampling bottom portion of image.
    Returns camera height above ground.
    """
    h, w = frame_shape[:2]
    
    # Sample points from bottom 30% of image (likely to be ground)
    sample_points = []
    sample_count = 50
    
    for _ in range(sample_count):
        x = np.random.randint(w // 4, 3 * w // 4)  # Middle half horizontally
        y = np.random.randint(int(h * 0.7), h - 10)  # Bottom 30%
        
        err, point = point_cloud.get_value(x, y)
        if err == sl.ERROR_CODE.SUCCESS:
            X, Y_cam, Z = point[0], point[1], point[2]
            if np.isfinite(Y_cam) and np.isfinite(Z) and Z > 0:
                sample_points.append(Y_cam)
    
    if len(sample_points) > 20:
        # Ground should be at consistent negative Y value (below camera)
        # Take median of most negative values (actual ground)
        sample_points.sort()
        ground_y = np.median(sample_points[:len(sample_points)//3])
        camera_height = abs(ground_y)
        
        return camera_height
    
    return None


def calibrate_ground_height(zed, point_cloud, frame_shape, attempts=30):
    """
    Try to detect ground for several frames.
    Returns best estimate of camera height.
    """
    print("\n🔍 Detecting ground plane...")
    print("   Point camera at the ground for calibration...")
    
    height_samples = []
    
    for i in range(attempts):
        height = detect_ground_plane(zed, point_cloud, frame_shape)
        if height is not None:
            height_samples.append(height)
            print(f"   Sample {i+1}/{attempts}: {height:.3f}m")
    
    if len(height_samples) > 10:
        # Use median to reject outliers
        camera_height = np.median(height_samples)
        std_dev = np.std(height_samples)
        print(f"\n✅ Ground detected! Camera height: {camera_height:.3f}m (±{std_dev:.3f}m)")
        return camera_height
    else:
        print(f"\n⚠️ Ground detection failed. Using fallback: {CAMERA_HEIGHT_FALLBACK}m")
        return CAMERA_HEIGHT_FALLBACK


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

    # Initial grab to get frame shape
    if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(left, sl.VIEW.LEFT)
        img_bgra = left.get_data()
        frame = cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
        frame_shape = frame.shape
        zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
    else:
        raise RuntimeError("Failed to grab initial frame")

    # Auto-calibrate ground height
    camera_height = calibrate_ground_height(zed, point_cloud, frame_shape)

    cv2.namedWindow("Ball and Cup Tracking", cv2.WINDOW_NORMAL)

    print("\n🎯 Ball in Cup Detection - Auto Ground Calibrated")
    print("=" * 60)
    print(f"Camera height: {camera_height:.3f}m (auto-detected)")
    print(f"Camera tilt: {CAMERA_TILT_ANGLE}°")
    print(f"\nDetection criteria (ALL must pass for {TIME_THRESHOLD}s):")
    print(f"  X (left/right):   < {X_TOLERANCE*100:.0f}cm")
    print(f"  Y (forward/back): < {Y_TOLERANCE*100:.0f}cm")
    print(f"  Z (height):       < {Z_TOLERANCE*100:.0f}cm")
    print("\nControls:")
    print("  'q' - quit")
    print("  'r' - reset score")
    print("  'd' - toggle debug")
    print("  'g' - recalibrate ground")
    print("=" * 60 + "\n")

    in_cup_start_time = None
    ball_is_in_cup = False
    score_time = None
    debug_mode = True

    while True:
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
                ball_d = depth_map[ball_cy, ball_cx]

                err, point = point_cloud.get_value(ball_cx, ball_cy)
                X, Y, Z = point[0], point[1], point[2]

                color = (0, 255, 0) if not ball_is_in_cup else (0, 255, 255)
                cv2.drawContours(frame, [ball_contour], -1, color, 3)
                cv2.circle(frame, (ball_cx, ball_cy), 8, color, -1)

                if np.isfinite(ball_d) and ball_d > 0 and np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                    ball_x, ball_y, ball_z = camera_to_ground_frame_with_tilt(
                        X, Y, Z, camera_height, CAMERA_TILT_ANGLE
                    )
                    ball_position = (ball_x, ball_y, ball_z)
                    
                    txt_ball = f"Ball: x={ball_x:+.2f}m y={ball_y:+.2f}m z={ball_z:+.2f}m"
                    cv2.putText(frame, txt_ball, (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Process CUP
            cup_position = None
            if cup_result is not None:
                cup_cx, cup_cy, cup_area, cup_contour = cup_result
                cup_d = depth_map[cup_cy, cup_cx]

                err, point = point_cloud.get_value(cup_cx, cup_cy)
                X, Y, Z = point[0], point[1], point[2]

                cv2.drawContours(frame, [cup_contour], -1, (255, 0, 255), 2)
                cv2.circle(frame, (cup_cx, cup_cy), 6, (255, 0, 255), -1)

                if np.isfinite(cup_d) and cup_d > 0 and np.isfinite(X) and np.isfinite(Y) and np.isfinite(Z):
                    cup_x, cup_y, cup_z = camera_to_ground_frame_with_tilt(
                        X, Y, Z, camera_height, CAMERA_TILT_ANGLE
                    )
                    cup_position = (cup_x, cup_y, cup_z)
                    
                    txt_cup = f"Cup: x={cup_x:+.2f}m y={cup_y:+.2f}m z={cup_z:+.2f}m"
                    cv2.putText(frame, txt_cup, (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            # --- CHECK IF BALL IS IN CUP ---
            is_in_cup = False
            criteria = {}
            
            if ball_position is not None and cup_position is not None:
                is_in_cup, criteria = check_ball_in_cup(
                    ball_position, cup_position, 
                    X_TOLERANCE, Y_TOLERANCE, Z_TOLERANCE
                )
                
                # Debug display
                if debug_mode:
                    y_pos = 90
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
                    
                    # Camera height
                    cv2.putText(frame, f"Cam height: {camera_height:.3f}m", 
                               (10, y_pos + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            # --- SCORING LOGIC ---
            current_time = time.time()
            
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
                    cv2.putText(frame, txt_countdown, (10, 230),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            elif not is_in_cup and not ball_is_in_cup:
                if in_cup_start_time is not None:
                    print("⚠️ Ball no longer meets criteria - timer reset")
                in_cup_start_time = None

            # Display score
            if ball_is_in_cup:
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
        elif key == ord('g'):
            print("\n🔄 Recalibrating ground...")
            camera_height = calibrate_ground_height(zed, point_cloud, frame.shape)

    zed.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()