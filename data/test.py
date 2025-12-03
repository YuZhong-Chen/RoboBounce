import cv2
import os
import numpy as np

# ================= Configuration =================
DATA_ROOT = "/home/user/RoboBounce/data/20251203_174215"
FPS = 60
# =================================================


def main():
    rgb_dir = os.path.join(DATA_ROOT, "rgb")
    depth_dir = os.path.join(DATA_ROOT, "depth")
    if not os.path.exists(rgb_dir) or not os.path.exists(depth_dir):
        print(f"Error: Directories not found. Check path: {DATA_ROOT}")
        return

    # Get file lists and sort them to ensure correct sequence
    rgb_files = sorted(os.listdir(rgb_dir))
    depth_files = sorted(os.listdir(depth_dir))
    print(f"Found {len(rgb_files)} RGB images")
    print(f"Found {len(depth_files)} Depth images")

    # Mouse callback variables
    mouse_x, mouse_y = 0, 0

    def mouse_callback(event, x, y, flags, param):
        nonlocal mouse_x, mouse_y
        if event == cv2.EVENT_MOUSEMOVE:
            mouse_x, mouse_y = x, y

    # Setup window and callback
    cv2.namedWindow("Player")
    cv2.setMouseCallback("Player", mouse_callback)

    for i, (rgb_f, depth_f) in enumerate(zip(rgb_files, depth_files)):
        rgb_path = os.path.join(rgb_dir, rgb_f)
        depth_path = os.path.join(depth_dir, depth_f)

        frame_rgb = cv2.imread(rgb_path)
        frame_depth_raw = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if frame_rgb is None or frame_depth_raw is None:
            print(f"Error reading frame {i}, skipping...")
            continue

        dist_m = 0.0
        h, w = frame_depth_raw.shape
        target_x = mouse_x % w
        target_y = mouse_y
        if 0 <= target_y < h and 0 <= target_x < w:
            dist_mm = frame_depth_raw[target_y, target_x]
            dist_m = dist_mm / 1000.0

        # --- Visualization ---
        # Normalize depth for display (0m to 2.5m mapped to 0-255)
        # This is strictly for human viewing; do not use this for analysis.
        depth_vis = frame_depth_raw.astype(np.float32)
        depth_vis = np.clip(depth_vis, 0, 2500) / 2500.0 * 255.0
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        info_text = f"Frame: {i}"
        cv2.putText(frame_rgb, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.circle(depth_vis, (target_x, target_y), 5, (255, 255, 255), 1)
        cv2.putText(depth_vis, f"{dist_m:.2f}m", (target_x + 10, target_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Combine RGB and Depth side-by-side
        combined = np.hstack((frame_rgb, depth_vis))
        cv2.imshow("Player", combined)

        # Handle user input
        key = cv2.waitKey(int(1000 / FPS))
        if key == ord("q"):
            break
        elif key == ord(" "):
            cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
