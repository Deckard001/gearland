#!/usr/bin/env python3
import numpy as np
import cv2
import os
import argparse
import yaml
import pickle
from glob import glob

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Calibrate camera using a video of a chessboard or a sequence of images.')
    parser.add_argument('input', nargs="?", help='input video file or glob mask')
    parser.add_argument('out', nargs="?", help='output calibration yaml file')
    parser.add_argument('--debug_dir', nargs="?", help='path to directory where images with detected chessboard will be written',
                        default='./pictures')
    parser.add_argument('--output_dir', nargs="?", help='path to directory where calibration files will be saved.', default='./calibrationFiles')
    parser.add_argument('-c', '--corners', nargs="?", help='output corners file', default=None)
    parser.add_argument('-fs', '--framestep', nargs="?", help='use every nth frame in the video', default=20, type=int)
    parser.add_argument('--height', nargs="?", help='Height in pixels of the image', default=480, type=int)
    parser.add_argument('--width', nargs="?", help='Width in pixels of the image', default=640, type=int)
    parser.add_argument('--mm', nargs="?", help='Size in mm of each square.', default=22, type=int)
    args = parser.parse_args()

    # Handle input source
    if args.input is None:
        source = cv2.VideoCapture(0)
        source.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        source.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    elif '*' in args.input:
        source = sorted(glob(args.input))
    else:
        source = cv2.VideoCapture(args.input)

    # Create directories if needed
    os.makedirs(args.debug_dir, exist_ok=True)
    os.makedirs(args.output_dir, exist_ok=True)

    # Setup calibration pattern (scale by actual size)
    pattern_size = (9, 6)
    square_size = args.mm / 1000.0  # Convert mm to meters
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    image_count = 0
    image_goal = 30

    i = -1
    while True:
        i += 1
        if isinstance(source, list):  # Image sequence
            if i >= len(source):
                break
            img = cv2.imread(source[i])
            if img is None:
                print(f"Warning: Could not read image {source[i]}")
                continue
        else:  # Video capture
            ret, img = source.read()
            if not ret:
                break
            if i % args.framestep != 0:
                continue

        # Display and process frame
        cv2.imshow('Calibration', img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape[:2]
        
        # Check for quit key
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        print(f'Searching for chessboard in frame {i}...')
        found, corners = cv2.findChessboardCorners(gray, pattern_size, flags=cv2.CALIB_CB_FILTER_QUADS)
        
        if found:
            # Refine corner locations
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
            image_count += 1
            
            # Draw and save debug image
            debug_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            cv2.drawChessboardCorners(debug_img, pattern_size, corners, found)
            cv2.imwrite(os.path.join(args.debug_dir, f'frame_{i:04d}.png'), debug_img)
            
            # Store points
            img_points.append(corners.reshape(-1, 2))
            obj_points.append(pattern_points)
            print('Found!')

            if image_count >= image_goal:
                print(f"Collected {image_goal} calibration frames")
                break
        else:
            print('Not found')

    # Release resources
    if isinstance(source, cv2.VideoCapture):
        source.release()
    cv2.destroyAllWindows()

    # Perform calibration (OpenCV 4.5.1 compatible)
    if len(obj_points) > 0:
        print(f'\nCalibrating with {len(obj_points)} images...')
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, 
            img_points, 
            (w, h), 
            None, 
            None
        )
        
        print(f"RMS Error: {ret:.4f}")
        print(f"Camera Matrix:\n{mtx}")
        print(f"Distortion Coefficients: {dist.ravel()}")

        # Save results
        if args.corners:
            with open(args.corners, 'wb') as f:
                pickle.dump((img_points, obj_points, (w, h)), f)

        # Save calibration files
        np.savetxt(os.path.join(args.output_dir, "cameraMatrix.txt"), mtx)
        np.savetxt(os.path.join(args.output_dir, "cameraDistortion.txt"), dist)

        # Save YAML output if specified
        if args.out:
            calibration = {
                'rms': float(ret),
                'camera_matrix': mtx.tolist(),
                'distortion_coefficients': dist.tolist(),
                'image_width': w,
                'image_height': h
            }
            with open(args.out, 'w') as f:
                yaml.dump(calibration, f)
    else:
        print("Error: No calibration points found!")
