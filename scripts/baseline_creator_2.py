#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import argparse
import os

class OfflineCenterlineDetector:
    def __init__(self, map_png, map_yaml):
        """
        Initialize the detector with map PNG and YAML files
        """
        # Load map image
        self.map_img = cv2.imread(map_png, cv2.IMREAD_GRAYSCALE)
        if self.map_img is None:
            raise FileNotFoundError(f"Could not load map image: {map_png}")

        # Load map metadata
        with open(map_yaml, 'r') as f:
            self.map_data = yaml.safe_load(f)
            
        self.resolution = self.map_data.get('resolution', 0.05)  # m/pixel
        self.origin = self.map_data.get('origin', [0, 0, 0])  # [x, y, theta]
        
        print(f"Loaded map: {map_png}, resolution: {self.resolution} m/pixel")
        
    def detect_centerline(self):
        """
        Detect the centerline using improved CV approach
        """
        # Create a copy for visualization
        color_img = cv2.cvtColor(self.map_img, cv2.COLOR_GRAY2BGR)
        
        # Step 1: Convert to binary (assuming black is obstacle, white is free space)
        # Invert if needed based on your map convention
        _, binary = cv2.threshold(self.map_img, 220, 255, cv2.THRESH_BINARY)
        
        #cv2.imshow("binary", binary)

        # Step 2: Apply morphological operations to clean the image
        # kernel = np.ones((3, 3), np.uint8)
        # cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        # #cv2.imshow("cleaned", cleaned)
        # cleaned_2 = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        # #cv2.imshow("cleaned_2", cleaned_2)
        
        # Step 3: Distance transform to get distance to nearest obstacle
        dist_transform = cv2.distanceTransform(binary, cv2.DIST_L2, 5)
        
        # Step 4: Normalize the distance transform for visualization
        cv2.normalize(dist_transform, dist_transform, 0, 1.0, cv2.NORM_MINMAX)
        dist_transform_vis = (dist_transform * 255).astype(np.uint8)
        # cv2.imshow("d", dist_transform_vis)
        
        # consider erode and redo dist
        # Apply erosion to thin the ridge
        if False:
            erosion_kernel = np.ones((3, 3), np.uint8)  # Adjust kernel size as needed
            eroded_ridge = cv2.erode(dist_transform_vis, erosion_kernel, iterations=1)  # Adjust iterations as needed
            # cv2.imshow("e", eroded_ridge)
        else:
            eroded_ridge = dist_transform_vis

        # Step 5: Threshold to get the ridge (centerline candidates)
        _, thresh = cv2.threshold(eroded_ridge, 60, 255, cv2.THRESH_BINARY)
        # cv2.imshow("t", thresh)

        # Step 3: Distance transform to get distance to nearest obstacle
        dist_transform = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        
        # Step 4: Normalize the distance transform for visualization
        cv2.normalize(dist_transform, dist_transform, 0, 1.0, cv2.NORM_MINMAX)
        dist_transform_vis = (dist_transform * 255).astype(np.uint8)
        # cv2.imshow("d2", dist_transform_vis)

        _, thresh = cv2.threshold(dist_transform_vis, 30, 255, cv2.THRESH_BINARY)
        # cv2.imshow("t2", thresh)

        # Step 6: Skeletonize to get a thin centerline
        if False:
            skeleton = self.skeletonize(thresh)
            #cv2.imshow("s", skeleton)
        else:
            skeleton = thresh

        # After skeletonization
        # Apply dilation to connect nearby skeleton segments
        # connect_kernel = np.ones((3, 3), np.uint8)
        # connected_skeleton = cv2.dilate(skeleton, connect_kernel, iterations=1)
        # connected_skeleton = self.skeletonize(connected_skeleton)  # Re-skeletonize to make it thin again

        # Step 7: Find contours to extract centerline points
        contours, _ = cv2.findContours(skeleton, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        
        # Step 8: Sort contours by length and take the longest (should be the track)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        centerline_points = []

        #outer = contours[1]
        #plt.imshow(skeleton, cmap='gray')
        #plt.plot([p[0][0] for p in outer], [p[0][1] for p in outer], 'r--')
        #plt.plot([p[0][0] for p in outer], [p[0][1] for p in outer], 'g--')
        #plt.title("outer")
        #plt.show()

        if contours and len(contours) > 0:
            # Get the largest contour (assuming it's the track)
            main_contour = contours[1]

            # Simplify the contour to reduce number of points
            if False:
                epsilon = 0.01 * cv2.arcLength(main_contour, True)
                approx_contour = cv2.approxPolyDP(main_contour, epsilon, False)
            else:
                approx_contour = main_contour

            # Extract the points
            for point in approx_contour:
                x, y = point[0]
                centerline_points.append((x, y))
                cv2.circle(color_img, (x, y), 2, (0, 0, 255), -1)
            
            # Draw the centerline
            cv2.drawContours(color_img, [approx_contour], -1, (0, 255, 0), 1)
        
        # Step 9: Smooth the centerline
        if centerline_points and len(centerline_points) > 3:
            centerline_points = self.smooth_path(centerline_points)
            
            # Draw smoothed centerline
            for i in range(len(centerline_points) - 1):
                p1 = tuple(map(int, centerline_points[i]))
                p2 = tuple(map(int, centerline_points[i + 1]))
                cv2.line(color_img, p1, p2, (0, 255, 255), 2)
        
        return color_img, centerline_points
    
    def skeletonize(self, img):
        """
        Skeletonize a binary image using morphological operations
        """
        size = np.size(img)
        skel = np.zeros(img.shape, np.uint8)
        element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
        done = False
        
        while not done:
            eroded = cv2.erode(img, element)
            temp = cv2.dilate(eroded, element)
            temp = cv2.subtract(img, temp)
            skel = cv2.bitwise_or(skel, temp)
            img = eroded.copy()
            
            zeros = size - cv2.countNonZero(img)
            if zeros == size:
                done = True
        
        return skel
    
    def smooth_path(self, points, smoothing=0.1):
        """
        Smooth the path using spline interpolation
        """
        if len(points) < 4:
            return points
            
        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        
        # Fit a spline to the points
        try:
            tck, u = splprep([x, y], s=smoothing, per=False)
            
            # Create more points for a smoother curve
            u_new = np.linspace(0, 1, num=len(points) * 2)
            x_new, y_new = splev(u_new, tck)
            
            # Return as a list of coordinate tuples
            return list(zip(x_new, y_new))
        except Exception as e:
            print(f"Spline smoothing failed: {e}")
            return points
    
    def pixel_to_world(self, pixel_point):
        """
        Convert pixel coordinates to world coordinates using the map metadata
        """
        x, y = pixel_point
        world_x = x * self.resolution + self.origin[0]
        world_y = (self.map_img.shape[0] - y) * self.resolution + self.origin[1]  # Adjust for image coordinate system
        return (world_x, world_y)
    
    def save_centerline(self, centerline_points, output_path):
        """
        Save the centerline as a CSV and display the results
        """
        # Convert to world coordinates
        world_points = [self.pixel_to_world(p) for p in centerline_points]
        
        # Save to CSV
        with open(output_path, 'w') as f:
            f.write("x,y\n")  # Header
            for x, y in world_points:
                f.write(f"{x:.2f},{y:.2f}\n")
        
        print(f"Saved centerline to {output_path}")
        
        # Also create a visualization and save it
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.imshow(self.map_img, cmap='gray')
        
        # Plot centerline points
        x_coords = [p[0] for p in centerline_points]
        y_coords = [p[1] for p in centerline_points]
        ax.plot(x_coords, y_coords, 'r-', linewidth=2)
        
        # Mark start and end
        if centerline_points:
            ax.plot(centerline_points[0][0], centerline_points[0][1], 'go', markersize=10)
            ax.plot(centerline_points[-1][0], centerline_points[-1][1], 'bo', markersize=10)
        
        # Save the plot
        viz_path = os.path.splitext(output_path)[0] + "_viz.png"
        plt.savefig(viz_path)
        print(f"Saved visualization to {viz_path}")
        
        return world_points

def main():
    parser = argparse.ArgumentParser(description='Generate track centerline from map')
    parser.add_argument('map_png', help='Path to map PNG file')
    parser.add_argument('map_yaml', help='Path to map YAML file')
    parser.add_argument('--output', '-o', default='centerline.csv', help='Output CSV file')
    parser.add_argument('--visualize', '-v', action='store_true', help='Show visualization')
    
    args = parser.parse_args()
    
    detector = OfflineCenterlineDetector(args.map_png, args.map_yaml)
    result_img, centerline_points = detector.detect_centerline()
    
    world_points = detector.save_centerline(centerline_points, args.output)
    
    if args.visualize:
        cv2.imshow("Map", detector.map_img)
        cv2.imshow("Centerline Result", result_img)
        print("Press any key to close the windows...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    return 0

if __name__ == "__main__":
    main()