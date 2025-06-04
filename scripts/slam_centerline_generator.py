#!/usr/bin/env python3
"""
SLAM Map Centerline Extractor
Extracts racing line centerline from SLAM-generated PGM maps and YAML metadata
Outputs CSV file in format: # x_m, y_m, w_tr_right_m, w_tr_left_m
"""

import numpy as np
import cv2
import yaml
import argparse
import os
from scipy import ndimage
from scipy.interpolate import splprep, splev
from skimage.morphology import skeletonize, medial_axis
from skimage.measure import find_contours
import matplotlib.pyplot as plt


class SLAMCenterlineExtractor:
    def __init__(self, pgm_file, yaml_file, track_width=10.0):
        """
        Initialize the centerline extractor
        
        Args:
            pgm_file: Path to PGM map file
            yaml_file: Path to YAML metadata file
            track_width: Total track width in meters
        """
        self.pgm_file = pgm_file
        self.yaml_file = yaml_file
        self.track_width = track_width
        
        # Load map data
        self.map_data = self.load_pgm(pgm_file)
        self.map_info = self.load_yaml(yaml_file)
        
        # Extract parameters from YAML
        self.resolution = self.map_info.get('resolution', 0.05)
        self.origin = self.map_info.get('origin', [0.0, 0.0, 0.0])
        
        print(f"Map loaded: {self.map_data.shape}")
        print(f"Resolution: {self.resolution} m/pixel")
        print(f"Origin: {self.origin}")
    
    def load_pgm(self, filename):
        """Load PGM file and return as numpy array"""
        try:
            # Try using OpenCV first (handles most PGM files)
            img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                return img
        except:
            pass
        
        # Manual PGM parsing for P5 format
        with open(filename, 'rb') as f:
            # Read header
            magic = f.readline().decode().strip()
            if magic != 'P5':
                raise ValueError("Only P5 PGM format supported")
            
            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()
            
            # Read dimensions
            width, height = map(int, line.split())
            
            # Skip comments
            line = f.readline().decode().strip()
            while line.startswith('#'):
                line = f.readline().decode().strip()
            
            # Read max value
            maxval = int(line)
            
            # Read pixel data
            pixels = np.frombuffer(f.read(), dtype=np.uint8)
            
            # Reshape to image
            img = pixels.reshape((height, width))
            return img
    
    def load_yaml(self, filename):
        """Load YAML metadata file"""
        with open(filename, 'r') as f:
            return yaml.safe_load(f)
    
    def preprocess_map(self, threshold=127, kernel_size=3):
        """
        Preprocess the occupancy grid map
        
        Args:
            threshold: Threshold for free space (0-255)
            kernel_size: Morphological operations kernel size
        """
        # Create binary map: 1 = free space, 0 = obstacle/unknown
        self.binary_map = (self.map_data > threshold).astype(np.uint8)
        
        # Clean up the map with morphological operations
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        
        # Remove small holes in free space
        self.binary_map = cv2.morphologyEx(self.binary_map, cv2.MORPH_CLOSE, kernel)
        
        # Remove small obstacles
        self.binary_map = cv2.morphologyEx(self.binary_map, cv2.MORPH_OPEN, kernel)
        
        print(f"Preprocessed map: {np.sum(self.binary_map)} free pixels")
    
    def extract_centerline_skeleton(self):
        """Extract centerline using skeletonization method"""
        # Get the skeleton of the free space
        skeleton = skeletonize(self.binary_map)
        
        # Find skeleton points
        y_coords, x_coords = np.where(skeleton)
        
        if len(x_coords) == 0:
            raise ValueError("No centerline found. Try adjusting threshold or preprocessing parameters.")
        
        return list(zip(x_coords, y_coords))
    
    def extract_centerline_medial_axis(self):
        """Extract centerline using medial axis method"""
        # Compute medial axis
        skel, distance = medial_axis(self.binary_map, return_distance=True)
        
        # Find medial axis points
        y_coords, x_coords = np.where(skel)
        
        if len(x_coords) == 0:
            raise ValueError("No centerline found. Try adjusting threshold or preprocessing parameters.")
        
        # Sort by distance from medial axis (prioritize center of track)
        distances = distance[y_coords, x_coords]
        sorted_indices = np.argsort(-distances)  # Sort by descending distance
        
        # Take points with sufficient distance from boundaries
        min_distance = np.max(distances) * 0.3  # At least 30% of max distance
        valid_indices = distances >= min_distance
        
        x_coords = x_coords[valid_indices]
        y_coords = y_coords[valid_indices]
        
        return list(zip(x_coords, y_coords))
    
    def extract_centerline_contour(self):
        """Extract centerline by finding track boundaries and computing midpoints"""
        # Find contours of the free space
        contours = find_contours(self.binary_map, level=0.5)
        
        if not contours:
            raise ValueError("No track boundaries found")
        
        # Find the largest contour (main track boundary)
        largest_contour = max(contours, key=len)
        
        # For oval/closed tracks, we need to find inner and outer boundaries
        # This is a simplified approach - for complex tracks, more sophisticated methods needed
        
        centerline_points = []
        height, width = self.binary_map.shape
        
        # Sample along the track
        for y in range(0, height, 5):  # Sample every 5 pixels
            # Find intersections with track boundaries at this y-level
            row_pixels = np.where(self.binary_map[y, :] == 1)[0]
            
            if len(row_pixels) > 0:
                # Find gaps (track sections)
                diff = np.diff(row_pixels)
                gap_starts = np.where(diff > 1)[0] + 1
                
                # Get continuous segments
                segments = []
                start = 0
                for gap_start in gap_starts:
                    if gap_start > start:
                        segments.append((row_pixels[start], row_pixels[gap_start - 1]))
                    start = gap_start
                
                # Add final segment
                if start < len(row_pixels):
                    segments.append((row_pixels[start], row_pixels[-1]))
                
                # Find the widest segment (main track)
                if segments:
                    widest_segment = max(segments, key=lambda s: s[1] - s[0])
                    center_x = (widest_segment[0] + widest_segment[1]) / 2
                    centerline_points.append((center_x, y))
        
        return centerline_points
    
    def order_centerline_points(self, points):
        """Order centerline points to form a continuous path"""
        if len(points) < 2:
            return points
        
        # Convert to numpy array
        points = np.array(points)
        
        # Start from bottom-left point
        start_idx = np.argmin(points[:, 1] + points[:, 0])  # Bottom-left corner
        
        ordered_points = [points[start_idx]]
        remaining_points = np.delete(points, start_idx, axis=0)
        
        # Greedy nearest neighbor ordering
        while len(remaining_points) > 0:
            current_point = ordered_points[-1]
            distances = np.sum((remaining_points - current_point) ** 2, axis=1)
            nearest_idx = np.argmin(distances)
            
            ordered_points.append(remaining_points[nearest_idx])
            remaining_points = np.delete(remaining_points, nearest_idx, axis=0)
        
        return ordered_points
    
    def smooth_centerline(self, points, smoothing_factor=0.1, num_points=None):
        """Smooth centerline using spline interpolation"""
        if len(points) < 4:
            return points
        
        points = np.array(points)
        
        # Fit spline
        try:
            tck, u = splprep([points[:, 0], points[:, 1]], s=smoothing_factor * len(points))
            
            # Generate smooth points
            if num_points is None:
                num_points = len(points)
            
            u_new = np.linspace(0, 1, num_points)
            smooth_points = splev(u_new, tck)
            
            return list(zip(smooth_points[0], smooth_points[1]))
        except:
            print("Warning: Spline smoothing failed, returning original points")
            return points.tolist()
    
    def pixel_to_world(self, pixel_points):
        """Convert pixel coordinates to world coordinates"""
        world_points = []
        
        for px, py in pixel_points:
            # Convert from image coordinates to world coordinates
            world_x = self.origin[0] + px * self.resolution
            world_y = self.origin[1] + (self.map_data.shape[0] - py) * self.resolution
            
            world_points.append({
                'x_m': world_x,
                'y_m': world_y,
                'w_tr_right_m': self.track_width / 2,
                'w_tr_left_m': self.track_width / 2
            })
        
        return world_points
    
    def extract_centerline(self, method='medial_axis', **kwargs):
        """
        Extract centerline using specified method
        
        Args:
            method: 'skeleton', 'medial_axis', or 'contour'
            **kwargs: Additional parameters for preprocessing and smoothing
        """
        # Preprocess map
        threshold = kwargs.get('threshold', 127)
        kernel_size = kwargs.get('kernel_size', 3)
        self.preprocess_map(threshold, kernel_size)
        
        # Extract centerline points
        if method == 'skeleton':
            pixel_points = self.extract_centerline_skeleton()
        elif method == 'medial_axis':
            pixel_points = self.extract_centerline_medial_axis()
        elif method == 'contour':
            pixel_points = self.extract_centerline_contour()
        else:
            raise ValueError(f"Unknown method: {method}")
        
        print(f"Extracted {len(pixel_points)} centerline points")
        
        # Order points
        pixel_points = self.order_centerline_points(pixel_points)
        
        # Smooth centerline
        smoothing_factor = kwargs.get('smoothing_factor', 0.1)
        num_output_points = kwargs.get('num_points', len(pixel_points))
        pixel_points = self.smooth_centerline(pixel_points, smoothing_factor, num_output_points)
        
        # Convert to world coordinates
        world_points = self.pixel_to_world(pixel_points)
        
        return world_points, pixel_points
    
    def save_csv(self, world_points, output_file):
        """Save centerline to CSV file"""
        with open(output_file, 'w') as f:
            f.write("# x_m, y_m, w_tr_right_m, w_tr_left_m\n")
            for point in world_points:
                f.write(f"{point['x_m']:.6f}, {point['y_m']:.6f}, "
                       f"{point['w_tr_right_m']:.6f}, {point['w_tr_left_m']:.6f}\n")
        
        print(f"Centerline saved to {output_file}")
    
    def visualize(self, world_points, pixel_points, save_plot=None):
        """Visualize the results"""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Plot original map with pixel centerline
        ax1.imshow(self.map_data, cmap='gray', origin='lower')
        if pixel_points:
            pixel_points = np.array(pixel_points)
            ax1.plot(pixel_points[:, 0], pixel_points[:, 1], 'r-', linewidth=2, label='Centerline')
            ax1.plot(pixel_points[0, 0], pixel_points[0, 1], 'go', markersize=8, label='Start')
        ax1.set_title('Original Map with Centerline (Pixels)')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Plot world coordinates
        if world_points:
            x_coords = [p['x_m'] for p in world_points]
            y_coords = [p['y_m'] for p in world_points]
            
            ax2.plot(x_coords, y_coords, 'r-', linewidth=2, label='Centerline')
            ax2.plot(x_coords[0], y_coords[0], 'go', markersize=8, label='Start')
            ax2.set_xlabel('X (meters)')
            ax2.set_ylabel('Y (meters)')
            ax2.set_title('Centerline in World Coordinates')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
            ax2.axis('equal')
        
        plt.tight_layout()
        
        if save_plot:
            plt.savefig(save_plot, dpi=300, bbox_inches='tight')
            print(f"Plot saved to {save_plot}")
        else:
            plt.show()


def main():
    parser = argparse.ArgumentParser(description='Extract centerline from SLAM map')
    parser.add_argument('pgm_file', help='Path to PGM map file')
    parser.add_argument('yaml_file', help='Path to YAML metadata file')
    parser.add_argument('-o', '--output', default='centerline.csv', help='Output CSV file')
    parser.add_argument('-w', '--track_width', type=float, default=10.0, help='Track width in meters')
    parser.add_argument('-m', '--method', choices=['skeleton', 'medial_axis', 'contour'], 
                       default='medial_axis', help='Centerline extraction method')
    parser.add_argument('-t', '--threshold', type=int, default=127, help='Free space threshold (0-255)')
    parser.add_argument('-s', '--smoothing', type=float, default=0.1, help='Smoothing factor')
    parser.add_argument('-n', '--num_points', type=int, help='Number of output points')
    parser.add_argument('--visualize', action='store_true', help='Show visualization')
    parser.add_argument('--save_plot', help='Save plot to file')
    
    args = parser.parse_args()
    
    # Check if files exist
    if not os.path.exists(args.pgm_file):
        print(f"Error: PGM file not found: {args.pgm_file}")
        return
    
    if not os.path.exists(args.yaml_file):
        print(f"Error: YAML file not found: {args.yaml_file}")
        return
    
    try:
        # Create extractor
        extractor = SLAMCenterlineExtractor(args.pgm_file, args.yaml_file, args.track_width)
        
        # Extract centerline
        world_points, pixel_points = extractor.extract_centerline(
            method=args.method,
            threshold=args.threshold,
            smoothing_factor=args.smoothing,
            num_points=args.num_points
        )
        
        # Save to CSV
        extractor.save_csv(world_points, args.output)
        
        # Visualize if requested
        if args.visualize or args.save_plot:
            extractor.visualize(world_points, pixel_points, args.save_plot)
        
        print(f"\nSuccess! Extracted {len(world_points)} points")
        print(f"Centerline saved to: {args.output}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()