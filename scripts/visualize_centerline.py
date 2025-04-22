#!/usr/bin/env python3
import cv2
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import yaml
import argparse
import os

def visualize_centerline(map_png, map_yaml, centerline_csv, output_path=None, show_plot=True):
    """
    Visualize centerline points on the map
    
    Args:
        map_png: Path to the map PNG file
        map_yaml: Path to the map YAML metadata file
        centerline_csv: Path to the CSV file with centerline points
        output_path: Path to save the visualization image
        show_plot: Whether to display the plot interactively
    """
    # Load map image
    map_img = cv2.imread(map_png, cv2.IMREAD_GRAYSCALE)
    if map_img is None:
        raise FileNotFoundError(f"Could not load map image: {map_png}")
    
    # Load map metadata
    with open(map_yaml, 'r') as f:
        map_data = yaml.safe_load(f)
        
    resolution = map_data.get('resolution', 0.05)  # m/pixel
    origin = map_data.get('origin', [0, 0, 0])  # [x, y, theta]
    
    # Load centerline data
    centerline_df = pd.read_csv(centerline_csv)
    
    # Check if the centerline is in world coordinates or pixel coordinates
    # We'll assume if x values are very small (< 1.0) it's in pixel coordinates 
    is_world_coords = centerline_df['x'].abs().max() > 1.0
    
    print(f"Detected coordinate system: {'world' if is_world_coords else 'pixel'}")
    
    # Convert world coordinates to pixel coordinates if needed
    if is_world_coords:
        # Function to convert world to pixel coordinates
        def world_to_pixel(world_x, world_y):
            pixel_x = int((world_x - origin[0]) / resolution)
            pixel_y = int(map_img.shape[0] - (world_y - origin[1]) / resolution)
            return pixel_x, pixel_y
        
        # Convert each point
        pixel_points = [world_to_pixel(x, y) for x, y in zip(centerline_df['x'], centerline_df['y'])]
        pixel_x = [p[0] for p in pixel_points]
        pixel_y = [p[1] for p in pixel_points]
    else:
        # Already in pixel coordinates
        pixel_x = centerline_df['x'].astype(int).tolist()
        pixel_y = centerline_df['y'].astype(int).tolist()
    
    # Create a figure for plotting
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Display the map
    ax.imshow(map_img, cmap='gray', origin='upper')
    
    # Plot the centerline
    ax.plot(pixel_x, pixel_y, 'r-', linewidth=2, label='Centerline')
    
    # Plot start and end points
    ax.plot(pixel_x[0], pixel_y[0], 'go', markersize=8, label='Start')
    ax.plot(pixel_x[-1], pixel_y[-1], 'bo', markersize=8, label='End')
    
    # Add indexing to some points for reference
    num_points = len(pixel_x)
    stride = max(1, num_points // 10)  # Show about 10 indices
    for i in range(0, num_points, stride):
        ax.annotate(str(i), (pixel_x[i], pixel_y[i]), 
                   color='blue', fontsize=8, ha='center', va='center',
                   bbox=dict(facecolor='white', alpha=0.7, pad=1))
    
    # Add labels and title
    ax.set_title('Track Map with Centerline')
    ax.set_xlabel('Pixel X')
    ax.set_ylabel('Pixel Y')
    ax.legend(loc='upper right')
    
    # Add grid for reference
    ax.grid(True, alpha=0.3)
    
    # Save the plot if output path is provided
    if output_path:
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Visualization saved to: {output_path}")
    
    # Show the plot
    if show_plot:
        plt.tight_layout()
        plt.show()
    
    plt.close()

def main():
    parser = argparse.ArgumentParser(description='Visualize centerline on map')
    parser.add_argument('map_png', help='Path to map PNG file')
    parser.add_argument('map_yaml', help='Path to map YAML file')
    parser.add_argument('centerline_csv', help='Path to centerline CSV file')
    parser.add_argument('--output', '-o', help='Path to save visualization')
    parser.add_argument('--no-display', action='store_true', help='Don\'t display the plot')
    
    args = parser.parse_args()
    
    visualize_centerline(
        args.map_png, 
        args.map_yaml, 
        args.centerline_csv,
        args.output,
        not args.no_display
    )

if __name__ == "__main__":
    main()