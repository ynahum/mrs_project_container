import cv2
import numpy as np
import yaml
import matplotlib.pyplot as plt
import os

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        map_metadata = yaml.safe_load(f)
    
    yaml_dir = os.path.dirname(yaml_path)
    image_path = os.path.join(yaml_dir, map_metadata['image'])

    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    resolution = map_metadata['resolution']
    origin = map_metadata['origin']
    return img, resolution, origin

def get_track_centerline(img):
    # Invert colors (assuming track is white on black background)
    _, thresh = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
    #plt.imshow(thresh, cmap='gray')
    #plt.title("thresh")
    #plt.show()
    edges = cv2.Canny(thresh, 50, 150)
    
    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) < 2:
        raise ValueError("Expected to find inner and outer track boundaries!")

    # Sort by contour size
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    outer = np.concatenate((contours[1],contours[2]))[0::4]
    inner = contours[0]

    #print(outer)
    #plt.imshow(edges, cmap='gray')
    #plt.plot([p[0][0] for p in inner], [p[0][1] for p in inner], 'r--')
    #plt.plot([p[0][0] for p in outer], [p[0][1] for p in outer], 'g--')
    #plt.title("edges")
    #plt.show()

    print(f"len(inner) = {len(inner)}")
    print(f"len(outer) = {len(outer)}")
    centerline = []
    for i in range(min(len(inner), len(outer))):
        p1 = inner[i % len(inner)][0]
        p2 = outer[i % len(outer)][0]
        mid = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
        centerline.append(mid)

    #print(centerline)
    return centerline

def pixels_to_map_coords(img, centerline_px, resolution, origin):
    origin_x, origin_y = origin[0], origin[1]
    print(img.shape)
    print(origin)

    map_coords = []
    for px, py in centerline_px:
        mx = px * resolution + origin_x
        my = (img.shape[0] - py) * resolution + origin_y  # y axis is flipped
        map_coords.append((mx, my))
    return map_coords

# ---- MAIN ----
yaml_path = "/home/yair/dev/ad_api_container/mrs_project_container/my_car/maps/practice_2_w_as_o.yaml"
img, resolution, origin = load_map(yaml_path)
centerline_px = get_track_centerline(img)
centerline_map = pixels_to_map_coords(img, centerline_px, resolution, origin)

# Plot for visualization
#x, y = zip(*centerline_map)
plt.imshow(img, cmap='gray')
plt.plot([p[0] for p in centerline_px], [p[1] for p in centerline_px], 'r--')
origin_pix_y = -origin[0] / resolution
print(origin_pix_y)
origin_pix_x = -origin[1] / resolution
print(origin_pix_x)
plt.plot(origin_pix_y, origin_pix_x, 'ro')
plt.title("Centerline on Map (Pixel)")
plt.show()
