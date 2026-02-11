
import cv2
import numpy as np
import os

def create_noise_texture(filename, color=(255, 255, 255)):
    width, height = 512, 512
    # Create a base color image
    img = np.full((height, width, 3), color, dtype=np.uint8)
    
    # Add random noise
    noise = np.random.normal(0, 50, (height, width, 3)).astype(np.int16)
    img_noise = img.astype(np.int16) + noise
    img_noise = np.clip(img_noise, 0, 255).astype(np.uint8)
    
    # Add grid lines for extra structure
    for i in range(0, width, 64):
        cv2.line(img_noise, (i, 0), (i, height), (0, 0, 0), 2)
    for i in range(0, height, 64):
        cv2.line(img_noise, (0, i), (width, i), (0, 0, 0), 2)

    cv2.imwrite(filename, img_noise)
    print(f"Generated {filename}")

if __name__ == '__main__':
    base_path = '/home/david/ros2_car_ws/src/gazebo_car_sim_package/models/textures'
    os.makedirs(base_path, exist_ok=True)
    
    # 1. Red Scratched Texture for Wall
    create_noise_texture(os.path.join(base_path, 'box_texture.png'), color=(200, 50, 50))
    
    # 2. Grey Scratched Texture for Map Wall
    create_noise_texture(os.path.join(base_path, 'map_wall_texture.png'), color=(128, 128, 128))

    # 3. Blue Scratched Texture for Box
    create_noise_texture(os.path.join(base_path, 'wall_texture.png'), color=(50, 50, 200))

    # 4. Green Scratched Texture for Ground
    create_noise_texture(os.path.join(base_path, 'ground_texture.png'), color=(0, 150, 0))
