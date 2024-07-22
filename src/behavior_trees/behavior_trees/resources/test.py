import argparse
from PIL import Image, ImageDraw
import numpy as np
import matplotlib.pyplot as plt

def overlay_grid(image_path, grid_size):
    # Open the image
    img = Image.open(image_path)
    print(img.size)
    # Convert to RGB if it's not already
    if img.mode != 'RGB':
        img = img.convert('RGB')
    
    # Create a drawing object
    draw = ImageDraw.Draw(img)
    
    # Get image dimensions
    width, height = img.size
    
    # Draw vertical lines
    for x in range(0, width, grid_size):
        line = ((x, 0), (x, height))
        draw.line(line, fill=(255, 0, 0))  # Red lines
    
    # Draw horizontal lines
    for y in range(0, height, grid_size):
        line = ((0, y), (width, y))
        draw.line(line, fill=(255, 0, 0))  # Red lines
    
    # Convert to numpy array for matplotlib
    img_array = np.array(img)
    
    return img_array

def main():
    image_path = 'src/behavior_trees/behavior_trees/resources/altered_map.pgm'
    grid_size = 9
    
    # Overlay grid on the image
    img_with_grid = overlay_grid(image_path, grid_size)
    
    # Display the image
    plt.figure(figsize=(12, 8))
    plt.imshow(img_with_grid)
    plt.title(f'PGM Image with {grid_size}x{grid_size} Grid')
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    main()