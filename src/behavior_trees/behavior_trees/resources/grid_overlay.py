from PIL import Image, ImageDraw
import numpy as np
import matplotlib.pyplot as plt

def overlay_grid(image_path, grid_size):

    img = Image.open(image_path)
    print(img.size)
    if img.mode != 'RGB':
        img = img.convert('RGB')
    
    draw = ImageDraw.Draw(img)

    width, height = img.size
    
    for x in range(0, width, grid_size):
        line = ((x, 0), (x, height))
        draw.line(line, fill=(255, 0, 0))
    
    for y in range(0, height, grid_size):
        line = ((0, y), (width, y))
        draw.line(line, fill=(255, 0, 0))
    
    img_array = np.array(img)
    
    return img_array

def main():
    image_path = 'src/behavior_trees/behavior_trees/resources/aws_small_house_map.pgm'
    grid_size = 9
    
    img_with_grid = overlay_grid(image_path, grid_size)
    
    plt.figure(figsize=(12, 8))
    plt.imshow(img_with_grid)
    plt.title(f'PGM Image with {grid_size}x{grid_size} Grid')
    plt.axis('off')
    plt.show()

if __name__ == "__main__":
    main()