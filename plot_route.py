import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

top_left = [42.302921, -83.699841]
bot_right = [42.298942, -83.696336]

# Load the background image
bgImage = np.array(Image.open('mcity_map.png'))
bgImage_flipped = np.flipud(bgImage)  # Flip the image vertically
imageHeight, imageWidth, _ = bgImage_flipped.shape

fig, ax = plt.subplots()
ax.imshow(bgImage, extent=[0, imageWidth, 0, imageHeight])

# Plot lines
line_color = 'blue' 
line_width = 0.5
filenames = input("File names (separated by commas): ").split(', ')
output_filename = input("Output file name: ")
for filename in filenames:
    # Load data
    data = np.loadtxt(f'csv/{filename}', delimiter=',', skiprows=1)
    lat = data[:, 1]
    lon = data[:, 2]
    time = data[:, 0]

    # Normalize latitude and longitude
    normalizedLat = (lat - bot_right[0]) / (top_left[0] - bot_right[0]) * imageHeight
    normalizedLon = (lon - top_left[1]) / (bot_right[1] - top_left[1]) * imageWidth

    ax.plot(normalizedLon, normalizedLat, color=line_color, linewidth=line_width)

ax.set_xlim(0, imageWidth)
ax.set_ylim(0, imageHeight)
ax.set_aspect('equal')
ax.axis('off')

fig.savefig(f"plots/{output_filename}", dpi=600, bbox_inches='tight', pad_inches=0, transparent=True)
