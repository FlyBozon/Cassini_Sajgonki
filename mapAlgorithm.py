import cv2
from fill import replace_bright_pixels
import numpy as np
from sklearn.cluster import DBSCAN



input_path = "clouds.png"
output_path = "fill_mask2.jpg"

coordinates, mask = replace_bright_pixels(
    input_path,
    output_path,
    brightness=165,
    min_cluster_size=2000,
    smooth_kernel_size=5
)
print(f"Przetworzono obraz. Liczba wykrytych pikseli: {len(coordinates)}")
print(coordinates)



image = cv2.imread('sample4.png')
bg=cv2.imread('clouds.png')
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
threshold = 50
bright_points = np.column_stack(np.where(grayImage > threshold))  
dbscan = DBSCAN(eps=10, min_samples=30) 
clusters = dbscan.fit_predict(bright_points) 
output_image = image.copy()
scale_factor = 6
contourClouds = np.array(coordinates, dtype=np.int64)
for point in coordinates:
    x, y = point
    cv2.circle(bg, (x, y), radius=3, color=(255, 255, 255), thickness=-1)
for cluster_id in np.unique(clusters):
    if cluster_id == -1:
        continue 

    cluster_points = bright_points[clusters == cluster_id]
    mask = np.zeros_like(grayImage, dtype=np.uint8)
    for point in cluster_points:
        mask[point[0], point[1]] = 255 

    dilated_mask = cv2.dilate(mask, None, iterations=scale_factor) 
    contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        color = (0, 0, 255,200)  
        cv2.drawContours(bg, [contour], -1, color, thickness=cv2.FILLED)
  



# print (cluster_points)
cv2.imshow('Clusters Highlighted (expanded)', bg)
cv2.imwrite('main.png', bg)
cv2.waitKey(0)
cv2.destroyAllWindows()
