import cv2
import numpy as np
from sklearn.cluster import DBSCAN

image = cv2.imread('sample.png')
grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
threshold = 50
bright_points = np.column_stack(np.where(grayImage > threshold))  
dbscan = DBSCAN(eps=10, min_samples=25) 
clusters = dbscan.fit_predict(bright_points) 
output_image = image.copy()
scale_factor = 10 

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
        color = (0, 0, 255)  
        cv2.drawContours(output_image, [contour], -1, color, thickness=cv2.FILLED)

print (cluster_points)
cv2.imshow('Clusters Highlighted (expanded)', output_image)
cv2.imwrite('clusters_expanded_contours.png', output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
