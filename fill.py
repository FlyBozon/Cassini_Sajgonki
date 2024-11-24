import cv2
import numpy as np


def replace_bright_pixels(image_path, output_path, brightness=200, min_cluster_size=100, smooth_kernel_size=3):
    """
    Przetwarza obraz zamieniając jasne obszary na czarne.

    Parametry:
    - image_path: ścieżka do obrazu wejściowego
    - output_path: ścieżka do obrazu wyjściowego (maska)
    - brightness: próg jasności (0-255)
    - min_cluster_size: minimalna wielkość skupiska pikseli
    - smooth_kernel_size: rozmiar kernela do wygładzania (nieparzysta liczba >= 3)
    """
    if smooth_kernel_size < 3 or smooth_kernel_size % 2 == 0:
        raise ValueError("smooth_kernel_size musi być nieparzystą liczbą >= 3")

    if not 0 <= brightness <= 255:
        raise ValueError("brightness musi być w zakresie 0-255")

    img = cv2.imread(image_path)

    if img is None:
        raise Exception("Nie można wczytać obrazu")

    mask = (img[:, :, 0] > brightness) & (img[:, :, 1] > brightness) & (img[:, :, 2] > brightness)
    mask = mask.astype(np.uint8) * 255

    kernel = np.ones((smooth_kernel_size, smooth_kernel_size), np.uint8)
    mask_smoothed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_smoothed = cv2.morphologyEx(mask_smoothed, cv2.MORPH_OPEN, kernel)
    mask_smoothed = cv2.GaussianBlur(mask_smoothed, (smooth_kernel_size, smooth_kernel_size), 0)
    mask_smoothed = (mask_smoothed > 127).astype(np.uint8) * 255

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_smoothed, connectivity=8)
    clean_mask = np.zeros_like(mask)

    for label in range(1, num_labels):
        size = stats[label, cv2.CC_STAT_AREA]
        if size >= min_cluster_size:
            clean_mask[labels == label] = 255

    y_coords, x_coords = np.where(clean_mask > 0)
    coordinates = list(zip(x_coords, y_coords))

    cv2.imwrite(output_path, clean_mask)

    return coordinates, clean_mask


if __name__ == "__main__":
    input_path = "2024-11-02-00_00_2024-11-02-23_59_Sentinel-2_L2A_True_color.png"
    output_path = "fill_mask2.jpg"

    try:
        coordinates, mask = replace_bright_pixels(
            input_path,
            output_path,
            brightness=165,
            min_cluster_size=2000,
            smooth_kernel_size=5
        )
        print(f"Przetworzono obraz. Liczba wykrytych pikseli: {len(coordinates)}")

    except Exception as e:
        print(f"Wystąpił błąd: {str(e)}")