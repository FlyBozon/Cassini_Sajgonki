import cv2
import numpy as np


def replace_bright_pixels(image_path, output_path, brightness=200, min_cluster_size=100, smooth_kernel_size=3):
    """
    Przetwarza obraz zamieniając jasne obszary na czarne i zwraca ich kontury.

    Parametry:
    - image_path: ścieżka do obrazu wejściowego
    - output_path: ścieżka do obrazu wyjściowego (maska)
    - brightness: próg jasności (0-255)
    - min_cluster_size: minimalna wielkość skupiska pikseli
    - smooth_kernel_size: rozmiar kernela do wygładzania (nieparzysta liczba >= 3)

    Zwraca:
    - Lista punktów konturu
    - Maska binarna
    """
    if smooth_kernel_size < 3 or smooth_kernel_size % 2 == 0:
        raise ValueError("smooth_kernel_size musi być nieparzystą liczbą >= 3")

    if not 0 <= brightness <= 255:
        raise ValueError("brightness musi być w zakresie 0-255")

    # Wczytaj obraz
    img = cv2.imread(image_path)

    if img is None:
        raise Exception("Nie można wczytać obrazu")

    # 1. Stwórz podstawową maskę dla jasnych pikseli
    mask = (img[:, :, 0] > brightness) & (img[:, :, 1] > brightness) & (img[:, :, 2] > brightness)
    mask = mask.astype(np.uint8) * 255

    # 2. Wygładź krawędzie używając operacji morfologicznych
    kernel = np.ones((smooth_kernel_size, smooth_kernel_size), np.uint8)
    mask_smoothed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask_smoothed = cv2.morphologyEx(mask_smoothed, cv2.MORPH_OPEN, kernel)
    mask_smoothed = cv2.GaussianBlur(mask_smoothed, (smooth_kernel_size, smooth_kernel_size), 0)
    mask_smoothed = (mask_smoothed > 127).astype(np.uint8) * 255

    # 3. Usuń małe skupiska
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask_smoothed, connectivity=8)
    clean_mask = np.zeros_like(mask)

    for label in range(1, num_labels):
        size = stats[label, cv2.CC_STAT_AREA]
        if size >= min_cluster_size:
            clean_mask[labels == label] = 255

    # 4. Znajdź kontury obszarów
    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Zbierz wszystkie punkty konturu
    boundary_points = []
    for contour in contours:
        # Spłaszcz punkty konturu do listy (x,y)
        points = contour.reshape(-1, 2).tolist()
        boundary_points.extend(points)

    # Opcjonalnie: narysuj kontury na masce dla wizualizacji
    contour_mask = np.zeros_like(clean_mask)
    cv2.drawContours(contour_mask, contours, -1, (255), 1)

    # Zapisz maskę z konturami
    cv2.imwrite(output_path, contour_mask)

    return boundary_points, clean_mask


# Przykład użycia
if __name__ == "__main__":
    input_path = "chmury2.jpg"
    output_path = "contour_mask.jpg"

    try:
        boundary_points, mask = replace_bright_pixels(
            input_path,
            output_path,
            brightness=165,  # próg jasności
            min_cluster_size=2000,  # minimalna wielkość skupiska
            smooth_kernel_size=5  # rozmiar kernela wygładzania
        )
        print(f"Przetworzono obraz. Liczba punktów konturu: {len(boundary_points)}")

        # Opcjonalnie: zapisz punkty konturu do pliku
        with open('boundary_points.txt', 'w') as f:
            for x, y in boundary_points:
                f.write(f"{x},{y}\n")

    except Exception as e:
        print(f"Wystąpił błąd: {str(e)}")