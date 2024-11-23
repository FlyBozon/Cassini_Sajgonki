import tkinter as tk
from tkintermapview import TkinterMapView
import time
import math


class DynamicMarkerMap:
    def __init__(self):
        # Początkowe współrzędne punktu (Warszawa)
        self.lat = 52.2297
        self.lon = 21.0122
        self.radius = 0.01  # Promień okręgu w stopniach geograficznych
        self.path_points = []  # Lista do przechowywania punktów śladu

        # Utwórz okno aplikacji
        self.root = tk.Tk()
        self.root.geometry("800x600")
        self.root.title("Mapa OSM z dynamicznymi markerami i śladem")

        # Dodaj widget mapy
        self.map_widget = TkinterMapView(self.root, width=800, height=600, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

        # Ustaw mapę na domyślną lokalizację
        self.map_widget.set_position(self.lat, self.lon)
        self.map_widget.set_zoom(13)

        # Dodaj marker
        self.marker = self.map_widget.set_marker(self.lat, self.lon, text="Dynamiczny punkt")

        # Zmienna do śledzenia poprzedniej pozycji
        self.prev_position = None

    def update_marker_position(self):
        """Funkcja do aktualizacji pozycji markera po okręgu i rysowania śladu."""
        # Oblicz przesunięcia na podstawie czasu
        angle = time.time() % (2 * math.pi)  # Kąt w radianach
        new_lat = self.lat + self.radius * math.sin(angle)
        new_lon = self.lon + (self.radius * math.cos(angle)) / math.cos(math.radians(self.lat))

        # Aktualizuj pozycję markera
        self.marker.set_position(new_lat, new_lon)

        # Dodaj nowy punkt do śladu
        current_position = (new_lat, new_lon)
        if self.prev_position:
            # Narysuj linię między poprzednią a obecną pozycją
            self.map_widget.set_path([self.prev_position, current_position],
                                     color="red",
                                     width=2)

        self.prev_position = current_position
        self.path_points.append(current_position)

        # Jeśli ślad jest zbyt długi, usuń najstarsze punkty
        if len(self.path_points) > 100:
            self.path_points = self.path_points[-100:]
            # Przerysuj cały ślad
            self.map_widget.set_path(self.path_points, color="red", width=2)

        self.root.after(100, self.update_marker_position)  # Aktualizacja co 100 ms

    def run(self):
        """Uruchom aplikację."""
        self.root.after(1000, self.update_marker_position)
        self.root.mainloop()


if __name__ == "__main__":
    app = DynamicMarkerMap()
    app.run()