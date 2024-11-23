import tkinter as tk
from tkintermapview import TkinterMapView
import time
import queue
import threading
import math


class DroneLocationTracker:
    def __init__(self):
        # Początkowa pozycja (Warszawa)
        self.start_lat = 52.2297
        self.start_lon = 21.0122
        self.radius = 0.01

        # Kolejka do przesyłania lokalizacji
        self.location_queue = queue.Queue()

        # Utworzenie okna
        self.root = tk.Tk()
        self.root.geometry("800x600")
        self.root.title("Śledzenie pozycji drona")

        # Widget mapy
        self.map_widget = TkinterMapView(self.root, width=800, height=600, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

        # Ustawienia mapy
        self.map_widget.set_position(self.start_lat, self.start_lon)
        self.map_widget.set_zoom(15)

        # Marker drona
        self.drone_marker = self.map_widget.set_marker(self.start_lat, self.start_lon, text="Dron")

        # Lista punktów śladu
        self.path_points = []

        # Flaga działania
        self.running = True

    def simulate_drone_location(self):
        """Symuluje odbieranie lokalizacji z drona."""
        current_lat = self.start_lat
        current_lon = self.start_lon

        while self.running:
            # Symulacja drobnych zmian pozycji
            # current_lat += random.uniform(-0.0001, 0.0001)
            # current_lon += random.uniform(-0.0001, 0.0001)
            angle = time.time() % (2 * math.pi)  # Kąt w radianach
            current_lat = self.start_lat + self.radius * math.sin(angle)
            current_lon = self.start_lon + (self.radius * math.cos(angle)) / math.cos(math.radians(self.start_lat))

            # Wysłanie lokalizacji do kolejki
            self.location_queue.put((current_lat, current_lon))

            # Symulacja częstotliwości odbierania danych (co 100ms)
            time.sleep(0.1)

    def update_display(self):
        """Aktualizuje pozycję drona na mapie."""
        try:
            while not self.location_queue.empty():
                lat, lon = self.location_queue.get_nowait()

                # Aktualizacja pozycji markera
                self.drone_marker.set_position(lat, lon)

                # Dodanie punktu do śladu
                current_position = (lat, lon)
                self.path_points.append(current_position)

                # Ograniczenie długości śladu
                if len(self.path_points) > 100:
                    self.path_points = self.path_points[-100:]

                # Aktualizacja ścieżki na mapie tylko jeśli mamy co najmniej 2 punkty
                if len(self.path_points) >= 2:
                    # Usuń wszystkie ścieżki z mapy
                    for path in self.map_widget.canvas_path_list:
                        self.map_widget.delete(path)

                    # Stwórz nową ścieżkę
                    self.map_widget.set_path(self.path_points, color="grey", width=15)

        except queue.Empty:
            pass

        if self.running:
            self.root.after(100, self.update_display)

    def run(self):
        """Uruchamia aplikację."""
        # Uruchomienie wątku odbierającego lokalizację
        threading.Thread(target=self.simulate_drone_location, daemon=True).start()

        # Rozpoczęcie aktualizacji wyświetlania
        self.root.after(100, self.update_display)

        # Obsługa zamknięcia okna
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Uruchomienie głównej pętli
        self.root.mainloop()

    def on_closing(self):
        """Obsługuje zamknięcie aplikacji."""
        self.running = False
        self.root.destroy()


if __name__ == "__main__":
    tracker = DroneLocationTracker()
    tracker.run()