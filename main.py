import tkinter as tk
from tkintermapview import TkinterMapView
import time
import queue
import threading
import math


class DroneLocationTracker:
    def __init__(self):
        # Początkowa pozycja (centrum okręgu - Warszawa)
        self.center_lat = 52.2297
        self.center_lon = 21.0122

        # Parametry okręgu
        self.radius = 0.001  # Promień okręgu w stopniach
        self.angle = 0  # Aktualny kąt na okręgu
        self.angular_speed = 0.05  # Prędkość kątowa (radiany na update)

        # Aktualna pozycja i cel
        self.current_lat = self.center_lat + self.radius
        self.current_lon = self.center_lon
        self.target_lat = None
        self.target_lon = None

        # Stan drona
        self.flight_mode = "circle"  # "circle" lub "target"
        self.return_after_target = False  # Czy wracać do okręgu po osiągnięciu celu

        # Prędkość drona podczas lotu do celu
        self.target_speed = 0.0001

        # Kolejka do przesyłania lokalizacji
        self.location_queue = queue.Queue()

        # Utworzenie okna
        self.root = tk.Tk()
        self.root.geometry("800x600")
        self.root.title("Dron - lot po okręgu z przerwaniem na cel")

        # Dodaj frame na przyciski nad mapą
        self.button_frame = tk.Frame(self.root)
        self.button_frame.pack(side="top", fill="x", padx=5, pady=5)

        # Flaga wyświetlania ścieżki
        self.show_path = True

        # Przycisk do przełączania ścieżki
        self.toggle_path_button = tk.Button(
            self.button_frame,
            text="Ukryj ścieżkę",
            command=self.toggle_path
        )
        self.toggle_path_button.pack(side="left")

        # Widget mapy - zmień pack na poniżej przycisku
        self.map_widget = TkinterMapView(self.root, width=800, height=600, corner_radius=0)
        self.map_widget.pack(fill="both", expand=True)

        # Ustawienia mapy
        self.map_widget.set_position(self.center_lat, self.center_lon)
        self.map_widget.set_zoom(15)

        # Marker drona
        self.drone_marker = self.map_widget.set_marker(
            self.current_lat,
            self.current_lon,
            text="Dron"
        )

        # Marker celu
        self.target_marker = None

        # Lista punktów śladu
        self.path_points = []

        # Flaga działania
        self.running = True

        # Dodanie obsługi kliknięcia na mapie
        self.map_widget.add_left_click_map_command(self.on_map_click)

    def toggle_path(self):
        """Przełącza wyświetlanie ścieżki."""
        self.show_path = not self.show_path

        # Aktualizuj tekst przycisku
        self.toggle_path_button.config(
            text="Ukryj ścieżkę" if self.show_path else "Pokaż ścieżkę"
        )

        # Usuń ścieżkę jeśli trzeba
        if not self.show_path:
            for path in self.map_widget.canvas_path_list:
                self.map_widget.delete(path)

    def on_map_click(self, coords):
        """Obsługa kliknięcia na mapie."""
        lat, lon = coords

        # Aktualizacja celu
        self.target_lat = lat
        self.target_lon = lon

        # Zmiana trybu lotu
        self.flight_mode = "target"
        self.return_after_target = True

        # Aktualizacja lub utworzenie markera celu
        if self.target_marker:
            self.target_marker.set_position(lat, lon)
        else:
            self.target_marker = self.map_widget.set_marker(
                lat,
                lon,
                text="Cel",
                marker_color_circle="green"
            )

    def calculate_circle_position(self):
        """Oblicza pozycję na okręgu."""
        new_lat = self.center_lat + self.radius * math.sin(self.angle)
        new_lon = self.center_lon + (self.radius * math.cos(self.angle)) / math.cos(math.radians(self.center_lat))
        self.angle = (self.angle + self.angular_speed) % (2 * math.pi)
        return new_lat, new_lon

    def calculate_target_step(self):
        """Oblicza krok w kierunku celu."""
        if self.flight_mode == "target" and self.return_after_target:
            if self.target_lat is None:  # Jeśli nie ma aktywnego celu
                target_lat = self.center_lat + self.radius * math.sin(self.angle)
                target_lon = self.center_lon + (self.radius * math.cos(self.angle)) / math.cos(
                    math.radians(self.center_lat))
            else:
                target_lat, target_lon = self.target_lat, self.target_lon
        else:
            target_lat, target_lon = self.target_lat, self.target_lon

        dlat = target_lat - self.current_lat
        dlon = target_lon - self.current_lon
        distance = math.sqrt(dlat ** 2 + dlon ** 2)

        # Jeśli jesteśmy bardzo blisko celu
        if distance < self.target_speed:
            if self.return_after_target:
                if target_lat == self.target_lat:  # Jeśli dotarliśmy do punktu kliknięcia
                    # Zmień cel na najbliższy punkt na okręgu
                    angle_to_current = math.atan2(
                        self.current_lat - self.center_lat,
                        self.current_lon - self.center_lon
                    )
                    self.angle = angle_to_current
                    self.target_lat = None  # Usuń pierwotny cel
                else:  # Jeśli dotarliśmy do punktu na okręgu
                    self.flight_mode = "circle"
                    self.return_after_target = False
            return self.current_lat, self.current_lon

        # Oblicz wektor kierunku
        ratio = self.target_speed / distance
        step_lat = dlat * ratio
        step_lon = dlon * ratio

        # Wykonaj krok
        self.current_lat += step_lat
        self.current_lon += step_lon

        return self.current_lat, self.current_lon

    def simulate_drone_location(self):
        """Symuluje ruch drona."""
        while self.running:
            if self.flight_mode == "circle":
                # Lot po okręgu
                lat, lon = self.calculate_circle_position()
                self.current_lat, self.current_lon = lat, lon
            else:
                # Lot do celu
                lat, lon = self.calculate_target_step()

            # Wyślij pozycję do kolejki
            self.location_queue.put((lat, lon))

            # Częstotliwość aktualizacji
            time.sleep(0.05)

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

                # Aktualizacja ścieżki na mapie tylko jeśli jest włączona
                if self.show_path and len(self.path_points) >= 2:
                    # Usuń wszystkie ścieżki z mapy
                    for path in self.map_widget.canvas_path_list:
                        self.map_widget.delete(path)

                    # Stwórz nową ścieżkę
                    self.map_widget.set_path(self.path_points, color="red", width=2)

        except queue.Empty:
            pass

        if self.running:
            self.root.after(50, self.update_display)

    def run(self):
        """Uruchamia aplikację."""
        # Uruchomienie wątku symulacji drona
        threading.Thread(target=self.simulate_drone_location, daemon=True).start()

        # Rozpoczęcie aktualizacji wyświetlania
        self.root.after(50, self.update_display)

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