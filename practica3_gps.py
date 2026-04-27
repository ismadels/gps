from __future__ import annotations

import argparse
import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None

from PIL import Image, ImageTk


# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 4800
DEFAULT_ZONE = 30

# Ajustados para que no quede todo apretado
DEFAULT_DISPLAY_WIDTH = 800
DEFAULT_PANEL_WIDTH = 560
DEFAULT_WINDOW_GEOMETRY = "1450x900"

ELLIPSOIDS = {
    "WGS84": {"name": "WGS84", "a": 6378137.0, "f": 1.0 / 298.257223563},
    "ED50": {"name": "Europeo 1950", "a": 6378388.0, "f": 1.0 / 297.0},
}

FIX_QUALITY = {
    0: "inválido",
    1: "GPS fix (SPS)",
    2: "DGPS fix",
    3: "PPS fix",
    4: "RTK",
    5: "Float RTK",
    6: "estimado",
    7: "manual",
    8: "simulación",
}

COLOR_OK = "#00C853"
COLOR_WARN = "#FFD600"
COLOR_ALERT = "#D50000"
COLOR_NO_LIMIT = "#1E88E5"
COLOR_NEUTRAL = "#D9D9D9"


# =========================================================
# DATOS
# =========================================================

@dataclass
class GGAData:
    time_utc: str
    fix_quality: int
    satellites: int
    hdop: float
    altitude_m: float
    latitude_deg: float | None
    longitude_deg: float | None


@dataclass(frozen=True)
class MapConfig:
    key: str
    title: str
    image_path: str
    orig_width: int
    orig_height: int
    display_width: int
    zone: int
    e_tl: float
    n_tl: float
    e_br: float
    n_br: float
    has_speed_limits: bool = False
    track_path: str | None = None


@dataclass
class TrackPoint:
    north: float
    east: float
    vmax: float


# =========================================================
# CONFIGURACIÓN DE MAPAS
# =========================================================

def map_config_insia(display_width: int) -> MapConfig:
    return MapConfig(
        key="insia",
        title="INSIA",
        image_path="gps_insia.png",
        orig_width=1481,
        orig_height=1012,
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=446183.14,
        n_tl=4470981.42,
        e_br=446490.92,
        n_br=4470767.54,
        has_speed_limits=True,
        track_path="Mapa_INSIA2.txt",
    )


def map_config_campus_sur(display_width: int) -> MapConfig:
    return MapConfig(
        key="campus",
        title="Campus Sur",
        image_path="campus_sur.png",
        orig_width=1481,
        orig_height=1012,
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=446425.48,
        n_tl=4471356.42,
        e_br=446992.21,
        n_br=4470961.96,
        has_speed_limits=True,
        track_path=None,
    )


def map_config_vicalvaro(display_width: int) -> MapConfig:
    return MapConfig(
        key="vicalvaro",
        title="Vicálvaro",
        image_path="vicalvaro.png",
        orig_width=1481,
        orig_height=1012,
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=447617.88,
        n_tl=4473623.84,
        e_br=448746.78,
        n_br=4472836.14,
        has_speed_limits=False,
        track_path=None,
    )


# =========================================================
# CARGA TRACK
# =========================================================

def load_track(path: str) -> list[TrackPoint]:
    points: list[TrackPoint] = []

    with open(path, "r", encoding="utf-8") as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue

            parts = line.replace(",", " ").split()
            if len(parts) != 3:
                raise ValueError(
                    f"Línea {line_num} inválida en {path}: se esperaban 3 valores."
                )

            north, east, vmax = map(float, parts)
            points.append(TrackPoint(north=north, east=east, vmax=vmax))

    if len(points) < 3:
        raise ValueError("El mapa electrónico debe contener al menos 3 puntos.")

    return points


# =========================================================
# NMEA / GGA
# =========================================================

def compute_checksum(nmea_body: str) -> int:
    value = 0
    for ch in nmea_body:
        value ^= ord(ch)
    return value


def verify_checksum(sentence: str) -> bool:
    line = sentence.strip()
    if not line.startswith("$") or "*" not in line:
        return False

    body = line[1: line.find("*")]
    chk_text = line[line.find("*") + 1: line.find("*") + 3]

    if len(chk_text) != 2:
        return False

    try:
        expected = int(chk_text, 16)
    except ValueError:
        return False

    return compute_checksum(body) == expected


def nmea_coord_to_decimal(raw_value: str, is_latitude: bool) -> float | None:
    if not raw_value:
        return None

    deg_len = 2 if is_latitude else 3
    if len(raw_value) < deg_len + 2:
        return None

    try:
        degrees = int(raw_value[:deg_len])
        minutes = float(raw_value[deg_len:])
    except ValueError:
        return None

    return degrees + minutes / 60.0


def parse_gga(sentence: str) -> GGAData | None:
    if not verify_checksum(sentence):
        return None

    line = sentence.strip()
    body = line[1: line.find("*")]
    fields = body.split(",")

    if len(fields) < 15:
        return None

    if fields[0] not in ("GPGGA", "GNGGA"):
        return None

    raw_time = fields[1]
    if len(raw_time) >= 6:
        hh = raw_time[0:2]
        mm = raw_time[2:4]
        ss = raw_time[4:6]
        frac = raw_time[6:] if len(raw_time) > 6 else ""
        time_utc = f"{hh}:{mm}:{ss}{frac}"
    else:
        time_utc = "N/A"

    try:
        fix_quality = int(fields[6]) if fields[6] else 0
    except ValueError:
        fix_quality = 0

    try:
        satellites = int(fields[7]) if fields[7] else 0
    except ValueError:
        satellites = 0

    try:
        hdop = float(fields[8]) if fields[8] else 0.0
    except ValueError:
        hdop = 0.0

    try:
        altitude_m = float(fields[9]) if fields[9] else 0.0
    except ValueError:
        altitude_m = 0.0

    latitude_deg = None
    longitude_deg = None

    if fix_quality != 0:
        lat = nmea_coord_to_decimal(fields[2], is_latitude=True)
        lon = nmea_coord_to_decimal(fields[4], is_latitude=False)
        lat_dir = fields[3]
        lon_dir = fields[5]

        if lat is not None and lon is not None and lat_dir and lon_dir:
            latitude_deg = -lat if lat_dir == "S" else lat
            longitude_deg = -lon if lon_dir == "W" else lon

    return GGAData(
        time_utc=time_utc,
        fix_quality=fix_quality,
        satellites=satellites,
        hdop=hdop,
        altitude_m=altitude_m,
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
    )


# =========================================================
# GEODESIA
# =========================================================

def get_ellipsoid_parameters(name: str) -> dict[str, float | str]:
    ell = ELLIPSOIDS[name]
    a = float(ell["a"])
    f = float(ell["f"])
    b = a * (1.0 - f)

    first_e2 = (a * a - b * b) / (a * a)
    second_e2 = (a * a - b * b) / (b * b)
    c = (a * a) / b

    return {
        "name": str(ell["name"]),
        "a": a,
        "b": b,
        "f": f,
        "first_e2": first_e2,
        "second_e2": second_e2,
        "c": c,
    }


def geo_to_utm(
    latitude_deg: float,
    longitude_deg: float,
    ellipsoid_name: str,
    zone: int
) -> tuple[float, float, str]:
    ell = get_ellipsoid_parameters(ellipsoid_name)
    a = float(ell["a"])
    e2 = float(ell["first_e2"])
    ep2 = float(ell["second_e2"])

    phi = math.radians(latitude_deg)
    lam = math.radians(longitude_deg)
    lambda_0 = math.radians(zone * 6 - 183)

    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)

    n = a / math.sqrt(1.0 - e2 * sin_phi ** 2)
    t = tan_phi ** 2
    c_term = ep2 * cos_phi ** 2
    a_term = cos_phi * (lam - lambda_0)

    m = a * (
        (1.0 - e2 / 4.0 - 3.0 * e2 ** 2 / 64.0 - 5.0 * e2 ** 3 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e2 ** 2 / 32.0 + 45.0 * e2 ** 3 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * e2 ** 2 / 256.0 + 45.0 * e2 ** 3 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e2 ** 3 / 3072.0) * math.sin(6.0 * phi)
    )

    easting = 0.9996 * n * (
        a_term
        + (1.0 - t + c_term) * a_term ** 3 / 6.0
        + (5.0 - 18.0 * t + t ** 2 + 72.0 * c_term - 58.0 * ep2) * a_term ** 5 / 120.0
    ) + 500000.0

    northing = 0.9996 * (
        m
        + n * tan_phi * (
            a_term ** 2 / 2.0
            + (5.0 - t + 9.0 * c_term + 4.0 * c_term ** 2) * a_term ** 4 / 24.0
            + (61.0 - 58.0 * t + t ** 2 + 600.0 * c_term - 330.0 * ep2) * a_term ** 6 / 720.0
        )
    )

    if latitude_deg < 0:
        northing += 10000000.0

    hemisphere = "N" if latitude_deg >= 0 else "S"
    return easting, northing, hemisphere


# =========================================================
# UTM -> PÍXEL
# =========================================================

def utm_to_pixel_original(cfg: MapConfig, easting: float, northing: float) -> tuple[int, int]:
    x = ((easting - cfg.e_tl) / (cfg.e_br - cfg.e_tl)) * (cfg.orig_width - 1)
    y = ((cfg.n_tl - northing) / (cfg.n_tl - cfg.n_br)) * (cfg.orig_height - 1)
    return int(round(x)), int(round(y))


def original_pixel_to_display(cfg: MapConfig, x: int, y: int) -> tuple[int, int]:
    display_height = int(cfg.orig_height * cfg.display_width / cfg.orig_width)
    x_disp = int(round(x * cfg.display_width / cfg.orig_width))
    y_disp = int(round(y * display_height / cfg.orig_height))
    return x_disp, y_disp


def inside_original_image(cfg: MapConfig, x: int, y: int) -> bool:
    return 0 <= x < cfg.orig_width and 0 <= y < cfg.orig_height


# =========================================================
# CÁLCULOS
# =========================================================

def euclidean_distance(e1: float, n1: float, e2: float, n2: float) -> float:
    return math.sqrt((e2 - e1) ** 2 + (n2 - n1) ** 2)


def compute_speed_kmh(
    prev_e: float,
    prev_n: float,
    prev_t: float,
    curr_e: float,
    curr_n: float,
    curr_t: float
) -> float:
    dt = curr_t - prev_t
    if dt <= 0:
        return 0.0
    d = euclidean_distance(prev_e, prev_n, curr_e, curr_n)
    return (d / dt) * 3.6


def dot(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * bx + ay * by


def nearest_track_index(track: list[TrackPoint], e: float, n: float, candidate_indices: list[int] | None = None) -> int:
    if candidate_indices is None:
        candidate_indices = list(range(len(track)))

    best_i = candidate_indices[0]
    best_d = float("inf")

    for i in candidate_indices:
        p = track[i]
        d = (p.east - e) ** 2 + (p.north - n) ** 2
        if d < best_d:
            best_d = d
            best_i = i

    return best_i


def infer_direction_from_motion(track: list[TrackPoint], idx: int, move_e: float, move_n: float) -> int:
    size = len(track)
    i_prev = (idx - 1) % size
    i_next = (idx + 1) % size

    fwd_e = track[i_next].east - track[idx].east
    fwd_n = track[i_next].north - track[idx].north

    bwd_e = track[i_prev].east - track[idx].east
    bwd_n = track[i_prev].north - track[idx].north

    score_fwd = dot(move_e, move_n, fwd_e, fwd_n)
    score_bwd = dot(move_e, move_n, bwd_e, bwd_n)

    return 1 if score_fwd >= score_bwd else -1


def circular_indices(center: int, size: int, back: int, forward: int, direction: int) -> list[int]:
    indices = []

    if direction >= 0:
        start = -back
        end = forward
    else:
        start = -forward
        end = back

    for offset in range(start, end + 1):
        indices.append((center + offset) % size)

    return indices


def speed_state(speed_kmh: float, limit_kmh: float | None) -> tuple[str, str]:
    if limit_kmh is None or limit_kmh <= 0:
        return "SIN_LIMITE", COLOR_NO_LIMIT

    if speed_kmh < 0.9 * limit_kmh:
        return "OK", COLOR_OK
    elif speed_kmh <= 1.1 * limit_kmh:
        return "CERCA", COLOR_WARN
    else:
        return "EXCESO", COLOR_ALERT


# =========================================================
# INTERFAZ
# =========================================================

class GPSMultiMapApp:
    def __init__(self, root: tk.Tk, cfg: MapConfig, track: list[TrackPoint] | None = None):
        self.root = root
        self.cfg = cfg
        self.track = track
        self.display_height = int(cfg.orig_height * cfg.display_width / cfg.orig_width)

        self.root.title(f"{cfg.title} - Navegación GPS y aviso al conductor")
        self.root.geometry(DEFAULT_WINDOW_GEOMETRY)
        self.root.minsize(1300, 820)

        self.last_x_disp: int | None = None
        self.last_y_disp: int | None = None

        self.prev_e: float | None = None
        self.prev_n: float | None = None
        self.prev_t: float | None = None

        self.prev_track_idx: int | None = None
        self.direction_sign: int = 1
        self.speed_history: list[float] = []

        main_frame = tk.Frame(root, bg="#f0f0f0")
        main_frame.pack(fill="both", expand=True)

        map_frame = tk.Frame(main_frame, padx=10, pady=10, bg="#f0f0f0")
        map_frame.pack(side=tk.LEFT, fill="both", expand=True)

        panel_frame = tk.Frame(main_frame, padx=14, pady=10, bg="#f7f7f7", width=DEFAULT_PANEL_WIDTH)
        panel_frame.pack(side=tk.RIGHT, fill=tk.Y)
        panel_frame.pack_propagate(False)

        try:
            self.base_image_pil = Image.open(cfg.image_path)
        except Exception as exc:
            raise RuntimeError(
                f"No se pudo abrir la imagen del mapa: {cfg.image_path}. "
                f"Pon la imagen en el mismo directorio del script. Detalle: {exc}"
            ) from exc

        self.base_image_pil = self.base_image_pil.resize((cfg.display_width, self.display_height))
        self.base_image_tk = ImageTk.PhotoImage(self.base_image_pil)

        self.canvas = tk.Canvas(
            map_frame,
            width=cfg.display_width,
            height=self.display_height,
            bg="white",
            highlightthickness=1,
            highlightbackground="black"
        )
        self.canvas.pack(anchor="nw")

        self.canvas.create_image(0, 0, anchor="nw", image=self.base_image_tk)

        if self.track is not None:
            self.draw_track_reference()

        self.gps_radius = 8
        self.gps_marker = self.canvas.create_oval(
            -100, -100, -100, -100,
            fill="red",
            outline="black",
            width=2
        )
        self.gps_label = self.canvas.create_text(
            -100, -100,
            text="GPS",
            fill="black",
            font=("Arial", 10, "bold")
        )

        tk.Label(
            panel_frame,
            text=f"SISTEMA GPS - {cfg.title.upper()}",
            font=("Arial", 15, "bold"),
            bg="#f7f7f7",
            wraplength=DEFAULT_PANEL_WIDTH - 30,
            justify="center"
        ).pack(pady=(5, 12), fill="x")

        self.big_speed_var = tk.StringVar(value="0.0 km/h")
        tk.Label(
            panel_frame,
            textvariable=self.big_speed_var,
            font=("Arial", 30, "bold"),
            fg="#111111",
            bg="#f7f7f7"
        ).pack(pady=(0, 10))

        self.limit_var = tk.StringVar(value="Límite actual: no disponible")
        self.alert_var = tk.StringVar(value="Estado: esperando datos GPS")
        self.direction_var = tk.StringVar(value="Sentido: -")
        self.map_mode_var = tk.StringVar(
            value="Modo: con límites INSIA" if self.cfg.has_speed_limits else "Modo: sin límites de velocidad"
        )

        tk.Label(panel_frame, textvariable=self.map_mode_var, font=("Arial", 13, "bold"), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=2, fill="x")
        tk.Label(panel_frame, textvariable=self.limit_var, font=("Arial", 14, "bold"), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")
        tk.Label(panel_frame, textvariable=self.alert_var, font=("Arial", 13), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")
        tk.Label(panel_frame, textvariable=self.direction_var, font=("Arial", 12), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")

        self.speed_canvas = tk.Canvas(
            panel_frame,
            width=440,
            height=175,
            bg="white",
            highlightthickness=1,
            highlightbackground="black"
        )
        self.speed_canvas.pack(pady=12)

        self.zone_green = self.speed_canvas.create_rectangle(
            10, 58, 150, 128, fill=COLOR_NEUTRAL, outline="black"
        )
        self.zone_yellow = self.speed_canvas.create_rectangle(
            150, 58, 295, 128, fill=COLOR_NEUTRAL, outline="black"
        )
        self.zone_red = self.speed_canvas.create_rectangle(
            295, 58, 430, 128, fill=COLOR_NEUTRAL, outline="black"
        )

        self.speed_canvas.create_text(80, 22, text="V < límite-10%", font=("Arial", 12))
        self.speed_canvas.create_text(222, 22, text="±10% límite", font=("Arial", 12))
        self.speed_canvas.create_text(362, 22, text="V > límite+10%", font=("Arial", 12))

        self.speed_canvas.create_text(220, 42, text="Velocidad actual", font=("Arial", 12, "bold"))
        self.speed_text = self.speed_canvas.create_text(
            220, 93, text="0.0 km/h", font=("Arial", 24, "bold")
        )

        separator = tk.Frame(panel_frame, height=2, bg="#cccccc")
        separator.pack(fill="x", pady=10)

        self.status_var = tk.StringVar(value="Esperando datos GPS...")
        self.utc_var = tk.StringVar(value="Hora UTC: -")
        self.fix_var = tk.StringVar(value="Fix: -")
        self.sat_var = tk.StringVar(value="Satélites: -")
        self.hdop_var = tk.StringVar(value="HDOP: -")
        self.alt_var = tk.StringVar(value="Altitud: -")
        self.lat_var = tk.StringVar(value="Latitud: -")
        self.lon_var = tk.StringVar(value="Longitud: -")
        self.utm_var = tk.StringVar(value="UTM: -")
        self.pix_var = tk.StringVar(value="Píxel pantalla: -")
        self.track_var = tk.StringVar(value="Punto mapa: no aplica")

        vars_to_show = [
            self.status_var,
            self.utc_var,
            self.fix_var,
            self.sat_var,
            self.hdop_var,
            self.alt_var,
            self.lat_var,
            self.lon_var,
            self.utm_var,
            self.pix_var,
            self.track_var,
        ]

        for var in vars_to_show:
            tk.Label(
                panel_frame,
                textvariable=var,
                anchor="w",
                justify="left",
                wraplength=470,
                bg="#f7f7f7",
                font=("Arial", 11)
            ).pack(fill="x", pady=3)

    def draw_track_reference(self) -> None:
        if self.track is None:
            return

        screen_points: list[tuple[int, int]] = []

        for p in self.track:
            x_orig, y_orig = utm_to_pixel_original(self.cfg, p.east, p.north)
            x_disp, y_disp = original_pixel_to_display(self.cfg, x_orig, y_orig)
            screen_points.append((x_disp, y_disp))

        for i in range(len(screen_points) - 1):
            x1, y1 = screen_points[i]
            x2, y2 = screen_points[i + 1]
            self.canvas.create_line(x1, y1, x2, y2, fill="#808080", width=2)

        x1, y1 = screen_points[-1]
        x2, y2 = screen_points[0]
        self.canvas.create_line(x1, y1, x2, y2, fill="#808080", width=2)

    def get_best_track_index(self, e: float, n: float) -> int:
        if self.track is None:
            raise RuntimeError("No hay track cargado para este mapa.")

        if self.prev_track_idx is None:
            return nearest_track_index(self.track, e, n)

        candidate_indices = circular_indices(
            center=self.prev_track_idx,
            size=len(self.track),
            back=8,
            forward=20,
            direction=self.direction_sign,
        )
        return nearest_track_index(self.track, e, n, candidate_indices=candidate_indices)

    def update_speedometer(self, speed_kmh: float, limit_kmh: float | None) -> tuple[str, str]:
        state, color = speed_state(speed_kmh, limit_kmh)

        self.speed_canvas.itemconfig(self.zone_green, fill=COLOR_NEUTRAL)
        self.speed_canvas.itemconfig(self.zone_yellow, fill=COLOR_NEUTRAL)
        self.speed_canvas.itemconfig(self.zone_red, fill=COLOR_NEUTRAL)

        if state == "OK":
            self.speed_canvas.itemconfig(self.zone_green, fill=COLOR_OK)
            status_txt = "Velocidad correcta"
            limit_txt = f"Límite actual: {limit_kmh:.1f} km/h"
        elif state == "CERCA":
            self.speed_canvas.itemconfig(self.zone_yellow, fill=COLOR_WARN)
            status_txt = "Cerca del límite"
            limit_txt = f"Límite actual: {limit_kmh:.1f} km/h"
        elif state == "EXCESO":
            self.speed_canvas.itemconfig(self.zone_red, fill=COLOR_ALERT)
            status_txt = "Exceso de velocidad"
            limit_txt = f"Límite actual: {limit_kmh:.1f} km/h"
        else:
            self.speed_canvas.itemconfig(self.zone_yellow, fill=COLOR_NO_LIMIT)
            status_txt = "Velocidad mostrada sin límites disponibles"
            limit_txt = "Límite actual: no disponible"

        self.speed_canvas.itemconfig(self.speed_text, text=f"{speed_kmh:.1f} km/h")
        self.big_speed_var.set(f"{speed_kmh:.1f} km/h")
        self.limit_var.set(limit_txt)
        self.alert_var.set(f"Estado: {status_txt}")

        return state, color

    def update_position(self, gga: GGAData) -> None:
        self.utc_var.set(f"Hora UTC: {gga.time_utc}")
        self.fix_var.set(f"Fix: {gga.fix_quality} ({FIX_QUALITY.get(gga.fix_quality, 'N/A')})")
        self.sat_var.set(f"Satélites: {gga.satellites}")
        self.hdop_var.set(f"HDOP: {gga.hdop}")
        self.alt_var.set(f"Altitud: {gga.altitude_m:.2f} m")

        if gga.latitude_deg is None or gga.longitude_deg is None:
            self.status_var.set("Sin fix válido.")
            self.lat_var.set("Latitud: -")
            self.lon_var.set("Longitud: -")
            self.utm_var.set("UTM: -")
            self.pix_var.set("Píxel pantalla: -")
            self.track_var.set("Punto mapa: -")
            return

        self.lat_var.set(f"Latitud: {gga.latitude_deg:.6f}")
        self.lon_var.set(f"Longitud: {gga.longitude_deg:.6f}")

        easting, northing, hemisphere = geo_to_utm(
            gga.latitude_deg,
            gga.longitude_deg,
            "WGS84",
            self.cfg.zone
        )

        self.utm_var.set(f"UTM: E={easting:.2f}, N={northing:.2f}, Zona={self.cfg.zone}{hemisphere}")

        x_orig, y_orig = utm_to_pixel_original(self.cfg, easting, northing)

        if not inside_original_image(self.cfg, x_orig, y_orig):
            self.status_var.set("La posición GPS está fuera de la zona georreferenciada.")
            self.pix_var.set("Píxel pantalla: fuera de imagen")
            self.canvas.coords(self.gps_marker, -100, -100, -100, -100)
            self.canvas.coords(self.gps_label, -100, -100)
            return

        x_disp, y_disp = original_pixel_to_display(self.cfg, x_orig, y_orig)
        self.pix_var.set(f"Píxel pantalla: x={x_disp}, y={y_disp}")

        now_t = time.time()

        raw_speed_kmh = 0.0
        move_e = 0.0
        move_n = 0.0

        if self.prev_e is not None and self.prev_n is not None and self.prev_t is not None:
            raw_speed_kmh = compute_speed_kmh(
                self.prev_e, self.prev_n, self.prev_t,
                easting, northing, now_t
            )
            move_e = easting - self.prev_e
            move_n = northing - self.prev_n

        self.speed_history.append(raw_speed_kmh)
        if len(self.speed_history) > 5:
            self.speed_history.pop(0)

        speed_kmh = sum(self.speed_history) / len(self.speed_history)

        limit_kmh: float | None = None

        if self.track is not None and self.cfg.has_speed_limits:
            track_idx = self.get_best_track_index(easting, northing)
            track_point = self.track[track_idx]
            limit_kmh = track_point.vmax

            self.track_var.set(
                f"Punto mapa: idx={track_idx}, E={track_point.east:.2f}, N={track_point.north:.2f}, Vmax={limit_kmh:.1f}"
            )

            movement_norm = math.hypot(move_e, move_n)
            if movement_norm > 0.20:
                self.direction_sign = infer_direction_from_motion(self.track, track_idx, move_e, move_n)

            if self.direction_sign >= 0:
                self.direction_var.set("Sentido: horario / orden del mapa")
            else:
                self.direction_var.set("Sentido: antihorario / orden inverso")

            self.prev_track_idx = track_idx
        else:
            self.track_var.set("Punto mapa: no aplica en este mapa")
            self.direction_var.set("Sentido: no evaluado (sin mapa de límites)")

        _, current_color = self.update_speedometer(speed_kmh, limit_kmh)

        if self.last_x_disp is not None and self.last_y_disp is not None:
            self.canvas.create_line(
                self.last_x_disp,
                self.last_y_disp,
                x_disp,
                y_disp,
                fill=current_color,
                width=3
            )

        self.last_x_disp = x_disp
        self.last_y_disp = y_disp

        r = self.gps_radius
        self.canvas.coords(
            self.gps_marker,
            x_disp - r,
            y_disp - r,
            x_disp + r,
            y_disp + r
        )
        self.canvas.coords(self.gps_label, x_disp, y_disp - 16)

        self.status_var.set("GPS dentro de la imagen.")

        self.prev_e = easting
        self.prev_n = northing
        self.prev_t = now_t


# =========================================================
# LECTURA SERIE
# =========================================================

def serial_worker(app: GPSMultiMapApp, port: str, baudrate: int) -> None:
    if serial is None:
        app.root.after(
            0,
            lambda: app.status_var.set(
                "ERROR: falta pyserial en este entorno. Instala con: python -m pip install pyserial"
            )
        )
        return

    try:
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        ) as gps:
            app.root.after(
                0,
                lambda: app.status_var.set(f"Conectado a {port} a {baudrate} baudios.")
            )

            while True:
                line = gps.readline().decode("ascii", errors="ignore").strip()

                if not line or "GGA" not in line:
                    continue

                gga = parse_gga(line)
                if gga is None:
                    continue

                app.root.after(0, lambda data=gga: app.update_position(data))

    except Exception as exc:
        msg = str(exc)
        app.root.after(0, lambda: app.status_var.set(f"Error serie: {msg}"))


# =========================================================
# MAIN
# =========================================================

def build_config(map_name: str, display_width: int) -> MapConfig:
    if map_name == "insia":
        return map_config_insia(display_width)
    if map_name == "campus":
        return map_config_campus_sur(display_width)
    if map_name == "vicalvaro":
        return map_config_vicalvaro(display_width)
    raise ValueError(f"Mapa no soportado: {map_name}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Práctica 3 multimapa: INSIA con límites, Campus/Vicálvaro sin límites"
    )
    parser.add_argument(
        "--map",
        choices=["insia", "campus", "vicalvaro"],
        default="insia",
        help="Mapa a mostrar: insia, campus o vicalvaro",
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Puerto serie (ej: COM3)")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudios (ej: 4800)")
    parser.add_argument(
        "--display-width",
        type=int,
        default=DEFAULT_DISPLAY_WIDTH,
        help="Ancho del mapa mostrado en píxeles",
    )
    parser.add_argument(
        "--track",
        default=None,
        help="Ruta opcional al track con límites. Si no se indica, INSIA usa Mapa_INSIA2.txt",
    )
    args = parser.parse_args()

    cfg = build_config(args.map, args.display_width)

    track: list[TrackPoint] | None = None
    track_path = args.track if args.track is not None else cfg.track_path

    if cfg.has_speed_limits and track_path:
        track = load_track(track_path)

    root = tk.Tk()
    app = GPSMultiMapApp(root, cfg, track)

    thread = threading.Thread(
        target=serial_worker,
        args=(app, args.port, args.baud),
        daemon=True
    )
    thread.start()

    root.mainloop()


if __name__ == "__main__":
    main()