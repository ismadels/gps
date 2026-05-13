from __future__ import annotations

import argparse
import math
import os
import subprocess
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from io import BytesIO

try:
    import serial
except ImportError:
    serial = None

try:
    import requests
except ImportError:
    requests = None

try:
    import pyttsx3
except ImportError:
    pyttsx3 = None

from PIL import Image, ImageTk


# =========================================================
# CONFIGURACIÓN GENERAL
# =========================================================

DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 4800
DEFAULT_ZONE = 30

DEFAULT_DISPLAY_WIDTH = 800
DEFAULT_PANEL_WIDTH = 560
DEFAULT_WINDOW_GEOMETRY = "1450x900"

# OpenStreetMap usa teselas de 256x256 px.
TILE_SIZE = 256
OSM_TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSM_USER_AGENT = "GPS-Practica-UPM/1.0"

# Radio de teselas alrededor de la tesela central.
# 2 => mapa compuesto de 5x5 teselas.
TILES_RADIUS = 2

# Zoom recomendado por zona. Se puede ajustar desde línea de comandos con --zoom.
ZOOM_BY_MAP = {
    "insia": 18,
    "campus": 17,
    "vicalvaro": 17,
}

# Para no saturar OSM, no descargamos mapa en cada trama si no hace falta.
MIN_MAP_UPDATE_SECONDS = 2.0
MIN_MAP_UPDATE_METERS = 8.0

VOICE_ALERT_COOLDOWN_SECONDS = 6.0
BRAKING_DROP_KMH = 3.0
BAD_HDOP_THRESHOLD = 4.0
VOICE_AVAILABLE = pyttsx3 is not None or os.name == "nt"
DESTINATION_RADIUS_METERS = 8.0

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
COLOR_TRACE_DEFAULT = "#1E88E5"


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


@dataclass
class RMCData:
    time_utc: str
    valid: bool
    speed_kmh: float | None


@dataclass(frozen=True)
class MapConfig:
    key: str
    title: str
    zone: int
    has_speed_limits: bool = False
    track_path: str | None = None


@dataclass
class TrackPoint:
    north: float
    east: float
    vmax: float


@dataclass
class OSMBackground:
    image: Image.Image
    zoom: int
    top_left_tile_x: int
    top_left_tile_y: int
    raw_size: int


# =========================================================
# CONFIGURACIÓN DE MAPAS
# =========================================================

# En la práctica 4 el mapa ya no depende de imágenes locales.
# Estos mapas solo sirven para seleccionar modo, zoom y si se cargan límites.

def map_config_insia() -> MapConfig:
    return MapConfig(
        key="insia",
        title="INSIA",
        zone=DEFAULT_ZONE,
        has_speed_limits=True,
        track_path="Mapa_INSIA2.txt",
    )


def map_config_campus_sur() -> MapConfig:
    return MapConfig(
        key="campus",
        title="Campus Sur",
        zone=DEFAULT_ZONE,
        has_speed_limits=True,
        track_path="Mapa_Campus_Sur.txt",
    )


def map_config_vicalvaro() -> MapConfig:
    return MapConfig(
        key="vicalvaro",
        title="Vicálvaro",
        zone=DEFAULT_ZONE,
        has_speed_limits=False,
        track_path=None,
    )


def build_config(map_name: str) -> MapConfig:
    if map_name == "insia":
        return map_config_insia()
    if map_name == "campus":
        return map_config_campus_sur()
    if map_name == "vicalvaro":
        return map_config_vicalvaro()
    raise ValueError(f"Mapa no soportado: {map_name}")


# =========================================================
# CARGA DEL MAPA ELECTRÓNICO DE LÍMITES
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
                    f"Línea {line_num} inválida en {path}: se esperaban 3 valores: UTM_Norte UTM_Este Vmax."
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


def parse_rmc(sentence: str) -> RMCData | None:
    if not verify_checksum(sentence):
        return None

    line = sentence.strip()
    body = line[1: line.find("*")]
    fields = body.split(",")

    if len(fields) < 12:
        return None

    if fields[0] not in ("GPRMC", "GNRMC"):
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

    valid = fields[2] == "A"
    speed_kmh = None

    if valid and fields[7]:
        try:
            speed_knots = float(fields[7])
            speed_kmh = speed_knots * 1.852
        except ValueError:
            speed_kmh = None

    return RMCData(time_utc=time_utc, valid=valid, speed_kmh=speed_kmh)


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
# OPENSTREETMAP: LAT/LON <-> TESELAS
# =========================================================

def latlon_to_tile_fraction(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    """Devuelve coordenadas de tesela fraccionales XYZ para una lat/lon."""
    lat_rad = math.radians(lat)
    n = 2 ** zoom

    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) / 2.0 * n

    return x, y


def tile_fraction_to_latlon(x_tile: float, y_tile: float, zoom: int) -> tuple[float, float]:
    n = 2 ** zoom
    lon = x_tile / n * 360.0 - 180.0
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * y_tile / n)))
    lat = math.degrees(lat_rad)
    return lat, lon


def download_osm_tile(z: int, x: int, y: int) -> Image.Image:
    if requests is None:
        raise RuntimeError("Falta requests. Instala con: python -m pip install requests")

    max_tile = 2 ** z
    if x < 0 or y < 0 or x >= max_tile or y >= max_tile:
        return Image.new("RGB", (TILE_SIZE, TILE_SIZE), "white")

    url = OSM_TILE_URL.format(z=z, x=x, y=y)
    headers = {"User-Agent": OSM_USER_AGENT}

    response = requests.get(url, headers=headers, timeout=8)
    response.raise_for_status()

    return Image.open(BytesIO(response.content)).convert("RGB")


def build_osm_background(center_lat: float, center_lon: float, zoom: int) -> OSMBackground:
    """Descarga un mosaico de tiles OSM centrado aproximadamente en la posición GPS."""
    center_x_tile, center_y_tile = latlon_to_tile_fraction(center_lat, center_lon, zoom)
    center_x_int = int(math.floor(center_x_tile))
    center_y_int = int(math.floor(center_y_tile))

    top_left_tile_x = center_x_int - TILES_RADIUS
    top_left_tile_y = center_y_int - TILES_RADIUS

    num_tiles = TILES_RADIUS * 2 + 1
    raw_size = num_tiles * TILE_SIZE
    full_image = Image.new("RGB", (raw_size, raw_size), "white")

    for dx in range(num_tiles):
        for dy in range(num_tiles):
            tile_x = top_left_tile_x + dx
            tile_y = top_left_tile_y + dy

            try:
                tile = download_osm_tile(zoom, tile_x, tile_y)
            except Exception:
                tile = Image.new("RGB", (TILE_SIZE, TILE_SIZE), "#eeeeee")

            full_image.paste(tile, (dx * TILE_SIZE, dy * TILE_SIZE))

    return OSMBackground(
        image=full_image,
        zoom=zoom,
        top_left_tile_x=top_left_tile_x,
        top_left_tile_y=top_left_tile_y,
        raw_size=raw_size,
    )


def latlon_to_raw_osm_pixel(lat: float, lon: float, bg: OSMBackground) -> tuple[float, float]:
    x_tile, y_tile = latlon_to_tile_fraction(lat, lon, bg.zoom)
    x_raw = (x_tile - bg.top_left_tile_x) * TILE_SIZE
    y_raw = (y_tile - bg.top_left_tile_y) * TILE_SIZE
    return x_raw, y_raw


def raw_osm_pixel_to_latlon(x_raw: float, y_raw: float, bg: OSMBackground) -> tuple[float, float]:
    x_tile = bg.top_left_tile_x + x_raw / TILE_SIZE
    y_tile = bg.top_left_tile_y + y_raw / TILE_SIZE
    return tile_fraction_to_latlon(x_tile, y_tile, bg.zoom)


def raw_to_display_pixel(x_raw: float, y_raw: float, raw_size: int, display_size: int) -> tuple[int, int]:
    scale = display_size / raw_size
    return int(round(x_raw * scale)), int(round(y_raw * scale))


def display_to_raw_pixel(x_display: float, y_display: float, raw_size: int, display_size: int) -> tuple[float, float]:
    scale = raw_size / display_size
    return x_display * scale, y_display * scale


# =========================================================
# CÁLCULOS DE VELOCIDAD Y LÍMITES
# =========================================================

def euclidean_distance(e1: float, n1: float, e2: float, n2: float) -> float:
    return math.sqrt((e2 - e1) ** 2 + (n2 - n1) ** 2)

def gga_time_to_seconds(time_utc: str) -> float | None:
    try:
        hh = int(time_utc[0:2])
        mm = int(time_utc[3:5])
        ss = float(time_utc[6:])
        return hh * 3600 + mm * 60 + ss
    except Exception:
        return None

def compute_speed_kmh(
    prev_e: float,
    prev_n: float,
    prev_t: float,
    curr_e: float,
    curr_n: float,
    curr_t: float
) -> float:
    dt = curr_t - prev_t

    # Si cambia de día UTC
    if dt < 0:
        dt += 24 * 3600

    if dt <= 0:
        return 0.0

    d = euclidean_distance(prev_e, prev_n, curr_e, curr_n)

    # Ruido típico del GPS: si el salto es pequeño, lo consideramos error
    if d < 0.35:
        return 0.0

    speed = (d / dt) * 3.6

    # Filtro de picos irreales para la práctica andando
    if speed > 80.0:
        return 0.0

    return speed


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


def speak_text(text: str) -> None:
    if pyttsx3 is not None:
        try:
            engine = pyttsx3.init()
            engine.setProperty("rate", 165)
            engine.say(text)
            engine.runAndWait()
            engine.stop()
            return
        except Exception:
            pass

    if os.name == "nt":
        escaped_text = text.replace("'", "''")
        command = (
            "Add-Type -AssemblyName System.Speech; "
            "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer; "
            "$s.Rate = 0; "
            f"$s.Speak('{escaped_text}')"
        )
        try:
            subprocess.run(
                ["powershell", "-NoProfile", "-Command", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=8,
                check=False,
            )
        except Exception:
            pass


# =========================================================
# INTERFAZ
# =========================================================

class GPSOSMApp:
    def __init__(self, root: tk.Tk, cfg: MapConfig, track: list[TrackPoint] | None, zoom: int):
        self.root = root
        self.cfg = cfg
        self.track = track
        self.zoom = zoom

        self.display_width = DEFAULT_DISPLAY_WIDTH
        self.display_height = DEFAULT_DISPLAY_WIDTH

        self.root.title(f"{cfg.title} - Práctica 4 OSM GPS")
        self.root.geometry(DEFAULT_WINDOW_GEOMETRY)
        self.root.minsize(1300, 820)

        self.prev_e: float | None = None
        self.prev_n: float | None = None
        self.prev_t: float | None = None

        self.prev_track_idx: int | None = None
        self.direction_sign: int = 1
        self.speed_history: list[float] = []
        self.previous_speed_kmh: float | None = None
        self.last_voice_alert: str | None = None
        self.last_voice_time: float = 0.0
        self.destination_select_mode = False
        self.destination_lat: float | None = None
        self.destination_lon: float | None = None
        self.destination_e: float | None = None
        self.destination_n: float | None = None
        self.destination_reached = False

        self.current_bg: OSMBackground | None = None
        self.current_map_tk: ImageTk.PhotoImage | None = None
        self.last_map_update_t: float = 0.0
        self.last_map_center_e: float | None = None
        self.last_map_center_n: float | None = None

        self.position_history: list[tuple[float, float, str]] = []

        main_frame = tk.Frame(root, bg="#f0f0f0")
        main_frame.pack(fill="both", expand=True)

        map_frame = tk.Frame(main_frame, padx=10, pady=10, bg="#f0f0f0")
        map_frame.pack(side=tk.LEFT, fill="both", expand=True)

        panel_frame = tk.Frame(main_frame, padx=14, pady=10, bg="#f7f7f7", width=DEFAULT_PANEL_WIDTH)
        panel_frame.pack(side=tk.RIGHT, fill=tk.Y)
        panel_frame.pack_propagate(False)

        self.canvas = tk.Canvas(
            map_frame,
            width=self.display_width,
            height=self.display_height,
            bg="white",
            highlightthickness=1,
            highlightbackground="black"
        )
        self.canvas.pack(anchor="nw")
        self.map_image_id = self.canvas.create_image(0, 0, anchor="nw")
        self.canvas.bind("<Button-1>", self.on_map_click)

        self.gps_radius = 8

        tk.Label(
            panel_frame,
            text=f"PRÁCTICA 4 - OSM - {cfg.title.upper()}",
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
        self.voice_enabled_var = tk.BooleanVar(value=VOICE_AVAILABLE)
        self.voice_status_var = tk.StringVar(
            value="Avisos de voz: activos" if VOICE_AVAILABLE else "Avisos de voz: no disponibles"
        )
        self.destination_var = tk.StringVar(value="Destino: no seleccionado")
        self.map_mode_var = tk.StringVar(
            value=f"Modo: OSM + límites {self.cfg.title}" if self.cfg.has_speed_limits else "Modo: OSM sin límites de velocidad"
        )

        tk.Label(panel_frame, textvariable=self.map_mode_var, font=("Arial", 13, "bold"), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=2, fill="x")
        tk.Label(panel_frame, textvariable=self.limit_var, font=("Arial", 14, "bold"), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")
        tk.Label(panel_frame, textvariable=self.alert_var, font=("Arial", 13), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")
        tk.Label(panel_frame, textvariable=self.direction_var, font=("Arial", 12), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=3, fill="x")
        tk.Checkbutton(
            panel_frame,
            text="Asistente por voz",
            variable=self.voice_enabled_var,
            state=tk.NORMAL if VOICE_AVAILABLE else tk.DISABLED,
            bg="#f7f7f7",
            font=("Arial", 12, "bold"),
            anchor="center"
        ).pack(pady=(8, 2), fill="x")
        tk.Label(panel_frame, textvariable=self.voice_status_var, font=("Arial", 11), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=2, fill="x")
        tk.Button(
            panel_frame,
            text="Seleccionar destino en el mapa",
            command=self.enable_destination_selection,
            font=("Arial", 12, "bold"),
            bg="#e8f0fe"
        ).pack(pady=(8, 2), fill="x")
        tk.Button(
            panel_frame,
            text="Borrar destino",
            command=self.clear_destination,
            font=("Arial", 11),
            bg="#eeeeee"
        ).pack(pady=2, fill="x")
        tk.Label(panel_frame, textvariable=self.destination_var, font=("Arial", 11), bg="#f7f7f7", wraplength=470, justify="center").pack(pady=2, fill="x")

        self.speed_canvas = tk.Canvas(
            panel_frame,
            width=440,
            height=175,
            bg="white",
            highlightthickness=1,
            highlightbackground="black"
        )
        self.speed_canvas.pack(pady=12)

        self.zone_green = self.speed_canvas.create_rectangle(10, 58, 150, 128, fill=COLOR_NEUTRAL, outline="black")
        self.zone_yellow = self.speed_canvas.create_rectangle(150, 58, 295, 128, fill=COLOR_NEUTRAL, outline="black")
        self.zone_red = self.speed_canvas.create_rectangle(295, 58, 430, 128, fill=COLOR_NEUTRAL, outline="black")

        self.speed_canvas.create_text(80, 22, text="V < límite-10%", font=("Arial", 12))
        self.speed_canvas.create_text(222, 22, text="±10% límite", font=("Arial", 12))
        self.speed_canvas.create_text(362, 22, text="V > límite+10%", font=("Arial", 12))
        self.speed_canvas.create_text(220, 42, text="Velocidad actual", font=("Arial", 12, "bold"))
        self.speed_text = self.speed_canvas.create_text(220, 93, text="0.0 km/h", font=("Arial", 24, "bold"))

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
        self.zoom_var = tk.StringVar(value=f"Zoom OSM: {self.zoom}")

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
            self.zoom_var,
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

    def should_update_map(self, easting: float, northing: float) -> bool:
        if self.current_bg is None:
            return True

        now = time.time()
        if now - self.last_map_update_t < MIN_MAP_UPDATE_SECONDS:
            return False

        if self.last_map_center_e is None or self.last_map_center_n is None:
            return True

        moved = euclidean_distance(self.last_map_center_e, self.last_map_center_n, easting, northing)
        return moved >= MIN_MAP_UPDATE_METERS

    def update_osm_background(self, lat: float, lon: float, easting: float, northing: float) -> None:
        if not self.should_update_map(easting, northing):
            return

        try:
            bg = build_osm_background(lat, lon, self.zoom)
        except Exception as exc:
            self.status_var.set(f"Error descargando mapa OSM: {exc}")
            return

        resized = bg.image.resize((self.display_width, self.display_height))
        self.current_map_tk = ImageTk.PhotoImage(resized)
        self.canvas.itemconfig(self.map_image_id, image=self.current_map_tk)

        self.current_bg = bg
        self.last_map_update_t = time.time()
        self.last_map_center_e = easting
        self.last_map_center_n = northing

    def enable_destination_selection(self) -> None:
        if self.current_bg is None:
            self.destination_var.set("Destino: espera a que cargue el mapa OSM")
            return

        self.destination_select_mode = True
        self.destination_var.set("Destino: haz clic en el mapa")

    def clear_destination(self) -> None:
        self.destination_select_mode = False
        self.destination_lat = None
        self.destination_lon = None
        self.destination_e = None
        self.destination_n = None
        self.destination_reached = False
        self.destination_var.set("Destino: no seleccionado")
        self.canvas.delete("destination")

    def on_map_click(self, event: tk.Event) -> None:
        if not self.destination_select_mode or self.current_bg is None:
            return

        x_raw, y_raw = display_to_raw_pixel(event.x, event.y, self.current_bg.raw_size, self.display_width)
        lat, lon = raw_osm_pixel_to_latlon(x_raw, y_raw, self.current_bg)
        easting, northing, _ = geo_to_utm(lat, lon, "WGS84", self.cfg.zone)

        self.destination_lat = lat
        self.destination_lon = lon
        self.destination_e = easting
        self.destination_n = northing
        self.destination_reached = False
        self.destination_select_mode = False
        self.destination_var.set(
            f"Destino: lat={lat:.6f}, lon={lon:.6f}, radio={DESTINATION_RADIUS_METERS:.0f} m"
        )
        self.draw_destination_marker()

    def draw_destination_marker(self) -> None:
        self.canvas.delete("destination")

        if self.current_bg is None or self.destination_lat is None or self.destination_lon is None:
            return

        x_raw, y_raw = latlon_to_raw_osm_pixel(self.destination_lat, self.destination_lon, self.current_bg)
        x, y = raw_to_display_pixel(x_raw, y_raw, self.current_bg.raw_size, self.display_width)

        r = 10
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill="#1E88E5", outline="white", width=3, tags="destination")
        self.canvas.create_line(x - 14, y, x + 14, y, fill="#1E88E5", width=2, tags="destination")
        self.canvas.create_line(x, y - 14, x, y + 14, fill="#1E88E5", width=2, tags="destination")
        self.canvas.create_text(x, y - 24, text="Destino", fill="#1E88E5", font=("Arial", 10, "bold"), tags="destination")

    def check_destination_arrival(self, easting: float, northing: float) -> str | None:
        if (
            self.destination_e is None
            or self.destination_n is None
            or self.destination_reached
        ):
            return None

        distance = euclidean_distance(easting, northing, self.destination_e, self.destination_n)
        self.destination_var.set(f"Destino: distancia {distance:.1f} m")

        if distance <= DESTINATION_RADIUS_METERS:
            self.destination_reached = True
            self.destination_var.set("Destino: has llegado")
            return "Has llegado a tu destino"

        return None

    def draw_trace_and_marker(self, current_lat: float, current_lon: float) -> None:
        self.canvas.delete("trace")
        self.canvas.delete("gps")

        if self.current_bg is None:
            return

        screen_points: list[tuple[int, int, str]] = []

        for lat, lon, color in self.position_history:
            x_raw, y_raw = latlon_to_raw_osm_pixel(lat, lon, self.current_bg)
            x, y = raw_to_display_pixel(x_raw, y_raw, self.current_bg.raw_size, self.display_width)
            screen_points.append((x, y, color))

        for i in range(1, len(screen_points)):
            x1, y1, _ = screen_points[i - 1]
            x2, y2, color = screen_points[i]
            self.canvas.create_line(x1, y1, x2, y2, fill=color, width=3, tags="trace")

        x_raw, y_raw = latlon_to_raw_osm_pixel(current_lat, current_lon, self.current_bg)
        x, y = raw_to_display_pixel(x_raw, y_raw, self.current_bg.raw_size, self.display_width)
        self.pix_var.set(f"Píxel pantalla: x={x}, y={y}")

        r = self.gps_radius
        self.canvas.create_oval(x - r, y - r, x + r, y + r, fill="red", outline="black", width=2, tags="gps")
        self.canvas.create_text(x, y - 16, text="GPS", fill="black", font=("Arial", 10, "bold"), tags="gps")
        self.draw_destination_marker()

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

    def choose_voice_alert(
        self,
        state: str,
        speed_kmh: float,
        limit_kmh: float | None,
        hdop: float,
    ) -> str | None:
        if hdop >= BAD_HDOP_THRESHOLD:
            return "GPS con mala precision"

        if state == "EXCESO":
            return "Estas superando el limite"

        if state == "CERCA":
            return "Cuidado, estas cerca del limite"

        if self.previous_speed_kmh is not None:
            speed_drop = self.previous_speed_kmh - speed_kmh
            if speed_drop >= BRAKING_DROP_KMH and speed_kmh > 1.0:
                return "Te estas frenando"

        return None

    def play_voice_alert(self, text: str | None) -> None:
        if text is None:
            return

        if not VOICE_AVAILABLE:
            self.voice_status_var.set("Avisos de voz: no disponibles")
            return

        if not self.voice_enabled_var.get():
            self.voice_status_var.set("Avisos de voz: desactivados")
            return

        now = time.time()
        if text == self.last_voice_alert and now - self.last_voice_time < VOICE_ALERT_COOLDOWN_SECONDS:
            return

        self.last_voice_alert = text
        self.last_voice_time = now
        self.voice_status_var.set(f"Aviso de voz: {text}")
        threading.Thread(target=speak_text, args=(text,), daemon=True).start()

    def update_position(self, gga: GGAData, rmc_speed_kmh: float | None = None) -> None:
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

        gps_seconds = gga_time_to_seconds(gga.time_utc)

        if gps_seconds is not None:
            now_t = gps_seconds
        else:
            now_t = time.time()
        raw_speed_kmh = rmc_speed_kmh if rmc_speed_kmh is not None else 0.0
        move_e = 0.0
        move_n = 0.0

        if self.prev_e is not None and self.prev_n is not None and self.prev_t is not None:
            if rmc_speed_kmh is None:
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

        state, current_color = self.update_speedometer(speed_kmh, limit_kmh)
        destination_alert = self.check_destination_arrival(easting, northing)
        voice_alert = destination_alert or self.choose_voice_alert(state, speed_kmh, limit_kmh, gga.hdop)
        self.play_voice_alert(voice_alert)
        self.previous_speed_kmh = speed_kmh

        # Mapa dinámico OSM centrado en la posición actual.
        self.update_osm_background(gga.latitude_deg, gga.longitude_deg, easting, northing)

        if self.current_bg is not None:
            self.position_history.append((gga.latitude_deg, gga.longitude_deg, current_color))
            if len(self.position_history) > 200:
                self.position_history.pop(0)

            self.draw_trace_and_marker(gga.latitude_deg, gga.longitude_deg)
            self.status_var.set("GPS recibido. Mapa OSM actualizado.")

        self.prev_e = easting
        self.prev_n = northing
        self.prev_t = now_t


# =========================================================
# LECTURA SERIE
# =========================================================

def serial_worker(app: GPSOSMApp, port: str, baudrate: int) -> None:
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

            latest_rmc_speed_kmh: float | None = None

            while True:
                line = gps.readline().decode("ascii", errors="ignore").strip()

                if not line:
                    continue

                if "RMC" in line:
                    rmc = parse_rmc(line)
                    if rmc is not None and rmc.speed_kmh is not None:
                        latest_rmc_speed_kmh = rmc.speed_kmh
                    continue

                if "GGA" in line:
                    gga = parse_gga(line)
                    if gga is None:
                        continue

                    app.root.after(
                        0,
                        lambda data=gga, speed=latest_rmc_speed_kmh: app.update_position(data, speed)
                    )

    except Exception as exc:
        msg = str(exc)
        app.root.after(0, lambda: app.status_var.set(f"Error serie: {msg}"))


# =========================================================
# MAIN
# =========================================================

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Práctica 4: servicio georreferenciado con OpenStreetMap"
    )
    parser.add_argument(
        "--map",
        choices=["insia", "campus", "vicalvaro"],
        default="insia",
        help="Mapa/modo a mostrar: insia, campus o vicalvaro",
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Puerto serie, por ejemplo COM3")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudios, por ejemplo 4800")
    parser.add_argument(
        "--zoom",
        type=int,
        default=None,
        help="Zoom OSM. Si no se indica, se usa el recomendado para cada mapa.",
    )
    parser.add_argument(
        "--track",
        default=None,
        help="Ruta opcional al track con límites. Si no se indica, INSIA usa Mapa_INSIA2.txt",
    )
    args = parser.parse_args()

    cfg = build_config(args.map)
    zoom = args.zoom if args.zoom is not None else ZOOM_BY_MAP.get(cfg.key, 17)

    track: list[TrackPoint] | None = None
    track_path = args.track if args.track is not None else cfg.track_path

    if cfg.has_speed_limits and track_path:
        track = load_track(track_path)

    root = tk.Tk()
    app = GPSOSMApp(root, cfg, track, zoom)

    thread = threading.Thread(
        target=serial_worker,
        args=(app, args.port, args.baud),
        daemon=True,
    )
    thread.start()

    root.mainloop()


if __name__ == "__main__":
    main()
