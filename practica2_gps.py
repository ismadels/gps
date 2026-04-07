from __future__ import annotations

import argparse
import math
import threading
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
DEFAULT_DISPLAY_WIDTH = 1000

ELLIPSOIDS = {
    "WGS84": {"name": "WGS84", "a": 6378137.0, "f": 1.0 / 298.257223563},
    "ED50": {"name": "Europeo 1950", "a": 6378388.0, "f": 1.0 / 297.0},
}

FIX_QUALITY = {
    0: "invalido",
    1: "GPS fix (SPS)",
    2: "DGPS fix",
    3: "PPS fix",
    4: "RTK",
    5: "Float RTK",
    6: "estimado",
    7: "manual",
    8: "simulacion",
}


# =========================================================
# DATOS GGA
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


# =========================================================
# CONFIGURACIÓN DE MAPAS
# =========================================================

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


def map_config_insia(display_width: int) -> MapConfig:
    """
    INSIA con UTM fijas medidas por vosotros.
    TL = esquina superior izquierda
    BR = esquina inferior derecha
    """
    return MapConfig(
        key="insia",
        title="INSIA",
        image_path="gps_insia.jpg",
        orig_width=1481,
        orig_height=1012,
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=446188.92,
        n_tl=4470994.44,
        e_br=446543.65,
        n_br=4470741.85,
    )


def map_config_campus_sur(display_width: int) -> MapConfig:
    """
    Campus Sur con UTM fijas.
    CAMBIA estos valores por los de TU imagen final.
    """
    return MapConfig(
        key="campus",
        title="Campus Sur",
        image_path="campus_sur.jpg",
        orig_width=1481,  
        orig_height=1012,    
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=446470.66,   
        n_tl=4471374.71,   
        e_br=447074.56,  
        n_br=4470944.35,   
    )


def map_config_vicalvaro(display_width: int) -> MapConfig:
    return MapConfig(
        key="vicalvaro",
        title="Vicálvaro",
        image_path="vicalvaro.jpg",
        orig_width=1481,  
        orig_height=1012,   
        display_width=display_width,
        zone=DEFAULT_ZONE,
        e_tl=447623.30,         
        n_tl=4473621.36,          
        e_br=448850.08,          
        n_br=4472749.13,          
    )


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

    n = a / math.sqrt(1.0 - e2 * sin_phi**2)
    t = tan_phi**2
    c_term = ep2 * cos_phi**2
    a_term = cos_phi * (lam - lambda_0)

    m = a * (
        (1.0 - e2 / 4.0 - 3.0 * e2**2 / 64.0 - 5.0 * e2**3 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e2**2 / 32.0 + 45.0 * e2**3 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * e2**2 / 256.0 + 45.0 * e2**3 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e2**3 / 3072.0) * math.sin(6.0 * phi)
    )

    easting = 0.9996 * n * (
        a_term
        + (1.0 - t + c_term) * a_term**3 / 6.0
        + (5.0 - 18.0 * t + t**2 + 72.0 * c_term - 58.0 * ep2) * a_term**5 / 120.0
    ) + 500000.0

    northing = 0.9996 * (
        m
        + n * tan_phi * (
            a_term**2 / 2.0
            + (5.0 - t + 9.0 * c_term + 4.0 * c_term**2) * a_term**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c_term - 330.0 * ep2) * a_term**6 / 720.0
        )
    )

    if latitude_deg < 0:
        northing += 10000000.0

    hemisphere = "N" if latitude_deg >= 0 else "S"
    return easting, northing, hemisphere


# =========================================================
# UTM -> PÍXEL
# =========================================================

def utm_to_pixel_original(cfg: MapConfig, E: float, N: float) -> tuple[int, int]:
    x = ((E - cfg.e_tl) / (cfg.e_br - cfg.e_tl)) * (cfg.orig_width - 1)
    y = ((cfg.n_tl - N) / (cfg.n_tl - cfg.n_br)) * (cfg.orig_height - 1)
    return int(round(x)), int(round(y))


def original_pixel_to_display(cfg: MapConfig, x: int, y: int) -> tuple[int, int]:
    display_height = int(cfg.orig_height * cfg.display_width / cfg.orig_width)
    x_disp = int(round(x * cfg.display_width / cfg.orig_width))
    y_disp = int(round(y * display_height / cfg.orig_height))
    return x_disp, y_disp


def inside_original_image(cfg: MapConfig, x: int, y: int) -> bool:
    return 0 <= x < cfg.orig_width and 0 <= y < cfg.orig_height


# =========================================================
# INTERFAZ
# =========================================================

class GPSMapApp:
    def __init__(self, root: tk.Tk, cfg: MapConfig):
        self.root = root
        self.cfg = cfg
        self.display_height = int(cfg.orig_height * cfg.display_width / cfg.orig_width)
        self.root.title(f"Práctica 2 - {cfg.title} con GPS")

        main_frame = tk.Frame(root)
        main_frame.pack(fill="both", expand=True)

        map_frame = tk.Frame(main_frame)
        map_frame.pack(side=tk.LEFT, padx=10, pady=10)

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
        self.canvas.pack()

        self.canvas.create_image(0, 0, anchor="nw", image=self.base_image_tk)

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

        self.last_x_disp = None
        self.last_y_disp = None

        info_frame = tk.Frame(main_frame, padx=10, pady=10)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y)

        self.status_var = tk.StringVar(value="Esperando datos GPS...")
        self.utc_var = tk.StringVar(value="Hora UTC: -")
        self.fix_var = tk.StringVar(value="Fix: -")
        self.sat_var = tk.StringVar(value="Satélites: -")
        self.hdop_var = tk.StringVar(value="HDOP: -")
        self.alt_var = tk.StringVar(value="Altitud: -")
        self.lat_var = tk.StringVar(value="Latitud: -")
        self.lon_var = tk.StringVar(value="Longitud: -")
        self.utm_var = tk.StringVar(value="UTM: -")
        self.pix_var = tk.StringVar(value="Píxel original: -")
        self.pix_disp_var = tk.StringVar(value="Píxel pantalla: -")
        self.size_var = tk.StringVar(
            value=f"Imagen mostrada: {cfg.display_width} x {self.display_height}"
        )

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
            self.pix_disp_var,
            self.size_var,
        ]

        for var in vars_to_show:
            tk.Label(
                info_frame,
                textvariable=var,
                anchor="w",
                justify="left",
                wraplength=320
            ).pack(fill="x", pady=3)

    def update_position(self, gga: GGAData) -> None:
        self.utc_var.set(f"Hora UTC: {gga.time_utc}")
        self.fix_var.set(f"Fix: {gga.fix_quality} ({FIX_QUALITY.get(gga.fix_quality, 'N/A')})")
        self.sat_var.set(f"Satélites: {gga.satellites}")
        self.hdop_var.set(f"HDOP: {gga.hdop}")
        self.alt_var.set(f"Altitud: {gga.altitude_m} m")

        if gga.latitude_deg is None or gga.longitude_deg is None:
            self.status_var.set("Sin fix válido.")
            self.lat_var.set("Latitud: -")
            self.lon_var.set("Longitud: -")
            self.utm_var.set("UTM: -")
            self.pix_var.set("Píxel original: -")
            self.pix_disp_var.set("Píxel pantalla: -")
            return

        self.lat_var.set(f"Latitud: {gga.latitude_deg:.6f}")
        self.lon_var.set(f"Longitud: {gga.longitude_deg:.6f}")

        E, N, hemisphere = geo_to_utm(
            gga.latitude_deg,
            gga.longitude_deg,
            "WGS84",
            self.cfg.zone
        )

        self.utm_var.set(f"UTM: E={E:.2f}, N={N:.2f}, Zona={self.cfg.zone}{hemisphere}")

        x_orig, y_orig = utm_to_pixel_original(self.cfg, E, N)
        self.pix_var.set(f"Píxel original: x={x_orig}, y={y_orig}")

        if inside_original_image(self.cfg, x_orig, y_orig):
            x_disp, y_disp = original_pixel_to_display(self.cfg, x_orig, y_orig)
            self.pix_disp_var.set(f"Píxel pantalla: x={x_disp}, y={y_disp}")

            if self.last_x_disp is not None and self.last_y_disp is not None:
                self.canvas.create_line(
                    self.last_x_disp,
                    self.last_y_disp,
                    x_disp,
                    y_disp,
                    fill="blue",
                    width=2
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
            self.canvas.coords(self.gps_label, x_disp, y_disp - 15)

            self.status_var.set("GPS dentro de la imagen.")
        else:
            self.status_var.set("La posición GPS está fuera de la zona georreferenciada.")
            self.pix_disp_var.set("Píxel pantalla: fuera de imagen")
            self.canvas.coords(self.gps_marker, -100, -100, -100, -100)
            self.canvas.coords(self.gps_label, -100, -100)


# =========================================================
# LECTURA SERIE
# =========================================================

def serial_worker(app: GPSMapApp, port: str, baudrate: int) -> None:
    if serial is None:
        app.root.after(
            0,
            lambda: app.status_var.set(
                "ERROR: falta pyserial en ESTE entorno. Instala con: python -m pip install pyserial"
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
        app.root.after(0, lambda: app.status_var.set(f"Error serie: {exc}"))


# =========================================================
# MAIN
# =========================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="Práctica 2 - mapa electrónico con GPS")
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
    args = parser.parse_args()

    if args.map == "insia":
        cfg = map_config_insia(args.display_width)
    elif args.map == "campus":
        cfg = map_config_campus_sur(args.display_width)
    else:
        cfg = map_config_vicalvaro(args.display_width)

    root = tk.Tk()
    app = GPSMapApp(root, cfg)

    thread = threading.Thread(
        target=serial_worker,
        args=(app, args.port, args.baud),
        daemon=True
    )
    thread.start()

    root.mainloop()


if __name__ == "__main__":
    main()
