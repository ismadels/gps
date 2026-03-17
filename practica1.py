"""Practica 1: lectura de tramas NMEA-GGA por puerto serie y conversion a UTM.

Uso:
    python practica1.py --port
    python practica1.py --file
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None


ELLIPSOIDS = {
    "WGS84": {"name": "WGS84", "a": 6378137.0, "f": 1.0 / 298.257223563},
    "ED50": {"name": "Europeo 1950", "a": 6378388.0, "f": 1.0 / 297.0},
}

DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 4800
DEFAULT_ZONE = 30
DEFAULT_FILE = "prueba.txt"

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


@dataclass
class GGAData:
    time_utc: str
    fix_quality: int
    satellites: int
    hdop: float
    altitude_m: float
    latitude_deg: float | None
    longitude_deg: float | None


def compute_checksum(nmea_body: str) -> int:
    """Calcula checksum XOR para la parte entre '$' y '*' de una trama NMEA."""
    value = 0
    for ch in nmea_body:
        value ^= ord(ch)
    return value


def verify_checksum(sentence: str) -> bool:
    """Devuelve True si el checksum NMEA de la linea es valido."""
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
    """Convierte ddmm.mmmm (lat) o dddmm.mmmm (lon) a grados decimales."""
    if not raw_value:
        return None

    deg_len = 2 if is_latitude else 3
    if len(raw_value) < deg_len + 2:
        return None

    degrees = int(raw_value[:deg_len])
    minutes = float(raw_value[deg_len:])
    return degrees + minutes / 60.0


def parse_gga(sentence: str) -> GGAData | None:
    """Parsea una trama GGA valida y devuelve sus campos principales."""
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

    fix_quality = int(fields[6]) if fields[6] else 0
    satellites = int(fields[7]) if fields[7] else 0
    hdop = float(fields[8]) if fields[8] else 0.0
    altitude_m = float(fields[9]) if fields[9] else 0.0

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


def get_ellipsoid_parameters(name: str) -> dict[str, float | str]:
    """Obtiene parametros del elipsoide."""
    ell = ELLIPSOIDS[name]
    a = float(ell["a"])
    f = float(ell["f"])
    b = a * (1.0 - f)

    # Primera excentricidad al cuadrado: e^2 = (a^2 - b^2)/a^2
    first_e2 = (a * a - b * b) / (a * a)

    # Segunda excentricidad al cuadrado: e'^2 = (a^2 - b^2)/b^2
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


def geo_to_utm(latitude_deg: float, longitude_deg: float, ellipsoid_name: str, zone: int) -> tuple[float, float, str]:
    """Convierte coordenadas geograficas a UTM con formulas de Gauss-Kruger/TM."""
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
        + n
        * tan_phi
        * (
            a_term**2 / 2.0
            + (5.0 - t + 9.0 * c_term + 4.0 * c_term**2) * a_term**4 / 24.0
            + (61.0 - 58.0 * t + t**2 + 600.0 * c_term - 330.0 * ep2) * a_term**6 / 720.0
        )
    )

    if latitude_deg < 0:
        northing += 10000000.0

    hemisphere = "N" if latitude_deg >= 0 else "S"
    return easting, northing, hemisphere


def print_header(ellipsoid_name: str, zone: int) -> None:
    ell = get_ellipsoid_parameters(ellipsoid_name)
    print("=" * 70)
    print("Practica 1: lectura GGA por puerto serie y conversion a UTM")
    print("=" * 70)
    print(f"Elipsoide: {ell['name']} ({ellipsoid_name})")
    print(f"a      = {float(ell['a']):.3f}")
    print(f"b      = {float(ell['b']):.3f}")
    print(f"f      = {float(ell['f']):.10f}")
    print(f"e^2    = {float(ell['first_e2']):.14f}")
    print(f"e'^2   = {float(ell['second_e2']):.14f}")
    print(f"c      = {float(ell['c']):.3f}")
    print(f"Huso UTM: {zone}")
    print("-" * 70)


def print_solution(gga: GGAData, ellipsoid_name: str, zone: int) -> None:
    print(f"Hora UTC:      {gga.time_utc}")
    print(f"Calidad fix:   {gga.fix_quality} ({FIX_QUALITY.get(gga.fix_quality, 'N/A')})")
    print(f"Satelites:     {gga.satellites}")
    print(f"HDOP:          {gga.hdop}")
    print(f"Altitud:       {gga.altitude_m} m")

    if gga.latitude_deg is None or gga.longitude_deg is None:
        print("Sin fix valido: no hay coordenadas geograficas para convertir.")
        print("-" * 70)
        return

    easting, northing, hemisphere = geo_to_utm(gga.latitude_deg, gga.longitude_deg, ellipsoid_name, zone)
    print(f"Latitud:       {gga.latitude_deg:.6f} grados")
    print(f"Longitud:      {gga.longitude_deg:.6f} grados")
    print(f"UTM Easting:   {easting:.3f} m")
    print(f"UTM Northing:  {northing:.3f} m")
    print(f"Zona UTM:      {zone}{hemisphere}")
    print("-" * 70)


def print_solution_both_ellipsoids(gga: GGAData, zone: int) -> None:
    """Imprime coordenadas geograficas y UTM para WGS84 y ED50."""
    print(f"Hora UTC:      {gga.time_utc}")
    print(f"Calidad fix:   {gga.fix_quality} ({FIX_QUALITY.get(gga.fix_quality, 'N/A')})")
    print(f"Satelites:     {gga.satellites}")
    print(f"HDOP:          {gga.hdop}")
    print(f"Altitud:       {gga.altitude_m} m")

    if gga.latitude_deg is None or gga.longitude_deg is None:
        print("Sin fix valido: no hay coordenadas geograficas para convertir.")
        print("-" * 70)
        return

    print(f"Latitud:       {gga.latitude_deg:.6f} grados")
    print(f"Longitud:      {gga.longitude_deg:.6f} grados")

    for ellipsoid_name in ("WGS84", "ED50"):
        easting, northing, hemisphere = geo_to_utm(gga.latitude_deg, gga.longitude_deg, ellipsoid_name, zone)
        print(f"{ellipsoid_name} Easting:  {easting:.3f} m")
        print(f"{ellipsoid_name} Northing: {northing:.3f} m")
        print(f"{ellipsoid_name} Zona:     {zone}{hemisphere}")

    print("-" * 70)


def run_serial(port: str, baudrate: int, zone: int) -> None:
    if serial is None:
        print("ERROR: falta pyserial. Instalar con: pip install pyserial")
        return

    print("=" * 70)
    print("Practica 1: lectura GGA por puerto serie y conversion a UTM")
    print("=" * 70)
    print(f"Puerto: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Huso UTM: {zone}")
    print("Elipsoides: WGS84 y ED50")
    print("-" * 70)
    print(f"Conectando a {port} a {baudrate} baudios. Ctrl+C para salir.")

    try:
        with serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        ) as gps:
            print(f"Puerto abierto: {gps.name}")
            print("-" * 70)

            while True:
                line = gps.readline().decode("ascii", errors="ignore").strip()
                if not line or "GGA" not in line:
                    continue

                gga = parse_gga(line)
                if gga is None:
                    continue

                print_solution_both_ellipsoids(gga, zone)

    except serial.SerialException as exc:
        print(f"Error de puerto serie: {exc}")
    except KeyboardInterrupt:
        print("Lectura finalizada por el usuario.")


def run_file(file_path: str, zone: int) -> None:
    """Lee tramas GGA desde archivo y muestra UTM para ambos elipsoides."""
    print("=" * 70)
    print("Practica 1: lectura GGA desde archivo y conversion a UTM")
    print("=" * 70)
    print(f"Archivo: {file_path}")
    print(f"Huso UTM: {zone}")
    print("Elipsoides: WGS84 y ED50")
    print("-" * 70)

    processed = 0
    with open(file_path, "r", encoding="ascii", errors="ignore") as handle:
        for line in handle:
            line = line.strip()
            if not line or "GGA" not in line:
                continue

            gga = parse_gga(line)
            if gga is None:
                continue

            print_solution_both_ellipsoids(gga, zone)
            processed += 1

    print(f"Tramas GGA procesadas: {processed}")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Lectura de tramas GGA por puerto serie y conversion a UTM"
    )
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument("--port", action="store_true", help=f"Usa puerto serie constante ({DEFAULT_PORT})")
    source_group.add_argument("--file", action="store_true", help=f"Usa archivo constante ({DEFAULT_FILE})")
    return parser


def main() -> None:
    args = build_arg_parser().parse_args()
    if args.file:
        run_file(DEFAULT_FILE, DEFAULT_ZONE)
        return

    run_serial(DEFAULT_PORT, DEFAULT_BAUD, DEFAULT_ZONE)


if __name__ == "__main__":
    main()
