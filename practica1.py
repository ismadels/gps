
"""
Práctica 1: Lectura y adquisición de datos de un receptor GPS
- Lee tramas NMEA GGA desde un puerto serie (pyserial)
- Parsea coordenadas geográficas (latitud, longitud)
- Transforma a coordenadas UTM mediante ecuaciones de Gauss-Krüger
"""

import math
import sys

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("AVISO: pyserial no instalado. Solo disponible modo simulación.")
    print("       Instalar con: pip install pyserial\n")


# ──────────────────────────────────────────────────────────────────────────────
# Parámetros de elipsoides
# ──────────────────────────────────────────────────────────────────────────────
ELLIPSOIDS = {
    'WGS84': {
        'a': 6378137.0,            # Semieje mayor (radio ecuatorial)
        'f': 1 / 298.257223563,    # Achatamiento
        'e2': 0.00669437999013,    # Cuadrado de la primera excentricidad
    },
    'ED50': {
        'a': 6378388.0,
        'f': 1 / 297.0,
        'e2': 0.00672267,
    }
}


# ──────────────────────────────────────────────────────────────────────────────
# Verificación de checksum NMEA
# ──────────────────────────────────────────────────────────────────────────────
def compute_checksum(sentence_body):
    """Calcula el checksum XOR de la parte entre $ y *."""
    cs = 0
    for ch in sentence_body:
        cs ^= ord(ch)
    return cs


def verify_checksum(sentence):
    """Verifica el checksum de una trama NMEA."""
    sentence = sentence.strip()
    if sentence.startswith('$'):
        sentence = sentence[1:]

    if '*' not in sentence:
        return False

    body, chk = sentence.split('*', 1)
    try:
        expected = int(chk[:2], 16)
    except ValueError:
        return False

    return compute_checksum(body) == expected


# ──────────────────────────────────────────────────────────────────────────────
# Parser de tramas GGA
# ──────────────────────────────────────────────────────────────────────────────
def parse_gga(sentence):
    """
    Parsea una trama NMEA GGA y devuelve un diccionario con los datos.
    Formato: $GPGGA,hhmmss.ss,llll.lll,a,yyyyy.yyy,b,q,ss,h,alt,M,geo,M,,*cs
    """
    if not verify_checksum(sentence):
        print("  AVISO: Checksum incorrecto.")

    parts = sentence.strip().split(',')

    if len(parts) < 15 or 'GGA' not in parts[0]:
        return None

    # Hora UTC
    time_str = parts[1]
    if time_str and len(time_str) >= 6:
        utc_time = f"{time_str[0:2]}:{time_str[2:4]}:{time_str[4:]}"
    else:
        utc_time = "N/A"

    # Latitud (formato: ddmm.mmm)
    lat_str = parts[2]
    lat_dir = parts[3]
    if not lat_str or not lat_dir:
        # Sin fix: devolver datos parciales
        fix_quality = int(parts[6]) if parts[6] else 0
        num_sats = int(parts[7]) if parts[7] else 0
        return {
            'time': utc_time,
            'latitude': None,
            'longitude': None,
            'lat_dir': '',
            'lon_dir': '',
            'fix_quality': fix_quality,
            'num_satellites': num_sats,
            'hdop': float(parts[8]) if parts[8] else 0.0,
            'altitude': float(parts[9]) if parts[9] else 0.0,
            'no_fix': True,
        }
    lat_deg = int(lat_str[:2])
    lat_min = float(lat_str[2:])
    latitude = lat_deg + lat_min / 60.0
    if lat_dir == 'S':
        latitude = -latitude

    # Longitud (formato: dddmm.mmm)
    lon_str = parts[4]
    lon_dir = parts[5]
    if not lon_str or not lon_dir:
        fix_quality = int(parts[6]) if parts[6] else 0
        num_sats = int(parts[7]) if parts[7] else 0
        return {
            'time': utc_time,
            'latitude': None,
            'longitude': None,
            'lat_dir': '',
            'lon_dir': '',
            'fix_quality': fix_quality,
            'num_satellites': num_sats,
            'hdop': float(parts[8]) if parts[8] else 0.0,
            'altitude': float(parts[9]) if parts[9] else 0.0,
            'no_fix': True,
        }
    lon_deg = int(lon_str[:3])
    lon_min = float(lon_str[3:])
    longitude = lon_deg + lon_min / 60.0
    if lon_dir == 'W':
        longitude = -longitude

    # Calidad del fix
    fix_quality = int(parts[6]) if parts[6] else 0

    # Número de satélites
    num_sats = int(parts[7]) if parts[7] else 0

    # HDOP
    hdop = float(parts[8]) if parts[8] else 0.0

    # Altitud
    altitude = float(parts[9]) if parts[9] else 0.0

    return {
        'no_fix': False,
        'time': utc_time,
        'latitude': latitude,
        'longitude': longitude,
        'lat_dir': lat_dir,
        'lon_dir': lon_dir,
        'fix_quality': fix_quality,
        'num_satellites': num_sats,
        'hdop': hdop,
        'altitude': altitude,
    }


# ──────────────────────────────────────────────────────────────────────────────
# Conversión de coordenadas geográficas a UTM  (Gauss-Krüger)
# ──────────────────────────────────────────────────────────────────────────────
def geo_to_utm(latitude, longitude, ellipsoid='WGS84'):
    """
    Convierte coordenadas geográficas (lat, lon en grados decimales)
    a coordenadas UTM (Easting, Northing) usando las ecuaciones de Gauss-Krüger.

    Retorna: (easting, northing, huso, hemisferio)
    """
    ell = ELLIPSOIDS[ellipsoid]
    a = ell['a']
    e2 = ell['e2']   # Primera excentricidad al cuadrado  (e'² del enunciado)

    # Segunda excentricidad al cuadrado  (e' del enunciado)
    ep2 = e2 / (1.0 - e2)

    # Coordenadas en radianes
    phi = math.radians(latitude)
    lam = math.radians(longitude)

    # Huso UTM
    huso = int((longitude + 180.0) / 6.0) + 1

    # Meridiano central del huso (en radianes)
    lambda_0g = huso * 6 - 183          # grados
    lambda_0 = math.radians(lambda_0g)  # radianes

    # ── Variables auxiliares ──
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)

    N = a / math.sqrt(1.0 - e2 * sin_phi ** 2)
    T = tan_phi ** 2
    C = ep2 * cos_phi ** 2
    A = cos_phi * (lam - lambda_0)

    # ── Longitud del arco de meridiano (M) ──
    M = a * (
        (1.0 - e2 / 4.0 - 3.0 * e2**2 / 64.0 - 5.0 * e2**3 / 256.0) * phi
        - (3.0 * e2 / 8.0 + 3.0 * e2**2 / 32.0 + 45.0 * e2**3 / 1024.0) * math.sin(2.0 * phi)
        + (15.0 * e2**2 / 256.0 + 45.0 * e2**3 / 1024.0) * math.sin(4.0 * phi)
        - (35.0 * e2**3 / 3072.0) * math.sin(6.0 * phi)
    )

    # ── UTM Easting (X) ──
    easting = 0.9996 * N * (
        A
        + (1.0 - T + C) * A**3 / 6.0
        + (5.0 - 18.0 * T + T**2 + 72.0 * C - 58.0 * ep2) * A**5 / 120.0
    ) + 500000.0

    # ── UTM Northing (Y) ──
    northing = 0.9996 * (
        M + N * tan_phi * (
            A**2 / 2.0
            + (5.0 - T + 9.0 * C + 4.0 * C**2) * A**4 / 24.0
            + (61.0 - 58.0 * T + T**2 + 600.0 * C - 330.0 * ep2) * A**6 / 720.0
        )
    )

    # Ajuste hemisferio sur
    if latitude < 0:
        northing += 10000000.0

    hemisferio = 'N' if latitude >= 0 else 'S'

    return easting, northing, huso, hemisferio


# ──────────────────────────────────────────────────────────────────────────────
# Utilidades
# ──────────────────────────────────────────────────────────────────────────────
FIX_QUALITY = {
    0: 'Inválido',
    1: 'GPS fix (SPS)',
    2: 'DGPS fix',
    3: 'PPS fix',
    4: 'Real Time Kinematic',
    5: 'Float RTK',
    6: 'Estimado (dead reckoning)',
    7: 'Entrada manual',
    8: 'Simulación',
}


def build_gga(time_s, lat, lat_d, lon, lon_d, q, sats, hdop, alt):
    """Construye una trama GGA con checksum correcto (para simulación)."""
    body = f"GPGGA,{time_s},{lat},{lat_d},{lon},{lon_d},{q},{sats},{hdop},{alt},M,0.0,M,,"
    cs = compute_checksum(body)
    return f"${body}*{cs:02X}"


def process_sentence(sentence, ellipsoid):
    """Procesa y muestra los resultados de una trama GGA."""
    print(f"  Trama: {sentence}")

    data = parse_gga(sentence)
    if data is None:
        print("  ERROR: No se pudo parsear la trama GGA.\n")
        return

    if data.get('no_fix'):
        print(f"  Hora UTC:      {data['time']}")
        print(f"  Calidad fix:   {data['fix_quality']} ({FIX_QUALITY.get(data['fix_quality'], '?')})")
        print(f"  Satélites:     {data['num_satellites']}")
        print("  ⏳ Esperando fix GPS (sin coordenadas todavía)...\n")
        return

    if data['fix_quality'] == 0:
        print("  AVISO: Fix inválido (quality=0).")

    print(f"\n  ─── Datos GPS ───")
    print(f"  Hora UTC:      {data['time']}")
    print(f"  Calidad fix:   {data['fix_quality']} ({FIX_QUALITY.get(data['fix_quality'], '?')})")
    print(f"  Satélites:     {data['num_satellites']}")
    print(f"  HDOP:          {data['hdop']}")
    print(f"  Altitud:       {data['altitude']} m")

    print(f"\n  ─── Coordenadas Geográficas ───")
    print(f"  Latitud:       {abs(data['latitude']):.6f}° {data['lat_dir']}")
    print(f"  Longitud:      {abs(data['longitude']):.6f}° {data['lon_dir']}")

    easting, northing, huso, hemisferio = geo_to_utm(
        data['latitude'], data['longitude'], ellipsoid
    )

    print(f"\n  ─── Coordenadas UTM ({ellipsoid}) ───")
    print(f"  Huso:          {huso}{hemisferio}")
    print(f"  Easting  (X):  {easting:.3f} m")
    print(f"  Northing (Y):  {northing:.3f} m")
    print()


# ──────────────────────────────────────────────────────────────────────────────
# Modos de ejecución
# ──────────────────────────────────────────────────────────────────────────────
def modo_simulacion(ellipsoid):
    """Ejecuta la aplicación con tramas GGA de ejemplo."""
    tramas = [
        # Ejemplo del enunciado (Múnich)
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47",
        # Ciudad Univ. Madrid (aprox.)
        build_gga("140530.00", "4027.1234", "N", "00344.5678", "W", "1", "10", "0.8", "650.0"),
        # Sevilla (aprox.)
        build_gga("091200.00", "3723.0000", "N", "00558.0000", "W", "2", "12", "0.6", "12.0"),
    ]

    print("\n" + "=" * 70)
    print("  MODO SIMULACIÓN — Tramas GGA de ejemplo")
    print("=" * 70 + "\n")

    for trama in tramas:
        process_sentence(trama, ellipsoid)
        print("-" * 70)


def modo_serie(port, ellipsoid, baudrate=None):
    """Lee tramas GGA desde un puerto serie real."""
    if not SERIAL_AVAILABLE:
        print("ERROR: pyserial no está instalado. Ejecuta: pip install pyserial")
        sys.exit(1)

    if baudrate is None:
        baudrate_str = input("  Baudrate [4800]: ").strip()
        baudrate = int(baudrate_str) if baudrate_str else 4800

    print(f"\n  Conectando a {port} @ {baudrate} baud …")
    print("  Pulsa Ctrl+C para salir.\n")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=2,
        )
        print(f"  Conectado a {ser.name}\n")
        print("-" * 70)

        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            if 'GGA' in line:
                process_sentence(line, ellipsoid)
                print("-" * 70)
            else:
                print(f"  [otra trama] {line}")

    except serial.SerialException as e:
        print(f"  Error de puerto serie: {e}")
    except KeyboardInterrupt:
        print("\n  Desconectado.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()


def modo_fichero(filepath, ellipsoid):
    """Lee tramas GGA desde un fichero de log NMEA."""
    print(f"\n  Leyendo fichero: {filepath}\n")
    print("-" * 70)

    try:
        with open(filepath, 'r', encoding='ascii', errors='ignore') as f:
            for line in f:
                line = line.strip()
                if line and 'GGA' in line:
                    process_sentence(line, ellipsoid)
                    print("-" * 70)
    except FileNotFoundError:
        print(f"  ERROR: Fichero no encontrado: {filepath}")
    except OSError as e:
        print(f"  ERROR: {e}")


# ──────────────────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────────────────
def main():
    print("=" * 70)
    print("  GPS Reader — Lectura de tramas GGA y conversión a UTM")
    print("=" * 70)

    # Selección de elipsoide
    print("\n  Elipsoides disponibles: WGS84, ED50")
    ellipsoid = input("  Elipsoide [WGS84]: ").strip().upper()
    if ellipsoid not in ELLIPSOIDS:
        ellipsoid = 'WGS84'

    ell = ELLIPSOIDS[ellipsoid]
    ep2 = ell['e2'] / (1 - ell['e2'])
    b = ell['a'] * (1 - ell['f'])
    c = ell['a'] ** 2 / b

    print(f"\n  Elipsoide seleccionado: {ellipsoid}")
    print(f"    a  (semieje mayor) = {ell['a']:.3f}")
    print(f"    b  (semieje menor) = {b:.3f}")
    print(f"    f  (achatamiento)  = {ell['f']:.10f}")
    print(f"    e² (1ª excent.²)   = {ell['e2']:.14f}")
    print(f"    e'²(2ª excent.²)   = {ep2:.14f}")
    print(f"    c  (radio polar)   = {c:.3f}")

    # Selección de modo
    print("\n  Modos de entrada:")
    print("    1 — Puerto serie (GPS real)")
    print("    2 — Fichero NMEA")
    print("    3 — Simulación (tramas de ejemplo)")
    modo = input("  Selecciona modo [3]: ").strip()

    if modo == '1':
        port = input("  Puerto serie (ej: COM3, /dev/ttyUSB0): ").strip()
        modo_serie(port, ellipsoid)
    elif modo == '2':
        filepath = input("  Ruta al fichero NMEA: ").strip()
        modo_fichero(filepath, ellipsoid)
    else:
        modo_simulacion(ellipsoid)


if __name__ == '__main__':
    # Uso rápido: py prueba.py --port COM9 [--baud 4800] [--ellipsoid WGS84]
    import argparse
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--port', default=None)
    parser.add_argument('--baud', type=int, default=4800)
    parser.add_argument('--ellipsoid', default='WGS84')
    args, _ = parser.parse_known_args()

    if args.port:
        # Modo directo sin menú interactivo
        ellipsoid = args.ellipsoid.upper()
        if ellipsoid not in ELLIPSOIDS:
            ellipsoid = 'WGS84'
        ell = ELLIPSOIDS[ellipsoid]
        ep2 = ell['e2'] / (1 - ell['e2'])
        b = ell['a'] * (1 - ell['f'])
        c = ell['a'] ** 2 / b
        print("=" * 70)
        print("  GPS Reader — Lectura de tramas GGA y conversión a UTM")
        print("=" * 70)
        print(f"\n  Elipsoide: {ellipsoid}")
        print(f"    a = {ell['a']:.3f}  b = {b:.3f}  e² = {ell['e2']:.14f}")
        # Saltar menú, ir directo al puerto serie con baudrate dado
        modo_serie(args.port, ellipsoid, args.baud)
    else:
        main()
