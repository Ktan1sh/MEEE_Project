#!/usr/bin/env python3
"""
ESP32 Drone IMU Logger
Parses structured serial output from RX3.ino and logs to CSV.
One row per packet, all sensor fields in separate columns.
"""

import serial
import time
import csv
import re
import sys
from datetime import datetime, timezone

# ── Configuration ─────────────────────────────────────────────────────────────
PORT = '/dev/ttyACM0'   # adjust if needed (e.g. /dev/ttyUSB0, COM3)
BAUD = 115200
# ──────────────────────────────────────────────────────────────────────────────

CSV_COLUMNS = [
    'packet_rx',        # packet number assigned by this logger
    'packet_esp',       # packet number from ESP32 ("PACKET #N")
    'pc_timestamp',     # PC wall-clock time (float, ms precision)
    'pc_time_utc',      # PC wall-clock readable  e.g. 2026-04-09 14:32:01.423
    'gps_unix',         # unix timestamp from GPS (or ESP32 millis fallback)
    'gps_time_utc',     # GPS unix -> readable UTC  e.g. 2026-04-09 14:32:00 | blank if fallback
    'time_source',      # "gps" | "millis_fallback" | "none"
    'bmp_temp_c',
    'bmp_pressure_hpa',
    'dht_temp_c',
    'dht_humidity_pct',
    'accel_x', 'accel_y', 'accel_z',   # g
    'gyro_x',  'gyro_y',  'gyro_z',    # deg/s
    'pitch_deg', 'roll_deg',
    'gps_lat', 'gps_lng', 'gps_alt_m',
    'gps_sats',
    'gps_fix',          # 1 if fix, 0 if "No fix"
    'solenoid_state',   # OPEN/CLOSE if button pressed this packet, else blank
    'solenoid_send_ok', # 1=OK, 0=FAILED, blank if no cmd this packet
]

# Events log columns (separate file, one row per button press)
EVENTS_COLUMNS = [
    'event_num',
    'pc_timestamp',
    'pc_time_utc',
    'gps_time_utc',     # GPS time at moment of button press, blank if no fix
    'solenoid_state',   # OPEN or CLOSE
    'send_ok',          # 1 or 0
    'nearest_packet_esp',
]

# ── Regex patterns matching RX3.ino Serial.printf output ─────────────────────
RE_PACKET    = re.compile(r'=== PACKET #(\d+) ===')
RE_TIME      = re.compile(r'unix=(\d+)')
RE_TIME_FB   = re.compile(r'Time \(fallback/raw\): (\d+)')
RE_BMP       = re.compile(r'BMP: T=([\d.\-]+).*?P=([\d.\-]+)')
RE_DHT       = re.compile(r'DHT: T=([\d.\-]+).*?H=([\d.\-]+)')
RE_IMU       = re.compile(r'IMU: A\(([\d.\-]+),\s*([\d.\-]+),\s*([\d.\-]+)\)\s+G\(([\d.\-]+),\s*([\d.\-]+),\s*([\d.\-]+)\)')
RE_ORI       = re.compile(r'Orientation: Pitch=([\d.\-]+).*?Roll=([\d.\-]+)')
RE_GPS       = re.compile(r'GPS: ([\d.\-]+).*?([\d.\-]+).*?([\d.\-]+) m \((\d+) sats\)')
RE_GPS_NONE  = re.compile(r'GPS: No fix')
RE_END       = re.compile(r'^={4,}')
RE_CMD_STATE = re.compile(r'\[CMD\] Solenoid -> (OPEN|CLOSE)')
RE_CMD_SEND  = re.compile(r'\[CMD\] Solenoid command send: (OK|FAILED)')

# GPS unix timestamps must be in a sane range (year 2000 – 2100)
GPS_TS_MIN = 946684800   # 2000-01-01
GPS_TS_MAX = 4102444800  # 2100-01-01


def unix_to_utc(ts):
    """Convert a unix int to a readable UTC string. Returns '' on failure."""
    try:
        return datetime.fromtimestamp(int(ts), tz=timezone.utc).strftime('%Y-%m-%d %H:%M:%S')
    except Exception:
        return ''


def pc_time_str():
    """Current PC time as a readable local string with milliseconds."""
    return datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]


def empty_packet():
    return {col: '' for col in CSV_COLUMNS}


def empty_event():
    return {col: '' for col in EVENTS_COLUMNS}


def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} @ {baud} baud")
        print("Press Ctrl+C to stop.\n")
        return ser
    except serial.SerialException as e:
        print(f"Error: could not open {port} — {e}")
        sys.exit(1)


def parse_line(line, pkt):
    """Apply regex patterns to a single line and update pkt dict in place."""

    # GPS-derived unix timestamp
    m = RE_TIME.search(line)
    if m:
        ts = int(m.group(1))
        pkt['gps_unix'] = ts
        if GPS_TS_MIN < ts < GPS_TS_MAX:
            pkt['gps_time_utc'] = unix_to_utc(ts)
            pkt['time_source']  = 'gps'
        else:
            pkt['gps_time_utc'] = ''
            pkt['time_source']  = 'millis_fallback'
        return

    # millis() fallback timestamp
    m = RE_TIME_FB.search(line)
    if m:
        pkt['gps_unix']     = int(m.group(1))
        pkt['gps_time_utc'] = ''
        pkt['time_source']  = 'millis_fallback'
        return

    m = RE_BMP.search(line)
    if m:
        pkt['bmp_temp_c']       = float(m.group(1))
        pkt['bmp_pressure_hpa'] = float(m.group(2))
        return

    m = RE_DHT.search(line)
    if m:
        pkt['dht_temp_c']       = float(m.group(1))
        pkt['dht_humidity_pct'] = float(m.group(2))
        return

    m = RE_IMU.search(line)
    if m:
        pkt['accel_x'], pkt['accel_y'], pkt['accel_z'] = float(m.group(1)), float(m.group(2)), float(m.group(3))
        pkt['gyro_x'],  pkt['gyro_y'],  pkt['gyro_z']  = float(m.group(4)), float(m.group(5)), float(m.group(6))
        return

    m = RE_ORI.search(line)
    if m:
        pkt['pitch_deg'] = float(m.group(1))
        pkt['roll_deg']  = float(m.group(2))
        return

    if RE_GPS_NONE.search(line):
        pkt['gps_fix'] = 0
        return

    m = RE_GPS.search(line)
    if m:
        pkt['gps_lat']   = float(m.group(1))
        pkt['gps_lng']   = float(m.group(2))
        pkt['gps_alt_m'] = float(m.group(3))
        pkt['gps_sats']  = int(m.group(4))
        pkt['gps_fix']   = 1
        return


def main():
    ser = open_serial(PORT, BAUD)

    stamp = datetime.now().strftime('%d-%m-%Y_%H%M%S')
    data_file   = f"drone_{stamp}.csv"
    events_file = f"drone_{stamp}_events.csv"

    rx_count     = 0
    event_count  = 0
    pkt          = None
    last_esp_pkt = ''
    last_gps_utc = ''    # most recent valid GPS UTC string, for tagging events
    pending_event = None

    with open(data_file, 'w', newline='') as df, \
         open(events_file, 'w', newline='') as ef:

        writer  = csv.DictWriter(df, fieldnames=CSV_COLUMNS)
        ewriter = csv.DictWriter(ef, fieldnames=EVENTS_COLUMNS)
        writer.writeheader()
        ewriter.writeheader()

        try:
            while True:
                line_bytes = ser.readline()
                if not line_bytes:
                    continue

                line = line_bytes.decode(errors='ignore').strip()
                if not line:
                    continue

                # ── Button press: solenoid state line ─────────────────────
                m = RE_CMD_STATE.search(line)
                if m:
                    state = m.group(1)
                    pending_event = empty_event()
                    pending_event['pc_timestamp']       = round(time.time(), 3)
                    pending_event['pc_time_utc']        = pc_time_str()
                    pending_event['gps_time_utc']       = last_gps_utc
                    pending_event['solenoid_state']     = state
                    pending_event['nearest_packet_esp'] = last_esp_pkt
                    print(f"  [BTN] Solenoid -> {state}  (GPS: {last_gps_utc or 'no fix'})")
                    continue

                # ── Button press: send-status line ────────────────────────
                m = RE_CMD_SEND.search(line)
                if m:
                    ok = 1 if m.group(1) == 'OK' else 0
                    if pending_event is not None:
                        pending_event['send_ok'] = ok
                        event_count += 1
                        pending_event['event_num'] = event_count
                        ewriter.writerow(pending_event)
                        ef.flush()
                        if pkt is not None:
                            pkt['solenoid_state']   = pending_event['solenoid_state']
                            pkt['solenoid_send_ok'] = ok
                        pending_event = None
                    continue

                # ── New sensor packet starts ───────────────────────────────
                m = RE_PACKET.search(line)
                if m:
                    pkt = empty_packet()
                    pkt['packet_esp']   = int(m.group(1))
                    pkt['pc_timestamp'] = round(time.time(), 3)
                    pkt['pc_time_utc']  = pc_time_str()
                    pkt['time_source']  = 'none'
                    last_esp_pkt = pkt['packet_esp']
                    continue

                # ── End of packet — write row ──────────────────────────────
                if pkt is not None and RE_END.match(line) and line.count('=') > 8:
                    rx_count += 1
                    pkt['packet_rx'] = rx_count

                    # Keep last_gps_utc updated for event tagging
                    if pkt.get('gps_time_utc'):
                        last_gps_utc = pkt['gps_time_utc']

                    writer.writerow(pkt)
                    df.flush()

                    sol      = pkt.get('solenoid_state') or ''
                    gps_disp = pkt.get('gps_time_utc') or f"millis={pkt.get('gps_unix','?')}"
                    print(
                        f"[{rx_count:4d}] esp#{pkt['packet_esp']} | "
                        f"{gps_disp} | "
                        f"Pitch={pkt.get('pitch_deg','?'):>6}° "
                        f"Roll={pkt.get('roll_deg','?'):>6}° | "
                        f"GPS fix={pkt.get('gps_fix','?')}"
                        + (f" | SOLENOID {sol}" if sol else "")
                    )
                    pkt = None
                    continue

                # ── Parse lines inside a packet ────────────────────────────
                if pkt is not None:
                    parse_line(line, pkt)

        except KeyboardInterrupt:
            print("\nLogger stopped.")

        finally:
            ser.close()

    print(f"\nData saved to:   {data_file}")
    print(f"Events saved to: {events_file}")
    print(f"Packets logged:  {rx_count}")
    print(f"Button events:   {event_count}")


if __name__ == "__main__":
    main()
