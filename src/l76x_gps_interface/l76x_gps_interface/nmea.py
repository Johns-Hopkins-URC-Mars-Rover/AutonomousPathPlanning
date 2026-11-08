"""Checksum-checked WGS84 NMEA parsing; independent of ROS and hardware."""
import math

def decimal_degrees(value, hemisphere, is_latitude):
    """Convert NMEA ddmm.mmmm / dddmm.mmmm to signed degrees."""
    allowed = ("N", "S") if is_latitude else ("E", "W")
    if hemisphere not in allowed:
        raise ValueError("Invalid hemisphere")

    raw = float(value)
    if not math.isfinite(raw) or raw < 0:
        raise ValueError("Invalid coordinate")

    degrees = int(raw // 100)
    minutes = raw - degrees * 100

    if not 0 <= minutes < 60:
        raise ValueError("Invalid minutes")

    result = degrees + minutes / 60.0
    limit = 90 if is_latitude else 180

    if result > limit:
        raise ValueError("Coordinate out of range")

    return -result if hemisphere in ("S", "W") else result


def parse_sentence(line, kinds):
    """Return fields only for a checksum-valid sentence of an allowed kind."""
    try:
        text = line.decode("ascii").strip()

        if not text.startswith("$"):
            return None

        body, checksum = text[1:].split("*")
        if len(checksum) != 2:
            return None

        calculated = 0
        for character in body:
            calculated ^= ord(character)

        if calculated != int(checksum, 16):
            return None

        fields = body.split(",")
        if fields[0] not in kinds:
            return None


        return fields

    except (UnicodeDecodeError, ValueError):
        return None


def parse_rmc(line):
    fields = parse_sentence(line, ('GPRMC', 'GNRMC'))
    if fields is None or len(fields) < 10 or fields[2] not in ('A', 'V'):
        return None
    return fields


def parse_gga(line):
    fields = parse_sentence(line, ('GPGGA', 'GNGGA'))
    return fields if fields is not None and len(fields) >= 13 else None


def ellipsoid_altitude(fields):
    if int(fields[6]) <= 0 or fields[10] != 'M' or fields[12] != 'M':
        raise ValueError('GGA needs a fix and altitude/geoid separation in meters')
    altitude = float(fields[9]) + float(fields[11])
    if not math.isfinite(altitude):
        raise ValueError('invalid GGA altitude')
    return altitude



def utc_timestamp(fields):
    """RMC acquisition time as Unix UTC seconds; reject missing/invalid dates."""
    from datetime import datetime, timezone
    date, clock = fields[9], fields[1]
    if len(date) != 6 or len(clock) < 6:
        raise ValueError('RMC needs valid UTC date and time')
    second = float(clock[4:])
    if not math.isfinite(second) or not 0 <= second < 60:
        raise ValueError('invalid RMC UTC seconds')
    result = datetime.strptime(date + clock[:4], '%d%m%y%H%M').replace(tzinfo=timezone.utc)
    return result.timestamp() + second
