"""Sun position and clear-sky vertical-surface irradiance, stdlib only.

Ports `sun_position`/`vertical_irradiance` from analysis/solar_orient_fit.py
(which stays unchanged, operating on a pandas DatetimeIndex over a whole
archive) to operate on a single timezone-aware `datetime`, returning plain
floats -- so `sim/` (stdlib `math`/`datetime` only, no numpy/pandas) can
share the exact same NOAA-style approximation that analysis/solar_orient_fit.py
uses to fit the s_ne/s_nw terms baked into sim/house.py's defaults. Keeping
the two in lockstep matters: a formula drift between analysis and sim would
silently invalidate the fitted coefficients.
"""
from __future__ import annotations

import math
from datetime import datetime, timezone

LAT = -34.93  # Adelaide
LON = 138.60
AZ_NE = 45.0
AZ_NW = 315.0  # winter sun sets ~299 deg (WNW); SW gets no direct winter beam
MIN_ELEV_DEG = 3.0


def sun_position(ts: datetime) -> tuple[float, float]:
    """(elevation_rad, azimuth_rad from north, clockwise) at `ts` -- NOAA-style
    approximation, adequate for irradiance weighting. `ts` must be
    timezone-aware; it is converted to UTC internally."""
    utc = ts.astimezone(timezone.utc)
    doy = utc.timetuple().tm_yday
    frac_h = utc.hour + utc.minute / 60.0 + utc.second / 3600.0
    gamma = 2 * math.pi / 365.0 * (doy - 1 + (frac_h - 12) / 24.0)
    decl = (
        0.006918 - 0.399912 * math.cos(gamma) + 0.070257 * math.sin(gamma)
        - 0.006758 * math.cos(2 * gamma) + 0.000907 * math.sin(2 * gamma)
        - 0.002697 * math.cos(3 * gamma) + 0.00148 * math.sin(3 * gamma)
    )
    eqtime = 229.18 * (
        0.000075 + 0.001868 * math.cos(gamma) - 0.032077 * math.sin(gamma)
        - 0.014615 * math.cos(2 * gamma) - 0.040849 * math.sin(2 * gamma)
    )
    tst = frac_h * 60.0 + eqtime + 4.0 * LON  # true solar time [min]
    ha = math.radians(tst / 4.0 - 180.0)  # hour angle
    ha = (ha + math.pi) % (2 * math.pi) - math.pi  # wrap to (-pi, pi]
    lat = math.radians(LAT)
    sin_elev = math.sin(lat) * math.sin(decl) + math.cos(lat) * math.cos(decl) * math.cos(ha)
    elev = math.asin(max(-1.0, min(1.0, sin_elev)))
    cos_az = (math.sin(decl) - math.sin(lat) * sin_elev) / (math.cos(lat) * math.cos(elev))
    az = math.acos(max(-1.0, min(1.0, cos_az)))
    if ha > 0:  # afternoon: west of north
        az = 2 * math.pi - az
    return elev, az


def vertical_irradiance(ts: datetime, az_deg: float) -> float:
    """Clear-sky direct-beam factor (0..1) on a vertical surface facing
    az_deg, at `ts`."""
    elev, az = sun_position(ts)
    face = math.radians(az_deg)
    fac = math.cos(elev) * math.cos(az - face)
    if math.degrees(elev) < MIN_ELEV_DEG:
        return 0.0
    return max(0.0, fac)
