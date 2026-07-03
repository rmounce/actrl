"""Sanity checks for sim/solar.py against known Adelaide winter-solstice sun
geometry (docs/tasks/011-orientation-solar.md)."""
from __future__ import annotations

from datetime import datetime, timedelta, timezone

from sim.solar import AZ_NE, AZ_NW, sun_position, vertical_irradiance

ADELAIDE_TZ = timezone(timedelta(hours=9, minutes=30))  # no DST in June (winter)


def _adelaide(year: int, month: int, day: int, hour: int, minute: int = 0) -> datetime:
    return datetime(year, month, day, hour, minute, tzinfo=ADELAIDE_TZ)


def test_winter_solstice_solar_noon_geometry():
    """At 2026-06-21 12:00 local (Adelaide), the sun should sit close to due
    north (low winter arc) at roughly 31-33 deg elevation."""
    ts = _adelaide(2026, 6, 21, 12, 0)
    elev, az = sun_position(ts)
    elev_deg = elev * 180.0 / 3.141592653589793
    az_deg = az * 180.0 / 3.141592653589793

    assert 31.0 <= elev_deg <= 33.0
    # azimuth measured from north, clockwise -- "near north" means near 0
    # (equivalently near 360), so compare via the wrapped difference.
    az_diff = min(az_deg, 360.0 - az_deg)
    assert az_diff <= 10.0


def test_ne_irradiance_peaks_in_the_morning():
    """A NE-facing vertical surface catches the morning beam and is dark
    by mid-afternoon (Adelaide winter sun: rise ~61 deg ENE, set ~299 deg
    WNW). The hour angle is wrapped to (-pi, pi] (fix 2026-07-03: the
    original unwrapped formula pushed pre-~09:45-local times into the
    wrong azimuth branch, zeroing the NE basis for most of the morning)."""
    sunrise_side = vertical_irradiance(_adelaide(2026, 6, 21, 8, 0), AZ_NE)
    morning = vertical_irradiance(_adelaide(2026, 6, 21, 9, 0), AZ_NE)
    afternoon = vertical_irradiance(_adelaide(2026, 6, 21, 15, 0), AZ_NE)
    assert sunrise_side > 0.0
    assert morning > afternoon


def test_sw_facing_surface_gets_no_direct_winter_noon_beam():
    """SW (225 deg) glazing receives ~0 direct beam at winter solar noon --
    the sun tracks NE -> N -> NW in Adelaide's winter, never south of due
    east/west."""
    ts = _adelaide(2026, 6, 21, 12, 0)
    fac = vertical_irradiance(ts, 225.0)
    assert fac == 0.0
