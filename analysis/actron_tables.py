"""ActronAir URC-100AS / LRE-100AS rated performance tables.

Transcribed 2026-07-02 from the Technical Selection Data sheet
(Ver. 7 190329, (c) 2016 Actron Engineering) supplied by the owner.
These are FULL-LOAD (rated compressor speed) points; part-load inverter
COP is typically better. Capacities are at the unit — duct losses to the
rooms are not included.

Unit facts (spec page):
- 1 x twin-rotary compressor, DC inverter. Compressor rated input 2600 W.
- Rated load amps: cooling 14.1 A, heating 13.7 A (~3.15 kW @ 230 V).
- Outdoor fan 150 W; indoor fan 420 W, airflow 440/520/600 l/s min/nom/max.
- Defrost: reverse cycle. Operating range heating: outdoor -25..30 C.
- R-410A, factory charge 3.8 kg (pre-charge length 15 m).
"""

# HEATING: {indoor_db_C: {outdoor_db_C: (net_capacity_kW, power_input_kW)}}
# Outdoor column headers are dry-bulb; the sheet pairs them with wet-bulb
# values ~1 K lower (saturated coil conditions).
HEATING = {
    15: {-15: (4.27, 1.72), -7: (7.75, 2.79), -5: (8.55, 2.46), 0: (9.46, 3.01),
         4: (10.07, 3.38), 7: (12.21, 3.35), 12: (13.43, 3.68), 24: (11.26, 3.08)},
    18: {-15: (4.08, 1.66), -7: (7.41, 2.69), -5: (8.16, 2.37), 0: (9.04, 2.91),
         4: (9.62, 3.26), 7: (11.66, 3.23), 12: (12.83, 3.55), 24: (10.76, 2.97)},
    20: {-15: (3.82, 1.56), -7: (6.92, 2.54), -5: (7.63, 2.24), 0: (8.45, 2.75),
         4: (8.99, 3.08), 7: (10.90, 3.05), 12: (11.99, 3.36), 24: (10.06, 2.81)},
    22: {-15: (3.70, 1.60), -7: (6.71, 2.59), -5: (7.40, 2.28), 0: (8.19, 2.80),
         4: (8.72, 3.15), 7: (10.57, 3.11), 12: (11.63, 3.43), 24: (9.75, 2.87)},
    27: {-15: (3.32, 1.60), -7: (6.02, 2.60), -5: (6.64, 2.29), 0: (7.35, 2.81),
         4: (7.82, 3.15), 7: (9.48, 3.12), 12: (10.43, 3.43), 24: (8.75, 2.87)},
}

# COOLING: {(indoor_db_C, indoor_wb_C): {outdoor_db_C:
#           (net_capacity_kW, sensible_capacity_kW, power_input_kW)}}
COOLING = {
    (21, 15): {21: (9.73, 7.66, 1.16), 25: (9.56, 7.52, 1.91), 30: (9.39, 7.39, 2.32),
               35: (8.93, 7.03, 2.84), 40: (8.53, 6.71, 3.18), 45: (6.25, 4.92, 3.21),
               50: (4.47, 3.51, 3.24)},
    (24, 17): {21: (10.36, 8.15, 1.19), 25: (10.17, 8.00, 1.97), 30: (9.99, 7.87, 2.40),
               35: (9.50, 7.48, 2.93), 40: (9.07, 7.14, 3.28), 45: (6.65, 5.23, 3.31),
               50: (4.75, 3.74, 3.34)},
    (27, 19): {21: (10.90, 8.58, 1.23), 25: (10.70, 8.42, 2.03), 30: (10.52, 8.28, 2.47),
               35: (10.00, 7.87, 3.02), 40: (9.55, 7.52, 3.38), 45: (7.00, 5.51, 3.41),
               50: (5.00, 3.94, 3.45)},
    (32, 23): {21: (12.21, 9.61, 1.36), 25: (11.98, 9.43, 2.24), 30: (11.78, 9.27, 2.73),
               35: (11.20, 8.81, 3.33), 40: (10.70, 8.42, 3.73), 45: (7.84, 6.17, 3.77),
               50: (5.60, 4.41, 3.81)},
}


def heating_cop(indoor_db: int, outdoor_db: int) -> float:
    cap, pwr = HEATING[indoor_db][outdoor_db]
    return cap / pwr


if __name__ == "__main__":
    print("rated heating COP at 20C indoor:")
    for tout, (cap, pwr) in HEATING[20].items():
        print(f"  Tout {tout:+3d}C: {cap:5.2f} kW / {pwr:4.2f} kW = COP {cap / pwr:.2f}")
