# destination data
DEST_LATS: List[float] = [
    40.17171,
    33.98832,
    33.91633,
    39.82954,
    39.94745,
    33.93216,
    40.82196,
    40.10201,
]
DEST_LONS: List[float] = [
    -80.25643,
    -83.87952,
    -84.82781,
    -75.43543,
    -75.14733,
    -83.35259,
    -74.42669,
    -110.23992,
]

assert len(DEST_LATS) == len(DEST_LONS)

# flow data (includes origin as 0th index)
ALL_DEMANDS: List[int] = [0, 5, 3, 7, 10, 15, 7, 8, 10]

assert len(ALL_DEMANDS) == len(DEST_LATS) + 1
assert len(ALL_DEMANDS) == len(DEST_LONS) + 1
