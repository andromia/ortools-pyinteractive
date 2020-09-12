# %%
#::IMPORTS & SETUP DATA
from typing import List, Tuple  # TODO: typing for numpy arrays?
from collections import namedtuple
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from haversine import haversine_vector, Unit
import numpy as np

# origin data
ORIGIN_LATS: List[float] = [37.69]
ORIGIN_LONS: List[float] = [-79.31]

assert len(ORIGIN_LATS) == len(ORIGIN_LONS)

# destination data
DEST_LATS: List[float] = [
    40.17171,
    33.98832,
    33.91633,
    39.82954,
    39.94745,
    33.93216,
    40.82196,
]
DEST_LONS: List[float] = [
    -80.25643,
    -83.87952,
    -84.82781,
    -75.43543,
    -75.14733,
    -83.35259,
    -74.42669,
]

assert len(DEST_LATS) == len(DEST_LONS)

# flow data
QUANTITIES: List[int] = [5] * len(DEST_LATS)

assert len(QUANTITIES) == len(DEST_LATS)
assert len(QUANTITIES) == len(DEST_LONS)

# vehicle data
MAX_VEHICLE_CAP: int = 26
MAX_VEHICLE_DIST: int = 2000
NUM_VEHICLES: int = len(QUANTITIES)

assert all(x < MAX_VEHICLE_CAP for x in QUANTITIES)

ORIGIN_TYPE = Tuple[float, float]
ORIGIN: ORIGIN_TYPE = namedtuple("Origin", ["lat", "lon"])
ORIGINS: List[ORIGIN_TYPE] = [
    ORIGIN(ORIGIN_LATS[i], ORIGIN_LONS[i]) for i in range(len(ORIGIN_LATS))
]

DEMAND_TYPE = Tuple[float, float, int]
DEMAND: DEMAND_TYPE = namedtuple(
    "Demand", ["lat", "lon", "qty"]
)  # TODO: add windows, dim_x, dim_y
DEMANDS: List[DEMAND_TYPE] = [
    DEMAND(DEST_LATS[i], DEST_LONS[i], QUANTITIES[i]) for i in range(len(DEST_LATS))
]

VEHICLE_TYPE = Tuple[int, int]
VEHICLE: VEHICLE_TYPE = namedtuple("Vehicle", ["qty_cap", "max_dist"])
VEHICLES: List[VEHICLE_TYPE] = [
    VEHICLE(MAX_VEHICLE_CAP, MAX_VEHICLE_DIST) for i in range(NUM_VEHICLES)
]

# %%
#::DISTANCE DATA PROCESSING


def create_vectorized_haversine_li(
    olat: float,
    olon: float,
    dlats: List[float],
    dlons: List[float],
):
    assert len(dlats) == len(dlons)

    olats: List[float] = [olat] * len(dlats)
    olons: List[float] = [olon] * len(dlons)
    os: List[Tuple[float, float]] = list(zip(olats, olons))
    ds: List[Tuple[float, float]] = list(zip(dlats, dlons))

    ds: List[float] = haversine_vector(os, ds, unit=Unit.MILES)

    return ds


def create_matrix(origin: ORIGIN_TYPE, dests: DEMAND_TYPE):
    """
    creates matrix using optimized matrix processing. distances
    are converted to integers (x*100).

    :origin:    (origin.lat: float, origin.lon: float)
    :dests:     list of demands; (demand.lat, demand.lon)

    returns matrix: list[list, ..., len(origin+dests)]
    """
    LATS: List[float] = [origin.lat] + [d.lat for d in dests]
    LONS: List[float] = [origin.lon] + [d.lon for d in dests]

    assert len(LATS) == len(LONS)

    matrix: List[List[int]] = []
    for i in range(len(LATS)):
        distances = create_vectorized_haversine_li(
            olat=LATS[i], olon=LONS[i], dlats=LATS, dlons=LONS
        )

        distances = np.ceil(distances * 100).astype(int)
        matrix.append(distances)

    return matrix


DIST_MATRIX: List[List[float]] = create_matrix(origin=ORIGINS[0], dests=DEMANDS)
