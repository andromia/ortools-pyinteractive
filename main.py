# %%
#::IMPORTS & DATA
from typing import List, Tuple
from collections import namedtuple
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from haversine import haversine_vector, Unit
import numpy as np

# origin data
ORIGIN_LATS: List[float] = [37.0]
ORIGIN_LONS: List[float] = [-79.0]

assert len(ORIGIN_LATS) == len(ORIGIN_LONS)

# destination data
DEST_LATS: List[float] = [40.1717, 33.9883, 33.9163, 39.8295, 39.9474, 33.9321, 40.8219]
DEST_LONS: List[float] = [
    -80.256,
    -83.8795,
    -84.8278,
    -75.4354,
    -75.1473,
    -83.3525,
    -74.42,
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
