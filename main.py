# %%
#::IMPORTS & SETUP DATA

"""TODO:
[x] Capacity
[ ] Distance
[ ] Travel Time
[ ] Windows
[ ] Cost
[ ] Business Penalties
[ ] Dev Tooling
"""
# NOTE: for ipykernels function scoped vars prefixed with underscores
# TODO: either figure out how people deal with this or write a linter
from typing import List, Tuple  # TODO: typing for numpy arrays?
from collections import namedtuple


# integer processing
INT_PRECISION: int = 100

# origin data
ORIGIN_LAT: float = 37.69
ORIGIN_LON: float = -79.31

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

# vehicle data
MAX_VEHICLE_CAP: int = 26
MAX_VEHICLE_DIST: int = 100000  # distance is x*100 for integers
NUM_VEHICLES: int = len(ALL_DEMANDS)
SOFT_MAX_VEHICLE_DIST: int = int(MAX_VEHICLE_DIST * 0.75)
SOFT_MAX_VEHICLE_COST: int = 100000

assert all(x < MAX_VEHICLE_CAP for x in ALL_DEMANDS)

VEHICLE_CAPACITIES: List[int] = [MAX_VEHICLE_CAP for i in range(NUM_VEHICLES)]


# %%
#::CLUSTER PROCESSING
from src import cluster

CLUSTERS = cluster.create_dbscan_clusters(lats=DEST_LATS, lons=DEST_LONS)

# %%
#::ORTOOLS MODEL
from src import distance
from src import model

import numpy as np


DIST_MATRIX: List[List[int]] = distance.create_matrix(
    origin_lat=ORIGIN_LAT,
    origin_lon=ORIGIN_LON,
    dest_lats=DEST_LATS,
    dest_lons=DEST_LONS,
)

CONSTRAINTS_TYPE = Tuple[int, int, int]
Constraints: CONSTRAINTS_TYPE = namedtuple(
    "Constraint", ["dist_constraint", "soft_dist_constraint", "soft_dist_penalty"]
)
CONSTRAINTS = Constraints(
    dist_constraint=MAX_VEHICLE_DIST,
    soft_dist_constraint=SOFT_MAX_VEHICLE_DIST,
    soft_dist_penalty=SOFT_MAX_VEHICLE_COST,
)

NODES_ARR: np.ndarray = np.array(
    [(0, ORIGIN_LAT, ORIGIN_LON)]
    + list(zip(list(range(1, len(ALL_DEMANDS))), DEST_LATS, DEST_LONS)),
    dtype=[("idx", int), ("lat", float), ("lon", float)],
)
MATRIX_ARR: np.ndarray = np.array(DIST_MATRIX)
DEMAND_ARR: np.ndarray = np.array(ALL_DEMANDS)
VEHICLE_CAP_ARR: np.ndarray = np.array(VEHICLE_CAPACITIES)

# preprocess exceptions based on MAX_VEHICLE_DIST
EXCEPTIONS = np.where(MATRIX_ARR[0] > MAX_VEHICLE_DIST)

vehicles = []
for i, c in enumerate(np.unique(CLUSTERS)):

    # align with matrix
    is_cluster = np.where(CLUSTERS == c)[0]
    is_cluster = is_cluster + 1
    is_cluster = np.insert(is_cluster, 0, 0)
    is_cluster = is_cluster[~np.isin(is_cluster, EXCEPTIONS)]

    solution = model.solve(
        nodes=NODES_ARR[is_cluster],
        distance_matrix=MATRIX_ARR[is_cluster],
        demand=DEMAND_ARR[is_cluster],
        vehicle_caps=VEHICLE_CAP_ARR[is_cluster],
        depot_index=0,
        constraints=CONSTRAINTS,
    )

    for vehicle in solution:
        vehicles.append(vehicle)

model.visualize_solution(vehicles)
