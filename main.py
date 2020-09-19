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
INT_PRECISION = 100

# origin data
ORIGIN_LATS = [37.69]
ORIGIN_LONS = [-79.31]

assert len(ORIGIN_LATS) == len(ORIGIN_LONS)

# destination data
DEST_LATS = [
    40.17171,
    33.98832,
    33.91633,
    39.82954,
    39.94745,
    33.93216,
    40.82196,
]
DEST_LONS = [
    -80.25643,
    -83.87952,
    -84.82781,
    -75.43543,
    -75.14733,
    -83.35259,
    -74.42669,
]

assert len(DEST_LATS) == len(DEST_LONS)

# depot data
ORIGIN_TYPE = Tuple[float, float]
Origin: ORIGIN_TYPE = namedtuple("Origin", ["lat", "lon"])
ORIGINS: List[ORIGIN_TYPE] = [
    Origin(ORIGIN_LATS[i], ORIGIN_LONS[i]) for i in range(len(ORIGIN_LATS))
]

# flow data (includes origin as 0th index)
ALL_DEMANDS: List[int] = [0, 5, 3, 7, 10, 15, 7, 8]

assert len(ALL_DEMANDS) == len(ORIGINS) + len(DEST_LATS)
assert len(ALL_DEMANDS) == len(ORIGINS) + len(DEST_LONS)

# vehicle data
MAX_VEHICLE_CAP: int = 26
MAX_VEHICLE_DIST: int = 100000  # distance is x*100 for integers
NUM_VEHICLES = len(ALL_DEMANDS)
SOFT_MAX_VEHICLE_DIST: int = int(MAX_VEHICLE_DIST * 0.75)
SOFT_MAX_VEHICLE_COST: int = 100000

assert all(x < MAX_VEHICLE_CAP for x in ALL_DEMANDS)

DEMAND_TYPE = Tuple[float, float, int]
# TODO: add windows, dim_x, dim_y
Demand: DEMAND_TYPE = namedtuple("Demand", ["lat", "lon", "qty"])
# NOTE: assumes QUANTITIES has origin qty at index 0
DEMANDS: List[DEMAND_TYPE] = [
    Demand(DEST_LATS[i], DEST_LONS[i], ALL_DEMANDS[i + 1])
    for i in range(len(DEST_LATS))
]

VEHICLE_TYPE = Tuple[int, int]
Vehicle: VEHICLE_TYPE = namedtuple("Vehicle", ["cap"])
VEHICLES: List[VEHICLE_TYPE] = [Vehicle(MAX_VEHICLE_CAP) for i in range(NUM_VEHICLES)]


# %%
#::CLUSTER PROCESSING
from src import cluster

clusters = cluster.create_dbscan_clusters(
    lats=[d.lat for d in DEMANDS], lons=[d.lon for d in DEMANDS]
)

# %%
#::ORTOOLS MODEL
from src import distance
from src import model

import numpy as np


DIST_MATRIX: List[List[int]] = distance.create_matrix(
    _origin=ORIGINS[0], _dests=DEMANDS
)

CONSTRAINTS_TYPE = Tuple[int, int, int]
Constraints: CONSTRAINTS_TYPE = namedtuple(
    "Constraint", ["dist_constraint", "soft_dist_constraint", "soft_dist_penalty"]
)
CONSTRAINTS = Constraints(
    dist_constraint=100000, soft_dist_constraint=75000, soft_dist_penalty=100000
)

solutions = []
for i, c in enumerate(np.unique(clusters)):

    # align with matrix
    is_cluster = np.where(clusters == c)[0]
    is_cluster = is_cluster + 1
    is_cluster = np.insert(is_cluster, 0, 0)

    solution = model.solve(
        nodes=[(ORIGINS + DEMANDS)[j] for j in is_cluster],
        distance_matrix=[DIST_MATRIX[j] for j in is_cluster],
        demand=[ALL_DEMANDS[j] for j in is_cluster],
        vehicles=[VEHICLES[j] for j in is_cluster],
        depot_index=0,
        constraints=CONSTRAINTS,
    )

    solutions.append(solution)


# model.visualize_solution(solution)
