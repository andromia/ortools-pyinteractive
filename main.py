# %%
#::IMPORTS & SETUP DATA

"""TODO:
[x] Capacity
[X] Distance
[X] Windows
[X] Travel Time NOTE: excluding layover/etc.
[ ] Costs
[ ] Business Penalties & Incentives
[ ] Dev Tooling
"""
# NOTE: for ipykernels function scoped vars prefixed with underscores
# TODO: either figure out how people deal with this or write a linter
from typing import List, Tuple  # TODO: typing for numpy arrays?
from collections import namedtuple
import pandas as pd
import os

# integer processing
INT_PRECISION: int = 100

# ortools config
MAX_SEARCH_SECONDS: int = 10

# origin data
ORIGIN_LAT: float = 40.0
ORIGIN_LON: float = -88

# demand data using df2
FILEPATH: str = os.path.join("d:\\", "data", "routing-sample-data", "vrp_testing_data.csv")
df: pd.DataFrame = pd.read_csv(FILEPATH)
DEST_LATS: List[float] = df.latitude.tolist()
DEST_LONS: List[float] = df.longitude.tolist()
ALL_DEMANDS: List[int] = [0] + df.pallets.tolist()

# vehicle data
MAX_VEHICLE_CAP: int = 26
MAX_VEHICLE_DIST: int = 300000  # distance is x*100 for integers
NUM_VEHICLES: int = len(ALL_DEMANDS)
SOFT_MAX_VEHICLE_DIST: int = int(MAX_VEHICLE_DIST * 0.75)
SOFT_MAX_VEHICLE_COST: int = SOFT_MAX_VEHICLE_DIST + 1  # cost feels ambiguous

# assert all(x < MAX_VEHICLE_CAP for x in ALL_DEMANDS)

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

TIME_MATRIX: List[List[int]] = (
    (np.array(DIST_MATRIX) / 440 / INT_PRECISION).round(0).astype(int)
)

TIME_WINDOWS: List[Tuple[int, int]] = [(8, 17)] * len(DIST_MATRIX)

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
    + list(zip(list(range(1, len(ALL_DEMANDS) + 1)), DEST_LATS, DEST_LONS)),
    dtype=[("idx", int), ("lat", float), ("lon", float)],
)
DIST_MATRIX_ARR: np.ndarray = np.array(DIST_MATRIX)
WINDOWS_MATRIX_ARR: np.ndarray = np.array(TIME_MATRIX)
WINDOWS_ARR: np.ndarray = np.array(TIME_WINDOWS, dtype=object)
DEMAND_ARR: np.ndarray = np.array(ALL_DEMANDS)
VEHICLE_CAP_ARR: np.ndarray = np.array(VEHICLE_CAPACITIES)

# preprocess exceptions based on MAX_VEHICLE_DIST
EXCEPTIONS = np.where(DIST_MATRIX_ARR[0] > MAX_VEHICLE_DIST)

vehicles = []
for i, c in enumerate(np.unique(CLUSTERS)):

    # align with matrix
    is_cluster = np.where(CLUSTERS == c)[0]
    is_cluster = is_cluster + 1
    is_cluster = np.insert(is_cluster, 0, 0)
    is_cluster = is_cluster[~np.isin(is_cluster, EXCEPTIONS)]

    solution = model.solve(
        nodes=NODES_ARR[is_cluster],
        distance_matrix=DIST_MATRIX_ARR[is_cluster],
        time_matrix=WINDOWS_MATRIX_ARR[is_cluster],
        time_windows=WINDOWS_ARR[is_cluster],
        demand=DEMAND_ARR[is_cluster],
        vehicle_caps=VEHICLE_CAP_ARR[is_cluster],
        depot_index=0,
        constraints=CONSTRAINTS,
        max_search_seconds=MAX_SEARCH_SECONDS,
    )

    if not solution:
        continue

    for vehicle in solution:
        vehicles.append(vehicle)

model.visualize_solution(vehicles)
