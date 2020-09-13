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
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from haversine import haversine_vector, Unit
import numpy as np


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
ORIGIN: ORIGIN_TYPE = namedtuple("Origin", ["lat", "lon"])
ORIGINS: List[ORIGIN_TYPE] = [
    ORIGIN(ORIGIN_LATS[i], ORIGIN_LONS[i]) for i in range(len(ORIGIN_LATS))
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
DEMAND: DEMAND_TYPE = namedtuple("Demand", ["lat", "lon", "qty"])
# NOTE: assumes QUANTITIES has origin qty at index 0
DEMANDS: List[DEMAND_TYPE] = [
    DEMAND(DEST_LATS[i], DEST_LONS[i], ALL_DEMANDS[i + 1])
    for i in range(len(DEST_LATS))
]

VEHICLE_TYPE = Tuple[int, int]
VEHICLE: VEHICLE_TYPE = namedtuple("Vehicle", ["qty_cap", "max_dist"])
VEHICLES: List[VEHICLE_TYPE] = [
    VEHICLE(MAX_VEHICLE_CAP, MAX_VEHICLE_DIST) for i in range(NUM_VEHICLES)
]


# %%
#::DISTANCE DATA PROCESSING


def create_vectorized_haversine_li(
    _olat: float,
    _olon: float,
    _dlats: List[float],
    _dlons: List[float],
):
    assert len(_dlats) == len(_dlons)

    _olats: List[float] = [_olat] * len(_dlats)
    _olons: List[float] = [_olon] * len(_dlons)
    _os: List[Tuple[float, float]] = list(zip(_olats, _olons))
    _ds: List[Tuple[float, float]] = list(zip(_dlats, _dlons))

    _ds: List[float] = haversine_vector(_os, _ds, unit=Unit.MILES)

    return _ds


def create_matrix(_origin: ORIGIN_TYPE, _dests: DEMAND_TYPE):
    """
    creates matrix using optimized matrix processing. distances
    are converted to integers (x*100).

    :_origin:    (origin.lat: float, origin.lon: float)
    :_dests:     list of demands; (demand.lat, demand.lon)

    returns _matrix: list[list, ..., len(origin+dests)]
    """
    _LATS: List[float] = [_origin.lat] + [d.lat for d in _dests]
    _LONS: List[float] = [_origin.lon] + [d.lon for d in _dests]

    assert len(_LATS) == len(_LONS)

    _matrix: List[List[int]] = []
    for _i in range(len(_LATS)):
        _fdistances: List[float] = create_vectorized_haversine_li(
            _olat=_LATS[_i], _olon=_LONS[_i], _dlats=_LATS, _dlons=_LONS
        )

        _idistances: List[int] = np.ceil(_fdistances * INT_PRECISION).astype(int)
        _matrix.append(_idistances)

    return _matrix


DIST_MATRIX: List[List[int]] = create_matrix(_origin=ORIGINS[0], _dests=DEMANDS)


# %%
#::ORTOOLS MODEL SETUP


NUM_NODES = len(DIST_MATRIX)
DEPOT_INDEX = 0

manager = pywrapcp.RoutingIndexManager(NUM_NODES, NUM_VEHICLES, DEPOT_INDEX)


def matrix_callback(_i: int, _j: int):
    """index of from (i) and to (j)"""
    _node_i = manager.IndexToNode(_i)
    _node_j = manager.IndexToNode(_j)
    _d: int = DIST_MATRIX[_node_i][_node_j]

    return _d


def demand_callback(_i: int):
    """capacity constraint"""
    _d = ALL_DEMANDS[manager.IndexToNode(_i)]

    return _d


model = pywrapcp.RoutingModel(manager)

# distance constraints
callback_id = model.RegisterTransitCallback(matrix_callback)
model.SetArcCostEvaluatorOfAllVehicles(callback_id)
model.AddDimensionWithVehicleCapacity(
    callback_id,
    0,  # 0 slack
    [MAX_VEHICLE_DIST for i in range(NUM_VEHICLES)],
    True,  # start to zero
    "Distance",
)

# demand constraint setup
model.AddDimensionWithVehicleCapacity(
    # function which return the load at each location (cf. cvrp.py example)
    model.RegisterUnaryTransitCallback(demand_callback),
    0,  # null capacity slack
    [v.qty_cap for v in VEHICLES],  # vehicle maximum capacity
    True,  # start cumul to zero
    "Capacity",
)

dst_dim = model.GetDimensionOrDie("Distance")
for i in range(manager.GetNumberOfVehicles()):
    end_idx = model.End(i)
    dst_dim.SetCumulVarSoftUpperBound(
        end_idx, SOFT_MAX_VEHICLE_DIST, SOFT_MAX_VEHICLE_COST
    )


MAX_SEARCH_SECONDS: int = 5

search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
)
search_parameters.time_limit.seconds = MAX_SEARCH_SECONDS

# %%
#::SOLVE MODEL


assignment = model.SolveWithParameters(search_parameters)


def get_solution(_manager, _model, _assignment):
    """
    Return a string displaying the output of the _model instance and
    assignment (_assignment).
    Args: _model (ortools.constraint_solver.pywrapcp.RoutingModel): _model.
    _assignment (ortools.constraint_solver.pywrapcp.Assignment): the assignment.
    Returns:
        (string) _assignment_output: describing each vehicle's _assignment.
        (List) dropped: list of dropped orders.

    NOTE: from https://github.com/google/or-tools/blob/stable/examples/python/cvrptw_plot.py
    """
    _dropped = []
    for _idx in range(_model.Size()):
        if _assignment.Value(_model.NextVar(_idx)) == _idx:
            _dropped.append(str(_idx))

    _STOP = namedtuple("Stop", ["idx", "lat", "lon", "demand", "dist"])

    _solution = []
    for _route_number in range(_model.vehicles()):
        _route = []
        _idx = _model.Start(_route_number)

        if _model.IsEnd(_assignment.Value(_model.NextVar(_idx))):
            continue

        else:
            _prev_node = _manager.IndexToNode(_idx)

            while True:

                # TODO: time_var = time_dimension.CumulVar(order)
                _node = _manager.IndexToNode(_idx)

                if _node == DEPOT_INDEX:
                    _lat = ORIGINS[0].lat
                    _lon = ORIGINS[0].lon
                else:
                    _lat = DEMANDS[_node - 1].lat
                    _lon = DEMANDS[_node - 1].lon

                _demand = ALL_DEMANDS[_node]
                _dist = DIST_MATRIX[_prev_node][_node] / INT_PRECISION

                _route.append(_STOP(_node, _lat, _lon, _demand, _dist))

                if _model.IsEnd(_idx):
                    break

                _prev_node = _node
                _idx = _assignment.Value(_model.NextVar(_idx))

        _solution.append(_route)

        _str = ""
        for _i, _r in enumerate(_solution):
            _str += f"Route(idx={_i})\n"
            _s = "\n".join("{}: {}".format(*k) for k in enumerate(_r))
            _str += _s + "\n\n"

    return (_solution, _dropped, _str)


if assignment:

    solution, dropped, _str = get_solution(
        _manager=manager, _model=model, _assignment=assignment
    )

    print(f"solution:\n{_str}" + f"dropped:\n{dropped}")
