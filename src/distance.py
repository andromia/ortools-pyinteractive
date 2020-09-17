from typing import List, Tuple
from haversine import haversine_vector, Unit
import numpy as np


def create_vectorized_haversine_li(
    _olat: float,
    _olon: float,
    _dlats: List[float],
    _dlons: List[float],
) -> List[float]:
    assert len(_dlats) == len(_dlons)

    _olats: List[float] = [_olat] * len(_dlats)
    _olons: List[float] = [_olon] * len(_dlons)
    _os: List[Tuple[float, float]] = list(zip(_olats, _olons))
    _ds: List[Tuple[float, float]] = list(zip(_dlats, _dlons))

    _ds: List[float] = haversine_vector(_os, _ds, unit=Unit.MILES)

    return _ds


def create_matrix(
    _origin: Tuple[float, float],
    _dests: Tuple[float, float, int],
    int_precision: int = 100,
) -> List[List[int]]:
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

        _idistances: List[int] = np.ceil(_fdistances * int_precision).astype(int)
        _matrix.append(_idistances)

    return _matrix
