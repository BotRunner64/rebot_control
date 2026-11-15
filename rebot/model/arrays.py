"""Small input checks shared by the pure-math modules."""

import numpy as np


def vector6(value, name):
    """A fresh float array of six finite values, one per arm joint."""
    result = np.array(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain six finite values.")
    return result
