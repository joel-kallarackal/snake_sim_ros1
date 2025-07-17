from copy import deepcopy

import numpy as np

def sin(*args, **kwargs):
    return np.sin(*args, **kwargs)

def compound_serpenoid(t, n, params) -> np.ndarray:
    r"""Vectorized implementation of the compound serpenoid equation

    ...math::
        \alpha(n, t) = \beta_{even} + A_{even}sin(\omega_{t, even}t + \omega_{s, even}n) \forall even n \\
        \alpha(n, t) = \beta_{odd} + A_{odd}sin(\omega_{t, odd} + \omega_{s, odd}n) \forall odd n

    Args:
        params (dict): Dictionary of gait parameters
        t (float): Time
        N (int): Number of modules

    Returns:
        alpha (np.ndarray): Array of target angles for each module
    """

    if n % 2 == 0:
        alpha = params["beta_even"] + params["A_even"] * sin(params["wS_even"] * n - params["wT_even"] * t)
    else:
        alpha = params["beta_odd"] + params["A_odd"] * sin(params["wS_odd"] * n - params["wT_odd"] * t + params["delta"])

    return alpha * (-1) ** np.floor(n / 2)


gait_params = {
    "beta_even": 0.0,
    "beta_odd": 0.0,
    "A_even": 0.2,
    "A_odd": 0.2,
    "wS_even": 1.2,
    "wS_odd": 1.2,
    "wT_even": 1.75,
    "wT_odd": 1.75,
    "delta": -1.57079632679,
    "tightness": 0.4,
    "pole_direction":1}

tightness_min = 0
tightness_max = 1.5
tightness_step = 0.005

def rolling_helix(t=0, params=None, pole_params=None):
    params = {} if params is None else params
    pole_params = {} if pole_params is None else pole_params

    current_gait = "rolling_helix"

    A_transition = 0.35
    A_max = 1.75
    dWs_dAodd = 0.125
    """ We start with spatial frequency being zero. Once the amplitude reaches
        amplitude_transition, we then set spatial frequency according to following line:
             ^
             |                     * * * * * * * * wS_odd
             |                   *
             |     dWs_dAodd   *
             |       --------*
         wS  |       |     *
             |       |   *
             |       | *
             |       *
    wS_min-->|     *
             |     *
             * * * *------------------------------>
                   ^           A_odd
                   |
                A_transition

    The "tightness" of the helix is therefore determined solely by the amplitude. It is
    also possible to adjust wS and the amplitude directly by deriving the Jacobian relating
    the radius of the helix and wS/amplitude. We leave this for future work.
    """

    wS_max = gait_params["wS_even"]  # Assume wS_even to be wS_max from yaml.
    A_min = gait_params["A_even"]

    # Update spatial frequency using commanded tightness.
    if gait_params["tightness"] < A_transition:
        gait_params["wS_odd"] = 0
    else:
        gait_params["wS_odd"] = min(wS_max, (gait_params["tightness"] - A_transition) * dWs_dAodd)

    gait_params["wS_odd"] *= gait_params["pole_direction"]

    # Update amplitude using commanded tightness.
    if gait_params["tightness"] < A_min:
        gait_params["A_odd"] = A_min
    else:
        gait_params["A_odd"] = min(gait_params["tightness"], A_max)

    gait_params["A_odd"] *= gait_params["pole_direction"]

    gait_params["wS_even"] = gait_params["wS_odd"]
    gait_params["A_even"] = gait_params["A_odd"]

    N = 16

    alpha = np.zeros(N)
    for n in range(N):
        alpha[n] = compound_serpenoid(t, n, gait_params)

    return alpha, gait_params

print(rolling_helix())