from multiprocessing import Pool
import numpy as np
import os

from cornering import getLatAccel

g = 9.8


def generateLatAccels(car, velocities, swangles, drs):
    a_lats = []
    yaws = []
    radii = []
    slipAngles = []
    num_workers = os.cpu_count() if os.cpu_count() is not None else 4  # Use all the computers cores and more
    with Pool(num_workers) as p:
        result = p.starmap(getLatAccel, [(car, np.radians(swangle), v, np.radians(0), drs)
                                         for v in velocities for swangle in swangles])
    for r in result:
        if r is not None:
            yaw, a_lat, SAs, moment, Fzs, v = r
            a_lats.append(a_lat)
            yaws.append(yaw)
            radii.append(v ** 2 / abs(a_lat))
            slipAngles.append(SAs)
    return (a_lats, radii)
