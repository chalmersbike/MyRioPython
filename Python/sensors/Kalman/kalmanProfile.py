import numpy as np

from .stateestimator_setting import f, Q
from .stateestimator_setting import himu,  Rimu
from .stateestimator_setting import henc, Renc
from .stateestimator_setting import hrps,  Rrps
from .stateestimator_setting import hgps,  Rgps

# Data Profile
kalmanProfile = {
    'header': 1,
    'offset': 0,

    'N_rps': 3,
    'model': (f, Q),
    'P0': Q,

    'sensors': [
        ( "imu", himu, Rimu ),
        ( "enc", henc, Renc ),
        ( "rps", hrps, Rrps ),
        ( "gps", hgps, Rgps ),
    ],

    #
    # 'skip': lambda row: np.isnan(float(row[2])),
    # 'getTime': lambda row: float(row[0]),
    # 'getControl': lambda row: float(row[9]),
    # 'getIMU': lambda row: np.array([[
    #     float(row[73]),
    #     float(row[74]),
    #     float(row[75]),
    #     float(row[76]),
    #     float(row[77]),
    #     float(row[78]),
    # ]]).T - bimu,
    # 'getENC': lambda row: float(row[79]) - benc,
    # 'getRPS': lambda params, row: 1/crps * float(row[80]) - brps,
    # 'getGPSTime': lambda row: float(row[0]),
    # 'getGPS': lambda params, row: np.array([[
    #     float(row[81]),
    #     float(row[82]),
    #     np.nan,
    #     np.nan,
    # ]]).T - bgps,
    #
    # 'isGroundTruthAvailable': True,
    # 'getGroundTruth': lambda row: np.array([[
    #     float(row[2]),
    #     float(row[3]),
    #     float(row[4]),
    #     float(row[5]),
    #     float(row[6]),
    #     float(row[7]),
    #     float(row[8]),
    # ]]).T,
    # 'getComplementaryFilter': lambda row: np.array([[
    #     float(row[66]),
    #     float(row[67]),
    #     float(row[68]),
    #     float(row[69]),
    #     float(row[70]),
    #     float(row[71]),
    #     float(row[72]),
    # ]]).T,
}
