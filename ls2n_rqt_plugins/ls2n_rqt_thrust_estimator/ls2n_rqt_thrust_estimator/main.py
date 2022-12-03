import sys

from rqt_gui.main import Main
from ls2n_rqt_thrust_estimator.thrust_estimator import ThrustEstimator


def main():
    plugin = 'ls2n_rqt_thrust_estimator.thrust_estimator.ThrustEstimator'
    main_ = Main(filename=plugin)
    sys.exit(main_.main(standalone=plugin))


if __name__ == '__main__':
    main()
