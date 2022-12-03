import sys

from rqt_gui.main import Main
from ls2n_rqt_drones_observers.drone_observers import DroneObservers


def main():
    plugin = 'ls2n_rqt_drones_observers.drone_observers.DroneObservers'
    main_ = Main(filename=plugin)
    sys.exit(main_.main(standalone=plugin))


if __name__ == '__main__':
    main()
