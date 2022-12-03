import sys

from rqt_gui.main import Main
from ls2n_rqt_delays.delays import Delays


def main():
    plugin = 'ls2n_rqt_delays.delays.Delays'
    main_ = Main(filename=plugin)
    sys.exit(main_.main(standalone=plugin))


if __name__ == '__main__':
    main()
