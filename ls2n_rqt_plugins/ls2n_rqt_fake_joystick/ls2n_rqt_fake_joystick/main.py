import sys

from rqt_gui.main import Main
from ls2n_rqt_fake_joystick.fake_joystick import FakeJoystick


def main():
    plugin = 'ls2n_rqt_fake_joystick.fake_joystick.FakeJoystick'
    main_ = Main(filename=plugin)
    sys.exit(main_.main(standalone=plugin, plugin_argument_provider=FakeJoystick.add_arguments))


if __name__ == '__main__':
    main()
