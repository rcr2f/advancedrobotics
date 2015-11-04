from __future__ import absolute_import, division, print_function

from math import pi


def dispatch(user_input, xyz, rpy, xyz_delta=0.004, rpy_delta=5 / 180 * pi):
    p = [xyz.x, xyz.y, xyz.z]
    o = list(rpy)

    mapping = {'w': (p, 0, xyz_delta),
               's': (p, 0, -xyz_delta),
               'a': (p, 1, xyz_delta),
               'd': (p, 1, -xyz_delta),
               '1': (p, 2, -xyz_delta),
               '2': (p, 2, xyz_delta),
               'j': (o, 0, -rpy_delta),
               'l': (o, 0, rpy_delta),
               'i': (o, 1, -rpy_delta),
               'k': (o, 1, rpy_delta),
               '3': (o, 2, rpy_delta),
               '4': (o, 2, -rpy_delta),
               }

    try:
        obj, idx, delta = mapping[user_input]
        obj[idx] += delta
    except KeyError:
        pass
    return p, o


class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()
