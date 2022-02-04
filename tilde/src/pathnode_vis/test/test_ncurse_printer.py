#!/usr/bin/env python3

import curses
import sys
import time

from pathnode_vis.printer import (
    NcursesPrinter
    )


def main(stdscr, args=None):
    printer = NcursesPrinter(stdscr)
    while True:
        now = time.time()
        lines = [f"{now}: {i}" for i in range(100)]
        printer.print(lines)
        time.sleep(1)


if __name__ == '__main__':
    curses.wrapper(main, args=sys.argv)
