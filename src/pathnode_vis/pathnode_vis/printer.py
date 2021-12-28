import curses
import datetime
import re


class Printer(object):
    def print(self, lines):
        for s in lines:
            print(s)


class NcursesPrinter(object):
    MODE_STOP = 0
    MODE_RUN = 1
    MODE_REGEXP = 2

    UP = curses.KEY_UP
    DOWN = curses.KEY_DOWN
    PPAGE = curses.KEY_PPAGE
    NPAGE = curses.KEY_NPAGE

    def __init__(self, stdscr):
        stdscr.scrollok(True)
        stdscr.timeout(0)  # set non-blocking
        self.stdscr = stdscr

        size = stdscr.getmaxyx()
        self.y_max = size[0]
        self.x_max = size[1]

        # mode line colors
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_WHITE)
        self.CYAN = curses.color_pair(1)
        self.WHITE = curses.color_pair(2)

        self.mode = self.MODE_RUN

        # showing lines
        self.lines = []
        self.start_line = 0

        # regexp mode
        self.regexp_ptn = ""
        self.search = None

        self.adjust_keys = (
            self.DOWN,
            ord('j'),
            self.NPAGE,
            ord('d'),

            self.UP,
            ord('k'),
            self.PPAGE,
            ord('u'),

            ord('g'),
            ord('G'),
            )

        self.run_keys = (
            ord('q'),
            ord('F')
            )

        self.regex_keys = (
            ord('r'),
            ord('/')
            )

    def adjust_lines(self, k, lines):
        step = 0
        if k in (self.DOWN, ord('j')):
            step = 1
        elif k in (self.NPAGE, ord('d')):
            step = 10
        elif k in (self.UP, ord('k')):
            step = -1
        elif k in (self.PPAGE, ord('u')):
            step = -10

        self.start_line += step

        if k == ord('g'):
            self.start_line = 0
        elif k == ord('G'):
            self.start_line = len(lines) - self.y_max + 2

        if self.start_line < 0:
            self.start_line = 0
        elif len(lines) <= self.start_line:
            self.start_line = len(lines) - 1

    def enter_regex_mode(self):
        self.mode = self.MODE_REGEXP

    def print(self, input_lines):
        stdscr = self.stdscr
        lines = self.lines

        keys = []
        k = stdscr.getch()
        while k >= 0:
            keys.append(k)
            k = stdscr.getch()

        if len(keys) == 0 and self.mode == self.MODE_RUN:
            self.lines = input_lines
            lines = self.lines

        if len(self.regexp_ptn) > 0:
            lines = list(filter(
                lambda x: self.search.search(x),
                lines))

        for k in keys:
            if self.mode == self.MODE_STOP:
                if k in self.run_keys:
                    self.mode = self.MODE_RUN
                    self.lines = input_lines
                elif k in self.adjust_keys:
                    self.adjust_lines(k, lines)
                elif k in self.regex_keys:
                    self.enter_regex_mode()
                else:
                    return
            elif self.mode == self.MODE_RUN:
                if k in self.adjust_keys:
                    self.adjust_lines(k, lines)
                    self.mode = self.MODE_STOP
                elif k in self.regex_keys:
                    self.enter_regex_mode()
                else:
                    # update displayed lines
                    self.lines = input_lines
                    lines = self.lines
            elif self.mode == self.MODE_REGEXP:
                if ord(' ') <= k <= ord('~'):
                    self.regexp_ptn += chr(k)
                    self.search = re.compile(self.regexp_ptn)
                    self.start_line = 0
                elif k == curses.KEY_BACKSPACE:
                    self.regexp_ptn = self.regexp_ptn[:-1]
                    self.search = re.compile(self.regexp_ptn)
                    self.start_line = 0
                elif k in (curses.KEY_ENTER, ord('\n')):
                    self.mode = self.MODE_RUN
            else:
                raise Exception("unknown mode")

        stdscr.clear()

        self.print_mode()
        for i, s in enumerate(lines[self.start_line:]):
            if i + 1 == self.y_max - 1:
                break

            if s[-1] != "\n":
                s += "\n"
            lineno = self.start_line + i
            stdscr.addstr(i+1, 0, f"{lineno:<3}| ")
            stdscr.addstr(i+1, 5, s)
        stdscr.refresh()

    def print_all(self, stdscr, lines):
        i = 1
        for s in lines[self.start_line:]:
            if i == self.y_max - 1:
                break

            if s[-1] != "\n":
                s += "\n"
            lineno = self.start_line + i
            stdscr.addstr(i, 0, f"{lineno:<3}| ")
            stdscr.addstr(i, 5, s)
            i += 1

    def print_filtered(self, stdscr, lines):
        i = 1
        for s in lines[self.start_line:]:
            if i == self.y_max - 1:
                break

            if not self.search.search(s):
                continue

            if s[-1] != "\n":
                s += "\n"
            lineno = self.start_line + i
            stdscr.addstr(i, 0, f"{lineno:<3}| ")
            stdscr.addstr(i, 5, s)
            i += 1

    def print_mode(self):
        stdscr = self.stdscr
        color = self.CYAN

        t = datetime.datetime.now().strftime('%Y/%m/%d %H:%M:%S')

        mode_str = ""
        if self.mode == self.MODE_RUN:
            mode_str = \
                "press UP/DOWN/PgUp/PgDown/jkdugG to scroll," + \
                "r or '/' to filter"
            color = self.WHITE
        elif self.mode == self.MODE_STOP:
            mode_str = "scrolling... Press q/F to periodically update"
            color = self.CYAN
        elif self.mode == self.MODE_REGEXP:
            mode_str = "Regexp: " + self.regexp_ptn + "_"
            color = self.CYAN

        s = mode_str
        s += " " * (self.x_max - len(mode_str) - len(t))
        s += t

        stdscr.addstr(0, 0, s, color)
