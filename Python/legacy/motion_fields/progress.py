"""Progress reporting that degrades gracefully when stdout is not a terminal."""

import sys

IS_TTY = sys.stdout.isatty()


def interval(tty_step, total, fraction=10):
    """Report often on a terminal, sparsely when redirected to a file."""
    return tty_step if IS_TTY else max(tty_step, total // fraction or 1)


def line(msg):
    """Overwrite the current line on a terminal, print a new one otherwise."""
    print(msg, end='\r' if IS_TTY else '\n', flush=True)


def end():
    if IS_TTY:
        print()
