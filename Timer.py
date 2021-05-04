from time import time
from time import strftime as stime


class timer():
    def __init__(self):
        self.origin = time()
        self.pred = self.origin
        self.curr = self.origin

    def lap(self):
        self.curr = time()
        elapsed = round(self.curr - self.pred, 2)
        self.pred = time()
        return str(elapsed)

    def empty_lap(self):
        self.cur = time()

    def print_elapsed(self):
        return (self.lap() + "s (global: " + str(round(self.global_time(), 2)) + ")")

    def reset(self):
        self.origin = time()
        self.pred = self.origin
        self.curr = self.origin

    def global_time(self):
        return time() - self.origin