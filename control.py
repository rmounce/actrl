# Pure (HA-independent) control logic shared by AppDaemon apps and tests.

from collections import deque

# per interval
# 0.2C per minute
# double the rate of statctrl.py, otherwise they cancel each other until the target is reached
grid_surplus_open_window_rate = 0.2 * (10.0 / 60.0)


class MyWMA:
    def __init__(self, window):
        self.window = window
        self.clear()

    def clear(self):
        self.history = deque([])
        self.pad()

    def pad(self):
        while len(self.history) < self.window:
            self.history.appendleft(0.0)
        while len(self.history) > self.window:
            self.history.popleft()

    def set(self, value):
        self.history.append(value)
        self.pad()

    def get(self):
        i = 0
        i_sum = 0
        val_sum = 0.0

        for val in self.history:
            i += 1
            i_sum += i
            val_sum += val * i

        return val_sum / i_sum


# ignores steps due to changed target
class MyDeriv:
    def __init__(self, window, factor):
        self.wma = MyWMA(window=window)
        self.factor = factor
        self.clear()

    def clear(self):
        self.wma.clear()
        self.prev_error = None
        self.prev_target = None

    def set(self, error, target):
        if self.prev_error is None:
            self.prev_error = error
        if self.prev_target is None:
            self.prev_target = target

        error_delta = error - self.prev_error
        target_delta = target - self.prev_target

        # compensate for changes due to target
        actual_delta = error_delta + target_delta

        self.wma.set(actual_delta)
        self.prev_error = error
        self.prev_target = target

    def get(self):
        return self.factor * self.wma.get()


class MyPID:
    def __init__(self, kp, ki, kd, window):
        self.kp = kp
        self.ki = ki
        self.deriv = MyDeriv(window=window, factor=kd)
        self.clear()

    def clear(self):
        """Reset controller state."""
        self.deriv.clear()
        self.p_term = 0.0
        self.i_term = 0.0

    def update(self, error, setpoint):
        """Update PID computation with new error and setpoint values."""
        self.p_term = error * self.kp
        self.i_term += error * self.ki
        self.deriv.set(error, setpoint)

    def get_output(self):
        """Get raw PID output."""
        return self.p_term + self.i_term + self.deriv.get()

    def set_integral(self, value):
        """Directly set the integral term."""
        self.i_term = value

    def adjust_integral(self, adjustment):
        """Apply an adjustment to the integral term."""
        self.i_term += adjustment


class DeadbandIntegrator:
    def __init__(self, ki):
        self.ki = ki
        self.clear()

    def clear(self):
        self.integral = 0.0
        self.increment_count = 0

    def set(self, error):
        if error > 0 and self.increment_count <= 0:
            self.integral = max(0.75, self.integral)
            self.increment_count = 1

        if error < 0 and self.increment_count >= 0:
            self.integral = min(-0.75, self.integral)
            self.increment_count = -1

        self.integral += error * self.ki

        rval = 0

        if self.integral > 1:
            self.integral = min(1, self.integral - 2)
            rval = 1
        elif self.integral < -1:
            self.integral = max(-1, self.integral + 2)
            rval = -1

        # print(
        #    f"input: {error}, integral: {self.integral}, increment_count: {self.increment_count} rval: {rval}"
        # )
        return rval

    def get(self):
        return self.integral


class WindowStateHandler:
    def __init__(self, accumulation_rate=grid_surplus_open_window_rate):
        self.window_offsets = {}
        self.accumulation_rate = accumulation_rate

    def update(self, room, window_open):
        if room not in self.window_offsets:
            self.window_offsets[room] = 0

        if window_open:
            self.window_offsets[room] += self.accumulation_rate
        else:
            self.window_offsets[room] = 0

    def get_offset(self, room):
        return self.window_offsets.get(room, 0)
