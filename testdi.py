import math


class DeadbandIntegrator:
    def __init__(self, ki, step_intervals):
        self.ki = ki
        self.step_intervals = step_intervals
        self.clear()

    def clear(self):
        self.integral = 0.0
        self.ramp_count = 0

    def set(self, error):
        self.integral += error * self.ki
        print("input " + str(error) + " integral " + str(self.integral))

        if abs(self.ramp_count) > 0 and (
            abs(self.ramp_count) > self.step_intervals
            or abs(self.integral - math.copysign(1.0, self.ramp_count)) > 2.0
        ):
            print("over thresh, resetting")
            print("step_intervals " + str(self.step_intervals))
            print("other thingy " + str(abs(self.integral - math.copysign(1.0, self.ramp_count))))
            self.ramp_count = 0

        if self.ramp_count == 0 and abs(self.integral) > 1.0:
            print("over thresh, starting")
            self.ramp_count = math.copysign(1, self.integral)
            self.integral -= math.copysign(2.0, self.ramp_count)

        print("ramp_count " + str(self.ramp_count))

        if self.integral < -1:
            self.integral = -1
        if self.integral > 1:
            self.integral = 1

        if abs(self.ramp_count) > 0:
            self.ramp_count += 1
            return math.copysign(1, self.ramp_count)
        else:
            return 0


di = DeadbandIntegrator(1, 5)

for i in range(0, 40):
    print("interval " + str(i) + " returned " + str(di.set(0.2)))
