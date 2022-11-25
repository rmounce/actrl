from collections import deque

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

        self.wma.set((error - self.prev_error) + (target - self.prev_target))
        self.prev_error = error
        self.prev_target = target

    def get(self):
        return self.factor * self.wma.get()



class MyPID:
    def __init__(self, kp, ki, kd, window, clamp_low, clamp_high):
        self.kp = kp
        self.ki = ki
        self.deriv = MyDeriv(window, kd)
        self.clamp_low = clamp_low
        self.clamp_high = clamp_high
        self.clear()

    def clear(self):
        self.deriv.clear()
        self.last_val = 0.0
        self.integral = 0.0

    def set(self, error, target):
        #print(" error " + str(error) + " target " + str(target))
        val = error - target
        self.last_val = val
        self.integral += val
        self.deriv.set(error, target)
        # clamp between +-1
        if not self.clamp_low <= self.get() <= self.clamp_high:
            if self.get() > self.clamp_high:
                clamp_to = self.clamp_high
            else:
                clamp_to = self.clamp_low
            self.integral = (
                -(clamp_to + self.deriv.get() + (self.last_val * self.kp)) / self.ki
            )

    def scale(self, factor):
        current = self.get()
        scaledcurrent = 1.0 - current
        target = 1.0 - (scaledcurrent * factor)
        delta = current - target
        self.integral += delta / self.kp

    def get(self):
        return (
            (-self.last_val * self.kp)
            + (-self.integral * self.ki)
            + (-self.deriv.get())
        )


testpid = MyPID(
                kp=0.1,
                ki=0.1,
                kd=0.1,
                window=5,
                clamp_low=-1.0,
                clamp_high=1.0,
            )

testpid.set(-1, 0)
print(testpid.get())
testpid.set(-1, 0)
print(testpid.get())
testpid.scale(0.5)
print(testpid.get())
print(testpid.get())
