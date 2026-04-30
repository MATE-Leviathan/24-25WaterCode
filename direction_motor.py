from machine import PWM, Pin


class DirectionMotor:
    def __init__(self, pwm_pin, dir_pin, freq=20000, forward_dir=1):
        self._dir = Pin(dir_pin, Pin.OUT)
        self._pwm = PWM(Pin(pwm_pin))
        self._pwm.freq(freq)
        self._forward_dir = forward_dir
        self._value = 0
        self.off()

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self.set(value)

    def set(self, value):
        value = max(-1.0, min(1.0, float(value)))
        self._value = value

        if value == 0:
            self.off()
            return

        if value > 0:
            self._dir.value(self._forward_dir)
        else:
            self._dir.value(1 - self._forward_dir)

        self._pwm.duty_u16(int(abs(value) * 65535))

    def off(self):
        self._value = 0
        self._pwm.duty_u16(0)
