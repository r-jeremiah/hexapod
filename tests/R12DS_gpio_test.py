import pigpio
import time

# Define GPIO pins connected to PWM channels from Radiolink
PWM_INPUT_PINS = {
    'ch1': 17,
    'ch2': 27,
    'ch5': 22,
}

class PWMReader:
    def __init__(self, pi):
        self.pi = pi
        self.pulse_widths = {ch: 1500 for ch in PWM_INPUT_PINS}  # default mid-point

        for ch, pin in PWM_INPUT_PINS.items():
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_OFF)
            self.pi.callback(pin, pigpio.EITHER_EDGE, self._callback_wrapper(ch))

    def _callback_wrapper(self, channel):
        def callback_func(gpio, level, tick):
            if level == 1:
                self.start_tick[channel] = tick
            elif level == 0:
                if channel in self.start_tick:
                    pulse_width = pigpio.tickDiff(self.start_tick[channel], tick)
                    self.pulse_widths[channel] = pulse_width
        self.start_tick = {}
        return callback_func

    def get_pulse_widths(self):
        return self.pulse_widths

# Initialize
pi = pigpio.pi()
reader = PWMReader(pi)

try:
    while True:
        widths = reader.get_pulse_widths()
        print("PWM Pulse Widths (us):", widths)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    pi.stop()
