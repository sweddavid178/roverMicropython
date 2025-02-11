from machine import Timer, Pin
from array import array
from utime import ticks_us, ticks_diff, sleep_ms
import time

# from micropython import alloc_emergency_exception_buf
# alloc_emergency_exception_buf(100)


# On 1st edge start a block timer. While the timer is running, record the time
# of each edge. When the timer times out decode the data. Duration must exceed
# the worst case block transmission time, but be less than the interval between
# a block start and a repeat code start (~108ms depending on protocol)


class IR_RX:
    Timer_id = -1  # Software timer but enable override
    # Result/error codes
    # Repeat button code
    REPEAT = -1
    # Error codes
    BADSTART = -2
    BADBLOCK = -3
    BADREP = -4
    OVERRUN = -5
    BADDATA = -6
    BADADDR = -7

    def __init__(self, pin, nedges, tblock, callback, *args):  # Optional args for callback
        self._pin = pin
        self._nedges = nedges
        self._tblock = tblock
        self.callback = callback
        self.args = args
        self._errf = lambda _: None
        self.verbose = False

        self._times = array("i", (0 for _ in range(nedges + 1)))  # +1 for overrun
        pin.irq(handler=self._cb_pin, trigger=(Pin.IRQ_FALLING | Pin.IRQ_RISING))
        self.edge = 0
        self.tim = Timer(self.Timer_id)  # Defaul is sofware timer
        self.cb = self.decode

    # Pin interrupt. Save time of each edge for later decode.
    def _cb_pin(self, line):
        t = ticks_us()
        # On overrun ignore pulses until software timer times out
        if self.edge <= self._nedges:  # Allow 1 extra pulse to record overrun
            if not self.edge:  # First edge received
                self.tim.init(period=self._tblock, mode=Timer.ONE_SHOT, callback=self.cb)
            self._times[self.edge] = t
            self.edge += 1

    def do_callback(self, cmd, addr, ext, thresh=0):
        self.edge = 0
        if cmd >= thresh:
            self.callback(cmd, addr, ext, *self.args)
        else:
            self._errf(cmd)

    def error_function(self, func):
        self._errf = func

    def close(self):
        self._pin.irq(handler=None)
        self.tim.deinit()
        
class NEC_ABC(IR_RX):
    def __init__(self, pin, extended, samsung, callback, *args):
        # Block lasts <= 80ms (extended mode) and has 68 edges
        super().__init__(pin, 68, 80, callback, *args)
        self._extended = extended
        self._addr = 0
        self._leader = 2500 if samsung else 4000  # 4.5ms for Samsung else 9ms

    def decode(self, _):
        try:
            if self.edge > 68:
                raise RuntimeError(self.OVERRUN)
            width = ticks_diff(self._times[1], self._times[0])
            if width < self._leader:  # 9ms leading mark for all valid data
                raise RuntimeError(self.BADSTART)
            width = ticks_diff(self._times[2], self._times[1])
            if width > 3000:  # 4.5ms space for normal data
                if self.edge < 68:  # Haven't received the correct number of edges
                    raise RuntimeError(self.BADBLOCK)
                # Time spaces only (marks are always 562.5µs)
                # Space is 1.6875ms (1) or 562.5µs (0)
                # Skip last bit which is always 1
                val = 0
                for edge in range(3, 68 - 2, 2):
                    val >>= 1
                    if ticks_diff(self._times[edge + 1], self._times[edge]) > 1120:
                        val |= 0x80000000
            elif width > 1700: # 2.5ms space for a repeat code. Should have exactly 4 edges.
                raise RuntimeError(self.REPEAT if self.edge == 4 else self.BADREP)  # Treat REPEAT as error.
            else:
                raise RuntimeError(self.BADSTART)
            addr = val & 0xff  # 8 bit addr
            cmd = (val >> 16) & 0xff
            if cmd != (val >> 24) ^ 0xff:
                raise RuntimeError(self.BADDATA)
            if addr != ((val >> 8) ^ 0xff) & 0xff:  # 8 bit addr doesn't match check
                if not self._extended:
                    raise RuntimeError(self.BADADDR)
                addr |= val & 0xff00  # pass assumed 16 bit address to callback
            self._addr = addr
        except RuntimeError as e:
            cmd = e.args[0]
            addr = self._addr if cmd == self.REPEAT else 0  # REPEAT uses last address
        # Set up for new data burst and run user callback
        self.do_callback(cmd, addr, 0, self.REPEAT)

class NEC_8(NEC_ABC):
    def __init__(self, pin, callback, *args):
        super().__init__(pin, False, False, callback, *args)

class NEC_16(NEC_ABC):
    def __init__(self, pin, callback, *args):
        super().__init__(pin, True, False, callback, *args)

class SAMSUNG(NEC_ABC):
    def __init__(self, pin, callback, *args):
        super().__init__(pin, True, True, callback, *args)

class IR_GET(IR_RX):
    def __init__(self, pin, nedges=100, twait=100, display=True):
        self.display = display
        super().__init__(pin, nedges, twait, lambda *_ : None)
        self.data = None

    def decode(self, _):
        def near(v, target):
            return target * 0.8 < v < target * 1.2
        lb = self.edge - 1  # Possible length of burst
        if lb < 3:
            return  # Noise
        burst = []
        for x in range(lb):
            dt = ticks_diff(self._times[x + 1], self._times[x])
            if x > 0 and dt > 10000:  # Reached gap between repeats
                break
            burst.append(dt)
        lb = len(burst)  # Actual length
        # Duration of pulse train 24892 for RC-5 22205 for RC-6
        duration = ticks_diff(self._times[lb - 1], self._times[0])

        if self.display:
            for x, e in enumerate(burst):
                print('{:03d} {:5d}'.format(x, e))
            print()
            # Attempt to determine protocol
            ok = False  # Protocol not yet found
            if near(burst[0], 9000) and lb == 67:
                print('NEC')
                ok = True

            if not ok and near(burst[0], 2400) and near(burst[1], 600):  # Maybe Sony
                try:
                    nbits = {25:12, 31:15, 41:20}[lb]
                except KeyError:
                    pass
                else:
                    ok = True
                    print('Sony {}bit'.format(nbits))

            if not ok and near(burst[0], 889):  # Maybe RC-5
                if near(duration, 24892) and near(max(burst), 1778):
                    print('Philps RC-5')
                    ok = True

            if not ok and near(burst[0], 2666) and near(burst[1], 889):  # RC-6?
                if near(duration, 22205) and near(burst[1], 889) and near(burst[2], 444):
                    print('Philips RC-6 mode 0')
                    ok = True

            if not ok and near(burst[0], 2000) and near(burst[1], 1000):
                if near(duration, 19000):
                    print('Microsoft MCE edition protocol.')
                    # Constant duration, variable burst length, presumably bi-phase
                    print('Protocol start {} {} Burst length {} duration {}'.format(burst[0], burst[1], lb, duration))
                    ok = True

            if not ok and near(burst[0], 4500) and near(burst[1], 4500) and lb == 67:  # Samsung
                print('Samsung')
                ok = True

            if not ok and near(burst[0], 3500) and near(burst[1], 1680):  # Panasonic?
                print('Unsupported protocol. Panasonic?')
                ok = True

            if not ok:
                print('Unknown protocol start {} {} Burst length {} duration {}'.format(burst[0], burst[1], lb, duration))

            print()
        self.data = burst
        # Set up for new data burst. Run null callback
        self.do_callback(0, 0, 0)

    def acquire(self):
        while self.data is None:
            sleep_ms(5)
        self.close()
        return self.data

def test():
    # Define pin according to platform
    pin = Pin(18, Pin.IN)
    irg = IR_GET(pin)
    print('Waiting for IR data...')
    return irg.acquire()


print("ir test")

test()

def callback(data, addr, ctrl):
    if data < 0:  # NEC protocol sends repeat codes.
        print('Repeat code.')
    else:
        print('Data {:02x} Addr {:04x}'.format(data, addr))

ir = NEC_8(Pin(18, Pin.IN), callback)
red = Pin(22, Pin.OUT)
while True:
    time.sleep_ms(500)
    red.on()
    time.sleep_ms(500)
    red.off()