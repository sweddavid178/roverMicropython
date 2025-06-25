from micropython import const
from array import array
from utime import ticks_us, ticks_diff, sleep_ms
from machine import Pin, PWM, Timer
from esp32 import RMT
import time

_TBURST = const(563)
_T_ONE = const(1687)
# Shared by NEC
STOP = const(0)  # End of data

# IR abstract base class. Array holds periods in μs between toggling 36/38KHz
# carrier on or off. Physical transmission occurs in an ISR context controlled
# by timer 2 and timer 5. See TRANSMITTER.md for details of operation.
class IR:
    _active_high = True  # Hardware turns IRLED on if pin goes high.
    _space = 0  # Duty ratio that causes IRLED to be off
    timeit = False  # Print timing info

    def __init__(self, pin, cfreq, asize, duty, verbose):
        self._rmt = RMT(0, pin=pin, clock_div=80, tx_carrier = (cfreq, duty, 1))
        self._tcb = self._cb  # Pre-allocate
        self._arr = array('H', 0 for _ in range(asize))  # on/off times (μs)
        self._mva = memoryview(self._arr)
        # Subclass interface
        self.verbose = verbose
        self.carrier = False  # Notional carrier state while encoding biphase
        self.aptr = 0  # Index into array
        self._busy = False

    def _cb(self, t):  # T5 callback, generate a carrier mark or space
        self._busy = True
        t.deinit()
        p = self.aptr
        v = self._arr[p]
        if v == STOP:
            self._ch.pulse_width_percent(self._space)  # Turn off IR LED.
            self._busy = False
            return
        self._ch.pulse_width_percent(self._space if p & 1 else self._duty)
        self._tim.init(prescaler=84, period=v, callback=self._tcb)
        self.aptr += 1

    def busy(self):
        return not self._rmt.wait_done()

    # Public interface
    # Before populating array, zero pointer, set notional carrier state (off).
    def transmit(self, addr, data, toggle=0, validate=False):  # NEC: toggle is unused
        while self.busy():
            pass
        t = ticks_us()
        if validate:
            if addr > self.valid[0] or addr < 0:
                raise ValueError('Address out of range', addr)
            if data > self.valid[1] or data < 0:
                raise ValueError('Data out of range', data)
            if toggle > self.valid[2] or toggle < 0:
                raise ValueError('Toggle out of range', toggle)
        self.aptr = 0  # Inital conditions for tx: index into array
        self.carrier = False
        self.tx(addr, data, toggle)  # Subclass populates ._arr
        self.trigger()  # Initiate transmission
        if self.timeit:
            dt = ticks_diff(ticks_us(), t)
            print('Time = {}μs'.format(dt))
        sleep_ms(1)  # Ensure ._busy is set prior to return

    # Subclass interface
    def trigger(self):  # Used by NEC to initiate a repeat frame
        self._rmt.write_pulses(tuple(self._mva[0 : self.aptr]))
        
    def append(self, *times):  # Append one or more time peiods to ._arr
        for t in times:
            self._arr[self.aptr] = t
            self.aptr += 1
            self.carrier = not self.carrier  # Keep track of carrier state
            self.verbose and print('append', t, 'carrier', self.carrier)

    def add(self, t):  # Increase last time value (for biphase)
        assert t > 0
        self.verbose and print('add', t)
        # .carrier unaffected
        self._arr[self.aptr - 1] += t


# Given an iterable (e.g. list or tuple) of times, emit it as an IR stream.
class Player(IR):

    def __init__(self, pin, freq=38000, verbose=False, asize=68):  # NEC specifies 38KHz
        super().__init__(pin, freq, asize, 33, verbose)  # Measured duty ratio 33%

    def play(self, lst):
        for x, t in enumerate(lst):
            self._arr[x] = t
        self.aptr = x + 1
        self.trigger()

class NEC(IR):
    valid = (0xffff, 0xff, 0)  # Max addr, data, toggle
    samsung = False

    def __init__(self, pin, freq=38000, verbose=False):  # NEC specifies 38KHz also Samsung
        super().__init__(pin, freq, 68, 33, verbose)  # Measured duty ratio 33%

    def _bit(self, b):
        self.append(_TBURST, _T_ONE if b else _TBURST)

    def tx(self, addr, data, _):  # Ignore toggle
        if self.samsung:
            self.append(4500, 4500)
        else:
            self.append(9000, 4500)
        if addr < 256:  # Short address: append complement
            if self.samsung:
              addr |= addr << 8
            else:
              addr |= ((addr ^ 0xff) << 8)
        for _ in range(16):
            self._bit(addr & 1)
            addr >>= 1
        data |= ((data ^ 0xff) << 8)
        for _ in range(16):
            self._bit(data & 1)
            data >>= 1
        self.append(_TBURST)

    def repeat(self):
        self.aptr = 0
        self.append(9000, 2250, _TBURST)
        self.trigger()  # Initiate physical transmission.

class IR_RX:
    Timer_id = 0  # Software timer but enable override
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
            if self.edge > 70:
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
            """if cmd != (val >> 24) ^ 0xff:
                raise RuntimeError(self.BADDATA)
            if addr != ((val >> 8) ^ 0xff) & 0xff:  # 8 bit addr doesn't match check
                if not self._extended:
                    raise RuntimeError(self.BADADDR)
                addr |= val & 0xff00  # pass assumed 16 bit address to callback"""
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

#test()

lastSendAddr = -1
lastRXData = -1
lastRXAddr = -1
def callback(data, addr, ctrl):
    global lastSendAddr,lastRXData,lastRXAddr
    if data < 0:  # NEC protocol sends repeat codes.
        print('Repeat code.')
    elif addr == lastSendAddr:
        #print("ignoring")
        lastSendAddr = -1
        return
    else:
        lastRXData = data
        lastRXAddr = addr
        print('Data {:02x} Addr {:04x}'.format(data, addr))

ir = NEC_8(Pin(18, Pin.IN), callback)

pin = Pin(19, Pin.OUT)
irTransmit = NEC(pin)

def IR_send_message(addr, data):
    global lastSendAddr
    lastSendAddr = addr
    irTransmit.transmit(addr,data)
    
"""
returns data,address: both will return -1
if no new message has been received
"""
def IR_get_last_rx_message():
    global lastSendAddr,lastRXData,lastRXAddr
    tempData = lastRXData
    tempAddr = lastRXAddr
    lastRXData = -1
    lastRXAddr = -1
    return (tempData, tempAddr)
    
"""while True:
    
    print("txing")
    #IR_send_message(20,50)
    time.sleep(1)
    
    print(IR_get_last_rx_message())
    irTransmit.transmit(21,51)
    time.sleep(1)
    print(IR_get_last_rx_message())
    
    #irTransmit.repeat()"""