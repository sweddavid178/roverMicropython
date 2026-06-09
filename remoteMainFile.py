import time

# analog id -> human name
ANALOG_MAP = {
    1:  "right_x",   # right joystick X
    2:  "right_y",   # right joystick Y
    3:  "left_x",    # left joystick X
    6:  "left_y",    # left joystick Y
    10: "voltage",   # battery voltage / analog sensor
}

# Create a dense list (index == analog id). Unassigned entries default to 0.
_max_analog_id = max(ANALOG_MAP.keys())
analog_inputs = [0] * (_max_analog_id + 1)

# Fill in known analogs. Replace the 0 values with actual ADC-like objects or callables,
# e.g. analog_inputs[1] = machine.ADC(0) or analog_inputs[10] = lambda: read_battery_mv()
for _id, _name in ANALOG_MAP.items():
    analog_inputs[_id] = 0  # placeholder; substitute with actual ADC/callable as needed

# optional: a human-readable mapping for debugging
analog_names = {i: ANALOG_MAP[i] for i in ANALOG_MAP}

# button id -> human name
BUTTON_MAP = {
    7:  "left_trigger",
    5:  "left_button",
    8:  "right_button",
    9:  "btn0_left_down",
    10: "btn1_missing",
    11: "btn2",
    12: "btn3",
    13: "btn4",
    14: "btn5",
    15: "btn6_left_up",
    16: "btn7",
    46: "left_middle",
    40: "right_trigger",
    37: "right_up",
    36: "right_middle",
    35: "right_down",
    39: "right_thumb",
    47: "left_thumb",
}

# Create a dense list (index == button id).  Unassigned entries default to 0.
_max_id = max(BUTTON_MAP.keys())
button_inputs = [0] * (_max_id + 1)

# Fill in known buttons. Replace the 0 values with actual Pin-like objects or callables,
# e.g. button_inputs[7] = machine.Pin(5, machine.Pin.IN) or button_inputs[9] = lambda: read_gpio(9)
for _id, _name in BUTTON_MAP.items():
    button_inputs[_id] = 0  # placeholder; substitute with actual pin/callable as needed

# optional: a human-readable mapping for debugging
button_names = {i: BUTTON_MAP[i] for i in BUTTON_MAP}


def read_all_button_states(button_inputs):
    """
    Read states for all buttons and return a comma-separated string of 0/1 values.

    button_inputs can be:
    - an ordered iterable (list/tuple) of Pin-like objects or callables,
    - a mapping (dict or dict-like) of id->Pin-like or id->callable (will be sorted by key).

    A "Pin-like" object is anything with a .value() method (e.g. machine.Pin).
    A callable is called with no arguments and should return a truthy/falsy value.
    """
    def _read(src):
        # Pin-like
        if hasattr(src, "value") and callable(getattr(src, "value")):
            try:
                return int(bool(src.value()))
            except Exception:
                return 0
        # callable
        if callable(src):
            try:
                return int(bool(src()))
            except Exception:
                return 0
        # already a numeric/boolean value
        try:
            return int(bool(src))
        except Exception:
            return 0

    values = []
    # treat mapping-like objects that expose .items() as mappings
    if hasattr(button_inputs, "items"):
        for key in sorted(button_inputs):
            values.append(str(_read(button_inputs[key])))
    else:
        for src in button_inputs:
            values.append(str(_read(src)))

    return ",".join(values)


def read_all_analog_values(analog_inputs):
    """
    Read values for all analog sticks and return a comma-separated string of integer values.

    analog_inputs can be:
    - an ordered iterable (list/tuple) of ADC-like objects, callables, or numeric values,
    - a mapping (dict or dict-like) of id->ADC-like or id->callable (will be sorted by key).

    Supported read methods (in order): read_u16(), read(), value(), callable().
    Falls back to treating the source as a numeric value. Any failures return 0 for that entry.
    """
    def _read(src):
        try:
            # ADC with 16-bit read
            if hasattr(src, "read_u16") and callable(getattr(src, "read_u16")):
                return int(src.read_u16())
            # Common MicroPython ADC read
            if hasattr(src, "read") and callable(getattr(src, "read")):
                return int(src.read())
            # Pin-like value method
            if hasattr(src, "value") and callable(getattr(src, "value")):
                return int(src.value())
            # Callable provider
            if callable(src):
                return int(src())
            # Already a numeric/boolean value
            return int(src)
        except Exception:
            return 0

    values = []
    if hasattr(analog_inputs, "items"):
        for key in sorted(analog_inputs):
            values.append(str(_read(analog_inputs[key])))
    else:
        for src in analog_inputs:
            values.append(str(_read(src)))

    return ",".join(values)


if __name__ == "__main__":

    try:
        while True:
            buttons = read_all_button_states(BUTTON_MAP)
            analogs = read_all_analog_values(ANALOG_MAP)
            out = buttons + ("," + analogs if buttons and analogs else analogs if not buttons else "")
            print(out)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass