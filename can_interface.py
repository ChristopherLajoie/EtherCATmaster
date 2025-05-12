import can
import time
import atexit
import signal
import os


BUTTONS_ID = 0x18A
MOVES_ID = 0x28A
LED_ID = 0x30A
MONITOR_ID = 0x40B

global_bus = None


def emergency_exit_handler(signum, frame):
    print(f"Python received signal {signum}, forcing exit")
    os._exit(1)


signal.signal(signal.SIGTERM, emergency_exit_handler)


def initialize_can_bus():
    global global_bus

    try:
        bus = can.interface.Bus(channel='can0', interface='socketcan',
                                receive_own_messages=False, fd=False)
        global_bus = bus
        return bus
    except can.CanError as e:
        print(f"CAN Error: {e}")
        return None
    except Exception as e:
        print(f"Error initializing CAN bus: {e}")
        return None


def shutdown_can_bus():

    global global_bus

    if global_bus:
        try:
            global_bus.shutdown()
            global_bus = None

        except Exception as e:
            print(f"Python: Error shutting down CAN bus: {e}")


atexit.register(shutdown_can_bus)


def send_can_message(bus, arbitration_id, data):
    """Send a CAN message to the specified arbitration ID with the provided data."""
    if bus is None:
        print("Error: CAN bus not initialized.")
        return False

    if len(data) != 8:
        if isinstance(data, bytearray) and len(data) < 8:
            data.extend([0] * (8 - len(data)))
        else:
            print("Error: Data must be exactly 8 bytes.")
            return False

    message = can.Message(arbitration_id=arbitration_id,
                          data=data, is_extended_id=False)

    try:
        bus.send(message)
        return True
    except can.CanError as e:
        print(f"Message NOT sent: {e}")
        return False


def receive_can_message(bus, target_can_id, timeout=1.0):
    """Receive a message for a specific CAN ID and return the data."""
    bus.set_filters(
        [{"can_id": target_can_id, "can_mask": 0x7FF, "extended": False}])

    try:
        message = bus.recv(timeout=timeout)
        if message and message.arbitration_id == target_can_id:
            return message.data
        return None
    except can.CanError as e:
        print(f"Error receiving message: {e}")
        return None


def extract_bit(data, bit_position):
    """Extract a single bit from a byte array based on bit position."""
    byte_index = bit_position // 8
    bit_index = bit_position % 8

    if byte_index >= len(data):
        return 0

    return (data[byte_index] >> bit_index) & 1


def extract_value(data, start_bit, bit_length):
    """Extract a multi-bit value from a byte array."""
    result = 0
    for i in range(bit_length):
        bit_pos = start_bit + i
        byte_index = bit_pos // 8
        bit_index = bit_pos % 8

        if byte_index < len(data):
            bit_value = (data[byte_index] >> bit_index) & 1
            result |= (bit_value << i)

    return result

# ---- BUTTONS Block Functions (0x18A) ----


def get_enable(bus):
    """Get the Enable bit (bit 6) from BUTTONS block."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return extract_bit(data, 6)
    return None


def get_speed(bus):
    """Get the Speed bit (bit 14) from BUTTONS block."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return extract_bit(data, 14)
    return None


def get_horn(bus):
    """Get the Horn bit (bit 16) from BUTTONS block."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return extract_bit(data, 16)
    return None


def get_can_enable(bus):
    """Get the CAN_Enable bit (bit 59) from BUTTONS block."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return extract_bit(data, 59)
    return None


def get_estop(bus):
    """Get the Estop bit (bit 63) from BUTTONS block."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return extract_bit(data, 63)
    return None


def get_all_buttons(bus):
    """Get all button states in one call."""
    data = receive_can_message(bus, BUTTONS_ID)
    if data:
        return {
            "enable": extract_bit(data, 6),
            "speed": extract_bit(data, 14),
            "horn": extract_bit(data, 16),
            "can_enable": extract_bit(data, 59),
            "estop": extract_bit(data, 63)
        }
    return None

# ---- MOVES Block Functions (0x28A) ----


def get_y_axis(bus):
    """Get the Y_Axis value (bits 0-7) from MOVES block."""
    data = receive_can_message(bus, MOVES_ID)
    if data:
        return data[0]  # First byte contains Y_Axis (8 bits starting at bit 0)
    return None


def get_x_axis(bus):
    """Get the X_Axis value (bits 8-15) from MOVES block."""
    data = receive_can_message(bus, MOVES_ID)
    if data:
        # Second byte contains X_Axis (8 bits starting at bit 8)
        return data[1]
    return None


def get_all_moves(bus):
    """Get all movement values in one call."""
    data = receive_can_message(bus, MOVES_ID)
    if data:
        return {
            "y_axis": data[0],
            "x_axis": data[1]
        }
    return None

# ---- LED Block Functions (0x30A) ----


def set_yellow_bat_led(bus, state):
    """Set the Yellow_Bat_Led (bit 0) in LED block."""
    data = bytearray(8)

    if state:
        data[0] |= (1 << 0)

    send_can_message(bus, LED_ID, data)


def set_red_bat_led(bus, state):
    """Set the Red_Bat_Led (bit 1) in LED block."""
    data = bytearray(8)
    if state:
        data[0] |= (1 << 1)
    send_can_message(bus, LED_ID, data)


def set_overload_led(bus, state):
    """Set the Overload_Led (bit 2) in LED block."""
    data = bytearray(8)
    if state:
        data[0] |= (1 << 2)
    send_can_message(bus, LED_ID, data)


def set_aux_led(bus, state):
    """Set the Aux_Led (bit 3) in LED block."""
    data = bytearray(8)
    if state:
        data[0] |= (1 << 3)
    send_can_message(bus, LED_ID, data)

# ---- Testing ----


def monitor_can_values(bus, interval=0.1, duration=None):
    """Monitor key CAN values from all blocks for specified duration."""
    start_time = time.time()

    while True:

        if duration and (time.time() - start_time) > duration:
            break

        buttons = get_all_buttons(bus)
        moves = get_all_moves(bus)

        if buttons:
            print(f"BUTTONS: Enable={buttons['enable']}, Speed={buttons['speed']}, "
                  f"Horn={buttons['horn']}, CAN_Enable={buttons['can_enable']}, "
                  f"Estop={buttons['estop']}")

        if moves:
            print(f"MOVES: Y_Axis={moves['y_axis']}, X_Axis={moves['x_axis']}")

        time.sleep(interval)


def main():
    bus = initialize_can_bus()

    if bus is None:
        print("Error: Could not initialize CAN bus.")
        return
    set_yellow_bat_led(bus, False)

    # monitor_can_values(bus, interval=0.05, duration=100)  # Monitor for 10 seconds
if __name__ == "__main__":
    main()
