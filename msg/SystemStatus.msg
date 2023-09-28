builtin_interfaces/Time stamp

float32 vehicle_current #[A]
float32 brake_left_current #[A]
float32 brake_right_current #[A]

# bit0: battery voltage error
# bit1: supply voltage error
# bit2: controller none
uint16 sys_error_code

# bit0: usb communication error
# bit1: rf controller communication error
# bit2: brake controller communication error
# bit3: steering controller communication error
# bit4: bms communication error
uint16 communicaton_error_code