builtin_interfaces/Time stamp

float32 battery_voltage # [V]
float32 supply_voltage # [V]

float32 usb_comm_period # [Hz]
float32 brake_status_period #[Hz]
float32 brake_encoder_period #[Hz]
float32 rf_controller_period #[Hz]
float32 steer_controller_period #[Hz]
float32 bms_period #[Hz]


float32 battery_temperature1 # [°C]
float32 battery_temperature2 # [°C]

uint8 charge_status

uint8 CHARGING = 1
uint8 NOT_CHARGING = 0

uint8 soc # [%]