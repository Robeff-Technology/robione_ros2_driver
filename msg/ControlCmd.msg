float32 throttle         # 0.0 .. 1.0
float32 brake            # 0.0 .. 1.0
float32 steer            # 1.0 .. -1.0  1.0->full left -1.0->full right
float32 steer_rate       # deg/s   43 deg/s...545 deg/s

uint8 handbrake

uint8 HANDBRAKE_ON=0
uint8 HANDBRAKE_OFF=1

uint8 gear

uint8 GEAR_NEUTRAL=0
uint8 GEAR_DRIVE=1
uint8 GEAR_REVERSE=2

uint8 emergency

uint8 EMERGENCY_OFF=0
uint8 EMERGENCY_ON=1

