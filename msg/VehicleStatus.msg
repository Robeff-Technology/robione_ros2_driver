builtin_interfaces/Time stamp
uint32 message_count

float32 velocity # [m/s] if direction reverse, speed val is negative

float32 steer_angle # degree

float32 throttle

uint8 handbrake

uint8 HANDBRAKE_ON=0
uint8 HANDBRAKE_OFF=1

uint8 gear

uint8 GEAR_NEUTRAL=0
uint8 GEAR_DRIVE=1
uint8 GEAR_REVERSE=2


uint8 intervention

uint8 NO_INTERVENTION=0
uint8 INTERVENTION=1

uint8 ready

uint8 NOT_READY=0
uint8 READY=1



uint8 mode

uint8 SET_STEER_RATE = 1
uint8 NO_STEER_RATE = 0

uint8 emergency

uint8 EMERGENCY_OFF=0
uint8 EMERGENCY_ON=1

