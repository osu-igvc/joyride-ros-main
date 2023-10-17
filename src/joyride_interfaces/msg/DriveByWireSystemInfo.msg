


bool is_charging
bool estop_pressed
bool ignition_on
bool parking_brake_active
bool software_auto_enable
bool hardware_auto_enable
bool auto_mode_active
bool brake_pedal_pressed

float32 brake_circuit_temp
float32 ebrake_circuit_temp
float32 acc_temp_probe_one
float32 acc_temp_probe_two
float32 drive_controller_voltage
float32 accessory_controller_voltage

float32 brake_front_psi
float32 brake_rear_psi
float32 brake_current
float32 ebrake_current

byte NEUTRAL=0
byte FORWARD=1
byte REVERSE=2
byte CHARGING=3
byte UNKNOWN=4
byte gear_commanded