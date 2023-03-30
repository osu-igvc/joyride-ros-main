# Holds the summary of the overall robot's status
# - Based on popular ROS diagnostics messages

builtin_interfaces/Time stamp

# Summary of system status, basically copy of DiagnosticStatus for overall aggregation

byte OK=0
byte WARN=1
byte ERROR=2
byte STALE=3

byte system_status

# What control method is being used
byte MANUAL=0
byte JOYSTICK=1
byte AUTONOMOUS=2

byte control_input

# Whether or not system is software enabled. This doesn't imply
# autonomous behavior, only that ROS wants the sytem to be enabled.
# True enable status will be included in a status message provided by
# DBW, published by ROSCAN.

bool auto_software_enabled