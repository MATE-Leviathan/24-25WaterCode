# Continuous normalized command for auxiliary hardware owned by the Jetson.
# actuator_type is "servo", "linear", or "stop_all".
# value is normalized:
#   servo: 0.0=min, 1.0=max
#   linear: -1.0=back, 0.0=stop, 1.0=forward
#   stop_all: ignored
string name
string actuator_type
float32 value
