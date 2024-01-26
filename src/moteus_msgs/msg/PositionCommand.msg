# Each item may either be an empty list, or an list of length 1.
#
# Empty items are not sent in the resulting message.  Non-empty lists
# will have their first element sent in the command and the remainder
# ignored.
float32[] position []
float32[] velocity []
float32[] feedforward_torque []
float32[] kp_scale []
float32[] kd_scale []
float32[] maximum_torque []
float32[] stop_position []
float32[] watchdog_timeout []
float32[] velocity_limit []
float32[] accel_limit []
float32[] fixed_voltage_override []
