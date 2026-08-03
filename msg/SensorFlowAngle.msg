# Five-hole probe derived air-data / flow angles
# Generated topic: sensor_flow_angle   struct: sensor_flow_angle_s

uint64 timestamp            # time since system start (microseconds) - REQUIRED, used by logger
uint64 timestamp_sample     # timestamp of the underlying pressure measurement (microseconds)

float32 true_airspeed_m_s   # airspeed from the total-static differential (m/s)
float32 alpha_rad           # angle of attack  (rad)
float32 beta_rad            # angle of sideslip (rad)

float32 dynamic_pressure_pa # measured dynamic pressure q (Pa)
float32 dp_alpha_pa         # alpha-port differential pressure (Pa)
float32 dp_beta_pa          # beta-port  differential pressure (Pa)

uint8 device_id             # which pitot instance fed the airspeed (for cross-check)
bool valid                  # true when q is above threshold and angles are within the calibrated cone
