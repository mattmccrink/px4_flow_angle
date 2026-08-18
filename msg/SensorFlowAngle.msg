# Five-hole probe flow-angle estimate.
# Topic: sensor_flow_angle   Struct: sensor_flow_angle_s

uint64 timestamp              # time since system start (microseconds)
uint64 timestamp_sample       # timestamp of the sample the estimate is derived from

float32 true_airspeed_m_s     # airspeed estimate (density-corrected TAS; see FA_RHO)
float32 dynamic_pressure_pa   # q from the pitot channel (Pa)

float32 alpha_rad             # angle of attack (rad)
float32 beta_rad              # sideslip (rad)

float32 dp_alpha_pa           # raw alpha port-pair differential (Pa)
float32 dp_beta_pa            # raw beta port-pair differential (Pa)

float32 temp_alpha_c          # alpha sensor die temperature (degC)
float32 temp_beta_c           # beta sensor die temperature (degC)
float32 temp_pitot_c          # airspeed sensor die temperature (degC)

uint32 device_id              # driver device id

bool valid                    # true when q > FA_Q_MIN and all channels read OK
