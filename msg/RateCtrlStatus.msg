uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ

# TD states
float32 td_trate_roll
float32 td_trate_pitch
float32 td_trate_yaw
float32 td_tdrate_roll
float32 td_tdrate_pitch
float32 td_tdrate_yaw

# ESO states
float32 eso_rate_roll
float32 eso_rate_pitch
float32 eso_rate_yaw
float32 eso_drate_roll
float32 eso_drate_pitch
float32 eso_drate_yaw
float32 eso_inertia_roll
float32 eso_inertia_pitch
float32 eso_inertia_yaw
float32 eso_dis_roll
float32 eso_dis_pitch
float32 eso_dis_yaw

# Rate commands
float32 rate_cmd_roll
float32 rate_cmd_pitch
float32 rate_cmd_yaw
float32 rate_pidcmd_roll
float32 rate_pidcmd_pitch
float32 rate_pidcmd_yaw
