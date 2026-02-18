#pragma once

// mismos nombres que hoy aparecen como constexpr en engine_*.h
extern const char* g_engine_name;

extern double g_engine_sound_volume;
extern double g_engine_radial_spacing;
extern double g_engine_source_sink_volume_m3;

extern double g_engine_piston_diameter_m;
extern double g_engine_piston_crank_throw_length_m;
extern double g_engine_piston_connecting_rod_length_m;
extern double g_engine_piston_connecting_rod_mass_kg;
extern double g_engine_piston_head_mass_density_kg_per_m3;
extern double g_engine_piston_head_compression_height_m;
extern double g_engine_piston_head_clearance_height_m;
extern double g_engine_piston_dynamic_friction_n_m_s_per_r;
extern double g_engine_piston_static_friction_n_m_s_per_r;

extern double g_engine_gas_momentum_damping_time_constant_s;

extern double g_engine_eplenum_wave_pipe_length_m;
extern double g_engine_mic_position_ratio;
extern double g_engine_velocity_low_pass_cutoff_frequency_hz;

extern double g_engine_chamber_volume_m3;
extern double g_engine_throttle_volume_m3;
extern double g_engine_irunner_volume_m3;
extern double g_engine_injector_volume_m3;
extern double g_engine_erunner_volume_m3;
extern double g_engine_eplenum_volume_m3;
extern double g_engine_exhaust_volume_m3;

extern double g_engine_max_flow_area_m2;
extern double g_engine_source_max_flow_area_m2;
extern double g_engine_throttle_max_flow_area_m2;
extern double g_engine_irunner_max_flow_area_m2;
extern double g_engine_injector_max_flow_area_m2;
extern double g_engine_piston_max_flow_area_m2;
extern double g_engine_erunner_max_flow_area_m2;
extern double g_engine_eplenum_max_flow_area_m2;
extern double g_engine_exhaust_max_flow_area_m2;

// ángulos (en vez de g_engine_piston_0_theta_r hardcodeados por engine)
extern double* g_engine_piston_theta_r;   // array dinámico [cyl]
extern unsigned g_engine_cylinders;

// válvulas / ignition
extern double g_engine_irunner_valve_engage_r;
extern double g_engine_irunner_valve_ramp_r;
extern double g_engine_piston_valve_engage_r;
extern double g_engine_piston_valve_ramp_r;
extern double g_engine_sparkplug_engage_r;
extern double g_engine_sparkplug_on_r;

// throttle presets
extern double g_engine_no_throttle;
extern double g_engine_low_throttle;
extern double g_engine_mid_throttle;
extern double g_engine_high_throttle;

// inercia / starter / limiter
extern double g_engine_crankshaft_mass_kg;
extern double g_engine_crankshaft_radius_m;
extern double g_engine_flywheel_mass_kg;
extern double g_engine_flywheel_radius_m;
extern double g_engine_limiter_cutoff_r_per_s;
extern double g_engine_limiter_relaxed_r_per_s;
extern double e_engine_starter_rated_torque_n_m;
extern double g_engine_starter_no_load_r_per_s;
extern double g_engine_starter_radius_m;
