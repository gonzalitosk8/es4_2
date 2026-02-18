#include "engine_runtime.h"
#include "engine_blueprint.h"
#include <stdlib.h>
#include <string.h>

static double* g_theta_owned = NULL;

void engine_apply_blueprint(const engine_blueprint_t* bp) {
  g_engine_name = bp->name;

  g_engine_sound_volume = bp->sound_volume;
  g_engine_radial_spacing = bp->radial_spacing;
  g_engine_source_sink_volume_m3 = bp->source_sink_volume_m3;

  g_engine_piston_diameter_m = bp->bore_m;
  g_engine_piston_crank_throw_length_m = bp->crank_throw_m;
  g_engine_piston_connecting_rod_length_m = bp->rod_len_m;
  g_engine_piston_connecting_rod_mass_kg = bp->rod_mass_kg;
  g_engine_piston_head_mass_density_kg_per_m3 = bp->head_density;
  g_engine_piston_head_compression_height_m = bp->head_comp_h_m;
  g_engine_piston_head_clearance_height_m = bp->head_clear_h_m;
  g_engine_piston_dynamic_friction_n_m_s_per_r = bp->dyn_friction;
  g_engine_piston_static_friction_n_m_s_per_r = bp->stat_friction;

  g_engine_gas_momentum_damping_time_constant_s = bp->gas_momentum_tau_s;

  g_engine_eplenum_wave_pipe_length_m = bp->eplenum_wave_pipe_len_m;
  g_engine_mic_position_ratio = bp->mic_position_ratio;
  g_engine_velocity_low_pass_cutoff_frequency_hz = bp->vel_lpf_hz;

  g_engine_chamber_volume_m3 = bp->chamber_volume_m3;
  g_engine_throttle_volume_m3 = bp->throttle_volume_m3;
  g_engine_irunner_volume_m3 = bp->irunner_volume_m3;
  g_engine_injector_volume_m3 = bp->injector_volume_m3;
  g_engine_erunner_volume_m3 = bp->erunner_volume_m3;
  g_engine_eplenum_volume_m3 = bp->eplenum_volume_m3;
  g_engine_exhaust_volume_m3 = bp->exhaust_volume_m3;

  g_engine_max_flow_area_m2 = bp->max_flow_area_m2;
  g_engine_source_max_flow_area_m2 = bp->source_max_flow_area_m2;
  g_engine_throttle_max_flow_area_m2 = bp->throttle_max_flow_area_m2;
  g_engine_irunner_max_flow_area_m2 = bp->irunner_max_flow_area_m2;
  g_engine_injector_max_flow_area_m2 = bp->injector_max_flow_area_m2;
  g_engine_piston_max_flow_area_m2 = bp->piston_max_flow_area_m2;
  g_engine_erunner_max_flow_area_m2 = bp->erunner_max_flow_area_m2;
  g_engine_eplenum_max_flow_area_m2 = bp->eplenum_max_flow_area_m2;
  g_engine_exhaust_max_flow_area_m2 = bp->exhaust_max_flow_area_m2;

  g_engine_irunner_valve_engage_r = bp->irunner_valve_engage_r;
  g_engine_irunner_valve_ramp_r = bp->irunner_valve_ramp_r;
  g_engine_piston_valve_engage_r = bp->piston_valve_engage_r;
  g_engine_piston_valve_ramp_r = bp->piston_valve_ramp_r;
  g_engine_sparkplug_engage_r = bp->spark_engage_r;
  g_engine_sparkplug_on_r = bp->spark_on_r;

  g_engine_no_throttle = bp->no_throttle;
  g_engine_low_throttle = bp->low_throttle;
  g_engine_mid_throttle = bp->mid_throttle;
  g_engine_high_throttle = bp->high_throttle;

  g_engine_crankshaft_mass_kg = bp->crank_mass_kg;
  g_engine_crankshaft_radius_m = bp->crank_radius_m;
  g_engine_flywheel_mass_kg = bp->fly_mass_kg;
  g_engine_flywheel_radius_m = bp->fly_radius_m;
  g_engine_limiter_cutoff_r_per_s = bp->limiter_cutoff_r_per_s;
  g_engine_limiter_relaxed_r_per_s = bp->limiter_relaxed_r_per_s;
  e_engine_starter_rated_torque_n_m = bp->starter_rated_torque_n_m;
  g_engine_starter_no_load_r_per_s = bp->starter_no_load_r_per_s;
  g_engine_starter_radius_m = bp->starter_radius_m;

  // pistons theta (own copy)
  free(g_theta_owned);
  g_theta_owned = (double*)malloc(sizeof(double) * bp->cylinders);
  memcpy(g_theta_owned, bp->piston_theta_r, sizeof(double) * bp->cylinders);
  g_engine_piston_theta_r = g_theta_owned;
  g_engine_cylinders = bp->cylinders;

  // nodes: se dejan apuntados al template; si querés mutarlos, copiás a heap
  // (en el siguiente paso se hace "build_dynamic_nodes")
}
