#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

    typedef enum {
        ENSIM_LAYOUT_INLINE = 0,
        ENSIM_LAYOUT_V = 1,
        ENSIM_LAYOUT_BOXER = 2,
    } ensim_layout_e;

    typedef enum {
        ENSIM_FIRING_AUTO = 0,
        ENSIM_FIRING_CUSTOM = 1,
    } ensim_firing_mode_e;

    typedef struct {
        // --- topología ---
        uint32_t cylinders;        // 1..16 (realista)
        ensim_layout_e layout;     // inline / V / boxer
        float bank_angle_deg;      // V-angle (si aplica)
        ensim_firing_mode_e firing_mode;

        // firing order (opcional)
        // si firing_mode=ENSIM_FIRING_CUSTOM: array de [cylinders] con índices 0..cyl-1
        const uint8_t* firing_order;

        // --- geometría / mecánica ---
        float bore_m;              // diámetro cilindro
        float stroke_m;            // carrera
        float rod_length_m;        // biela
        float compression_ratio;   // RC
        float piston_mass_kg;
        float conrod_mass_kg;

        float crank_mass_kg;
        float crank_radius_m;
        float flywheel_mass_kg;
        float flywheel_radius_m;

        // fricción
        float piston_static_friction;
        float piston_dynamic_friction;

        // --- admisión / escape (áreas/volúmenes simplificados) ---
        float intake_plenum_volume_m3;
        float intake_runner_volume_m3;
        float intake_runner_max_area_m2;

        float exhaust_runner_volume_m3;
        float exhaust_runner_max_area_m2;
        float exhaust_plenum_volume_m3;

        // --- válvulas / timing (simplificado) ---
        float intake_open_deg;
        float intake_close_deg;
        float exhaust_open_deg;
        float exhaust_close_deg;
        float valve_ramp_deg;

        // --- limiter / starter ---
        float redline_rpm;
        float limiter_relax_rpm;
        float starter_torque_nm;
        float starter_no_load_rpm;
        float starter_radius_m;

        // --- audio ---
        float master_volume;
        float radial_spacing;      // el que usa ensim4 para spatial-ish
        float sample_rate_hz;      // normalmente 48000

        // --- combustión (simplificado) ---
        float afr;                 // 14.7 default
        float ignition_advance_deg;
        float fuel_lhv_j_per_kg;   // energía específica

    } ensim_engine_params_t;

    typedef struct ensim_context_t ensim_context_t;

    // lifecycle
    ensim_context_t* ensim_create(double monitor_refresh_hz);
    void ensim_destroy(ensim_context_t* ctx);

    // engine
    int  ensim_engine_build(ensim_context_t* ctx, const ensim_engine_params_t* p);
    void ensim_engine_reset(ensim_context_t* ctx);

    // realtime controls
    void ensim_set_throttle(ensim_context_t* ctx, float throttle_0_1);
    void ensim_set_starter(ensim_context_t* ctx, int on);
    void ensim_set_can_ignite(ensim_context_t* ctx, int on);

    // step: genera audio en buffer (mono float)
    size_t ensim_get_frames_per_tick(ensim_context_t* ctx); // tamaño recomendado
    int ensim_tick_audio_f32(ensim_context_t* ctx, float* out_frames, size_t frames);

    // telemetry
    float ensim_get_rpm(ensim_context_t* ctx);

#ifdef __cplusplus
}
#endif
