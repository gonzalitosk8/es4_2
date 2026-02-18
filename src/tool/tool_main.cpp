#include <cstdio>
#include <vector>
#include <SDL3/SDL.h>

extern "C" {
#include "../src/api/ensim_api.h"
}

// Dear ImGui
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"

static void default_params(ensim_engine_params_t* p) {
    *p = {};
    p->cylinders = 4;
    p->layout = ENSIM_LAYOUT_INLINE;
    p->bank_angle_deg = 90.0f;

    p->bore_m = 0.086f;
    p->stroke_m = 0.086f;
    p->rod_length_m = 0.143f;
    p->compression_ratio = 10.5f;

    p->piston_mass_kg = 0.45f;
    p->conrod_mass_kg = 0.55f;

    p->crank_mass_kg = 12.0f;
    p->crank_radius_m = 0.040f;
    p->flywheel_mass_kg = 8.0f;
    p->flywheel_radius_m = 0.140f;

    p->piston_static_friction = 0.08f;
    p->piston_dynamic_friction = 0.04f;

    p->intake_plenum_volume_m3 = 0.0030f;
    p->intake_runner_volume_m3 = 0.00035f;
    p->intake_runner_max_area_m2 = 0.0009f;

    p->exhaust_runner_volume_m3 = 0.00030f;
    p->exhaust_runner_max_area_m2 = 0.0010f;
    p->exhaust_plenum_volume_m3 = 0.0020f;

    p->intake_open_deg = 350.0f;
    p->intake_close_deg = 580.0f;
    p->exhaust_open_deg = 130.0f;
    p->exhaust_close_deg = 360.0f;
    p->valve_ramp_deg = 20.0f;

    p->redline_rpm = 7000.0f;
    p->limiter_relax_rpm = 6800.0f;

    p->starter_torque_nm = 80.0f;
    p->starter_no_load_rpm = 250.0f;
    p->starter_radius_m = 0.08f;

    p->master_volume = 1.0f;
    p->radial_spacing = 0.035f;
    p->sample_rate_hz = 48000.0f;

    p->afr = 14.7f;
    p->ignition_advance_deg = 12.0f;
    p->fuel_lhv_j_per_kg = 43000000.0f;
}

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) return 1;

    SDL_Window* win = SDL_CreateWindow("ensim tool", 1280, 720, 0);
    SDL_Renderer* ren = SDL_CreateRenderer(win, nullptr, 0);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplSDL3_InitForSDLRenderer(win, ren);
    ImGui_ImplSDLRenderer3_Init(ren);

    // ensim core
    ensim_context_t* ctx = ensim_create(60.0);
    ensim_engine_params_t P;
    default_params(&P);

    // construir engine inicial
    ensim_engine_build(ctx, &P);
    ensim_set_can_ignite(ctx, 1);
    ensim_set_starter(ctx, 0);
    ensim_set_throttle(ctx, 0.0f);

    bool running = true;
    float throttle = 0.0f;
    bool starter = false;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);
            if (e.type == SDL_EVENT_QUIT) running = false;
        }

        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        bool rebuild = false;

        ImGui::Begin("Engine");
        rebuild |= ImGui::SliderInt("Cylinders", (int*)&P.cylinders, 1, 16);
        rebuild |= ImGui::SliderFloat("Bore (m)", &P.bore_m, 0.05f, 0.12f);
        rebuild |= ImGui::SliderFloat("Stroke (m)", &P.stroke_m, 0.05f, 0.12f);
        rebuild |= ImGui::SliderFloat("Compression", &P.compression_ratio, 6.0f, 14.0f);
        rebuild |= ImGui::SliderFloat("Redline (rpm)", &P.redline_rpm, 1000.0f, 12000.0f);
        rebuild |= ImGui::SliderFloat("Master Vol", &P.master_volume, 0.0f, 2.0f);

        if (ImGui::Button("Rebuild")) rebuild = true;

        ImGui::Separator();
        ImGui::SliderFloat("Throttle", &throttle, 0.0f, 1.0f);
        ImGui::Checkbox("Starter", &starter);

        ImGui::Text("RPM: %.0f", (double)ensim_get_rpm(ctx));
        ImGui::End();

        if (rebuild) {
            // rebuild sin recompilar
            ensim_engine_build(ctx, &P);
            ensim_set_can_ignite(ctx, 1);
        }

        ensim_set_throttle(ctx, throttle);
        ensim_set_starter(ctx, starter ? 1 : 0);

        // Render UI
        SDL_SetRenderDrawColor(ren, 10, 10, 10, 255);
        SDL_RenderClear(ren);
        ImGui::Render();
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(ren);
    }

    ensim_destroy(ctx);
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
tool_main.cpp