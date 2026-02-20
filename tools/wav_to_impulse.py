#!/usr/bin/env python3
"""
wav_to_impulse.py
=================
Convierte una grabación WAV de un caño de escape real en coeficientes
de respuesta impulsional (IR) para usar en ensim4.

USO BÁSICO (impulso ya grabado):
    python3 wav_to_impulse.py grabacion.wav --output auto_sport.h --name auto_sport

USO DECONVOLUCION (grabaste sweep + respuesta):
    python3 wav_to_impulse.py respuesta.wav --sweep barrido_referencia.wav \
            --output moto_naked.h --name moto_naked

USO SÍNTESIS PROCEDURAL (sin grabación real, por tipo):
    python3 wav_to_impulse.py --synth car_4cyl --output auto_4cil.h --name auto_4cil
    python3 wav_to_impulse.py --synth motorcycle --output moto.h --name moto
    python3 wav_to_impulse.py --synth truck --output camion.h --name camion

OPCIONES:
    --samples N        Número de coeficientes en la IR (default: 8192)
    --samplerate HZ    Sample rate de destino (default: 48000)
    --normalize        Normalizar peak de la IR a 1.0 (default: on)
    --no-normalize     No normalizar
    --output FILE.h    Archivo C++ de salida (default: stdout)
    --name IDENTIFIER  Nombre C++ del preset (sin espacios)
    --append           Si el archivo existe, agregar en vez de reemplazar

FLUJO PARA GRABACIÓN REAL:
    1. Generar sweep:  python3 wav_to_impulse.py --gen-sweep sweep.wav
    2. Reproducir sweep.wav por altavoz cerca del caño
    3. Grabar la respuesta del micrófono en respuesta.wav
    4. Deconvolucionar: python3 wav_to_impulse.py respuesta.wav \\
                        --sweep sweep.wav --output mi_auto.h --name mi_auto

FLUJO RÁPIDO (síntesis procedural):
    Genera una IR sintética diseñada para sonar a ese tipo de vehículo.
    No requiere grabación. Resultado: timbre genérico pero correcto.
"""

import sys
import os
import argparse
import struct
import math
import random
import wave
import array

# ── Dependencias opcionales ────────────────────────────────────
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False
    print("[wav_to_impulse] numpy no encontrado — usando modo básico (solo síntesis procedural)")


# ══════════════════════════════════════════════════════════════
# Síntesis procedural de IRs por tipo de vehículo
# ══════════════════════════════════════════════════════════════

VEHICLE_PROFILES = {
    # (decay_ms, fundamental_hz, harmonics, noise_ratio, description)
    "car_4cyl": dict(
        decay_ms=180,
        fundamental_hz=120,
        harmonics=[1.0, 0.6, 0.35, 0.18, 0.08, 0.04],
        noise_ratio=0.05,
        description="Auto 4 cilindros aspirado (tipo Fiat, VW, Toyota)"
    ),
    "car_4cyl_turbo": dict(
        decay_ms=140,
        fundamental_hz=110,
        harmonics=[1.0, 0.7, 0.4, 0.2, 0.1, 0.05],
        noise_ratio=0.08,
        description="Auto 4 cilindros turbocargado"
    ),
    "car_v6": dict(
        decay_ms=200,
        fundamental_hz=100,
        harmonics=[1.0, 0.5, 0.25, 0.12, 0.06, 0.03],
        noise_ratio=0.04,
        description="Auto V6"
    ),
    "car_v8": dict(
        decay_ms=220,
        fundamental_hz=90,
        harmonics=[1.0, 0.45, 0.22, 0.1, 0.05, 0.025],
        noise_ratio=0.03,
        description="Auto V8 americano"
    ),
    "motorcycle": dict(
        decay_ms=80,
        fundamental_hz=280,
        harmonics=[1.0, 0.8, 0.55, 0.3, 0.15, 0.07, 0.03],
        noise_ratio=0.12,
        description="Moto 1-2 cilindros"
    ),
    "motorcycle_inline4": dict(
        decay_ms=90,
        fundamental_hz=320,
        harmonics=[1.0, 0.7, 0.45, 0.2, 0.08, 0.03],
        noise_ratio=0.09,
        description="Moto 4 cilindros en línea (tipo Ninja, CBR)"
    ),
    "truck": dict(
        decay_ms=300,
        fundamental_hz=65,
        harmonics=[1.0, 0.55, 0.28, 0.14, 0.07, 0.035],
        noise_ratio=0.06,
        description="Camión diesel"
    ),
    "tractor": dict(
        decay_ms=350,
        fundamental_hz=45,
        harmonics=[1.0, 0.6, 0.3, 0.15],
        noise_ratio=0.15,
        description="Tractor / motor lento"
    ),
    "f1": dict(
        decay_ms=60,
        fundamental_hz=450,
        harmonics=[1.0, 0.9, 0.7, 0.5, 0.35, 0.2, 0.1, 0.05],
        noise_ratio=0.15,
        description="F1 / auto de competición"
    ),
    "custom": dict(
        decay_ms=150,
        fundamental_hz=150,
        harmonics=[1.0, 0.5, 0.25],
        noise_ratio=0.05,
        description="Preset custom"
    ),
}


def synth_ir(profile_name, num_samples, sample_rate, seed=42):
    """
    Genera una respuesta impulsional sintética para el perfil dado.
    Modela: decaimiento exponencial + armónicos + ruido de sala.
    """
    if not HAS_NUMPY:
        return synth_ir_basic(profile_name, num_samples, sample_rate, seed)

    prof = VEHICLE_PROFILES.get(profile_name)
    if prof is None:
        print(f"[wav_to_impulse] Perfil desconocido: '{profile_name}'")
        print(f"  Disponibles: {', '.join(VEHICLE_PROFILES.keys())}")
        sys.exit(1)

    rng = np.random.default_rng(seed)
    ir = np.zeros(num_samples)
    t = np.arange(num_samples) / sample_rate

    decay_tau = prof["decay_ms"] / 1000.0 / np.log(1000)  # -60dB en decay_ms
    envelope = np.exp(-t / decay_tau)

    # Armónicos
    f0 = prof["fundamental_hz"]
    for i, amp in enumerate(prof["harmonics"]):
        freq = f0 * (i + 1)
        if freq > sample_rate / 2:
            break
        # Pequeña variación de fase por armónico (más natural)
        phase = rng.uniform(0, 2 * np.pi)
        ir += amp * np.sin(2 * np.pi * freq * t + phase) * envelope

    # Ruido de sala (decaimiento más rápido)
    noise_tau = decay_tau * 0.3
    noise_env = np.exp(-t / noise_tau)
    ir += prof["noise_ratio"] * rng.standard_normal(num_samples) * noise_env

    # Pre-delay suave (ataque ~0.5ms)
    pre_delay_samples = int(0.0005 * sample_rate)
    if pre_delay_samples > 0 and pre_delay_samples < num_samples:
        attack = np.linspace(0, 1, pre_delay_samples)
        ir[:pre_delay_samples] *= attack

    # Normalizar
    peak = np.max(np.abs(ir))
    if peak > 0:
        ir /= peak

    return ir.tolist()


def synth_ir_basic(profile_name, num_samples, sample_rate, seed=42):
    """
    Versión sin numpy: síntesis básica usando math estándar.
    """
    prof = VEHICLE_PROFILES.get(profile_name)
    if prof is None:
        print(f"[wav_to_impulse] Perfil desconocido: '{profile_name}'")
        sys.exit(1)

    random.seed(seed)
    ir = [0.0] * num_samples

    decay_tau = (prof["decay_ms"] / 1000.0) / math.log(1000)
    f0 = prof["fundamental_hz"]

    for n in range(num_samples):
        t = n / sample_rate
        envelope = math.exp(-t / decay_tau)

        for i, amp in enumerate(prof["harmonics"]):
            freq = f0 * (i + 1)
            if freq > sample_rate / 2:
                break
            phase = (i * 1.234567)  # fase fija reproducible
            ir[n] += amp * math.sin(2 * math.pi * freq * t + phase) * envelope

        noise_tau = decay_tau * 0.3
        noise_env = math.exp(-t / noise_tau)
        ir[n] += prof["noise_ratio"] * (random.random() * 2 - 1) * noise_env

    # Pre-delay
    pre_delay = int(0.0005 * sample_rate)
    for n in range(min(pre_delay, num_samples)):
        ir[n] *= n / pre_delay

    # Normalizar
    peak = max(abs(x) for x in ir)
    if peak > 0:
        ir = [x / peak for x in ir]

    return ir


# ══════════════════════════════════════════════════════════════
# Carga y procesamiento de WAV
# ══════════════════════════════════════════════════════════════

def load_wav_mono(filepath):
    """
    Carga un WAV y retorna (samples_float, sample_rate).
    Mezcla a mono si es estéreo.
    """
    with wave.open(filepath, 'r') as wf:
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        n_frames = wf.getnframes()
        raw = wf.readframes(n_frames)

    fmt = {1: 'b', 2: 'h', 4: 'i'}.get(sample_width)
    if fmt is None:
        raise ValueError(f"Formato WAV no soportado: {sample_width} bytes/muestra")

    samples = array.array(fmt, raw)
    max_val = float(2 ** (8 * sample_width - 1))

    if n_channels == 1:
        floats = [s / max_val for s in samples]
    else:
        # Mezcla a mono
        floats = []
        for i in range(0, len(samples), n_channels):
            mono = sum(samples[i + c] for c in range(n_channels)) / n_channels
            floats.append(mono / max_val)

    return floats, sample_rate


def deconvolve_sweep(response, sweep, num_samples):
    """
    Deconvoluciona una respuesta del micrófono con el sweep de referencia
    para extraer la IR real del caño.
    Requiere numpy.
    """
    if not HAS_NUMPY:
        print("[wav_to_impulse] La deconvolución requiere numpy.")
        print("  Instalá con: pip3 install numpy")
        sys.exit(1)

    r = np.array(response)
    s = np.array(sweep)

    # FFT-based deconvolution con regularización
    N = max(len(r), len(s))
    N = 1 << int(math.ceil(math.log2(N * 2)))  # siguiente potencia de 2

    R = np.fft.rfft(r, n=N)
    S = np.fft.rfft(s, n=N)

    eps = 1e-6 * np.max(np.abs(S))
    H = R * np.conj(S) / (np.abs(S) ** 2 + eps)

    ir = np.fft.irfft(H)
    ir = ir[:num_samples]

    peak = np.max(np.abs(ir))
    if peak > 0:
        ir /= peak

    return ir.tolist()


def generate_sweep(duration_s=5.0, f_start=20.0, f_end=20000.0,
                   sample_rate=48000, output_path="sweep.wav"):
    """
    Genera un sine sweep logarítmico y lo guarda como WAV.
    """
    if HAS_NUMPY:
        n = int(duration_s * sample_rate)
        t = np.arange(n) / sample_rate
        # Log sweep
        k = duration_s / math.log(f_end / f_start)
        phase = 2 * math.pi * f_start * k * (np.exp(t / k) - 1)
        sig = np.sin(phase)
        samples = (sig * 32767).astype(np.int16)
        data = samples.tobytes()
    else:
        n = int(duration_s * sample_rate)
        k = duration_s / math.log(f_end / f_start)
        data = bytearray()
        for i in range(n):
            t = i / sample_rate
            phase = 2 * math.pi * f_start * k * (math.exp(t / k) - 1)
            val = int(math.sin(phase) * 32767)
            data += struct.pack('<h', val)

    with wave.open(output_path, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(bytes(data))

    print(f"[wav_to_impulse] Sweep generado: {output_path} ({duration_s}s, {f_start:.0f}–{f_end:.0f}Hz)")


# ══════════════════════════════════════════════════════════════
# Generación del header C++
# ══════════════════════════════════════════════════════════════

def ir_to_cpp_header(ir, name, description="", source_info=""):
    """
    Convierte una lista de floats en un header C++ compatible con
    el sistema de impulse library de ensim4.
    """
    n = len(ir)

    lines = []
    lines.append(f"// ── Impulse Response: {name} ──")
    if description:
        lines.append(f"// {description}")
    if source_info:
        lines.append(f"// Fuente: {source_info}")
    lines.append(f"// Muestras: {n} @ 48000 Hz ({n/48000*1000:.1f} ms)")
    lines.append(f"//")
    lines.append(f"// Generado con: wav_to_impulse.py")
    lines.append(f"// Para agregar al engine JSON:")
    lines.append(f'//   "impulse_preset": "{name}"')
    lines.append(f"")
    lines.append(f"#pragma once")
    lines.append(f"")
    lines.append(f"static const double g_impulse_{name}[] = {{")

    # Escribir en filas de 8 valores
    for i in range(0, n, 8):
        chunk = ir[i:i+8]
        row = ", ".join(f"{v:.10f}" for v in chunk)
        if i + 8 < n:
            lines.append(f"    {row},")
        else:
            lines.append(f"    {row}")

    lines.append(f"}};")
    lines.append(f"")
    lines.append(f"static const size_t g_impulse_{name}_size = {n};")
    lines.append(f"")

    return "\n".join(lines)


def update_impulse_library(header_path, new_header_content, name):
    """
    Actualiza o crea impulse_library.h agregando el nuevo preset.
    """
    lib_path = os.path.join(os.path.dirname(header_path), "impulse_library.h")

    # Incluir el header individual
    include_line = f'#include "{os.path.basename(header_path)}"'

    # Entrada en el array de registro
    entry = f'    {{ "{name}", g_impulse_{name}, g_impulse_{name}_size }},'

    if os.path.exists(lib_path):
        with open(lib_path, 'r') as f:
            content = f.read()
        # Agregar include si no existe
        if include_line not in content:
            content = content.replace("// ── INCLUDES ──", f"// ── INCLUDES ──\n{include_line}")
        # Agregar entry si no existe
        if f'"{name}"' not in content:
            content = content.replace("    // ── ENTRIES ──", f"    // ── ENTRIES ──\n{entry}")
        with open(lib_path, 'w') as f:
            f.write(content)
        print(f"[wav_to_impulse] Actualizado: {lib_path}")
    else:
        print(f"[wav_to_impulse] impulse_library.h no encontrado. Creá el archivo base primero.")
        print(f"  Ejecutá: python3 wav_to_impulse.py --gen-library {os.path.dirname(lib_path)}/")


# ══════════════════════════════════════════════════════════════
# CLI
# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument("input_wav", nargs="?",
                        help="WAV de entrada (impulso grabado o respuesta al sweep)")
    parser.add_argument("--sweep", metavar="SWEEP.WAV",
                        help="WAV del sweep de referencia para deconvolución")
    parser.add_argument("--synth", metavar="VEHICLE_TYPE",
                        help="Sintetizar IR proceduralmente por tipo de vehículo")
    parser.add_argument("--gen-sweep", metavar="OUTPUT.WAV",
                        help="Generar sweep de calibración y salir")
    parser.add_argument("--list-profiles", action="store_true",
                        help="Mostrar perfiles de síntesis disponibles")

    parser.add_argument("--output", "-o", metavar="FILE.h",
                        help="Archivo de salida (default: stdout)")
    parser.add_argument("--name", "-n", metavar="IDENTIFIER",
                        default="custom",
                        help="Identificador C++ del preset (default: custom)")
    parser.add_argument("--samples", type=int, default=8192,
                        help="Número de coeficientes (default: 8192)")
    parser.add_argument("--samplerate", type=int, default=48000,
                        help="Sample rate (default: 48000)")
    parser.add_argument("--no-normalize", action="store_true",
                        help="No normalizar la IR")
    parser.add_argument("--append", action="store_true",
                        help="Agregar al archivo en lugar de sobreescribir")
    parser.add_argument("--update-library", action="store_true",
                        help="Actualizar impulse_library.h con el nuevo preset")

    args = parser.parse_args()

    # ── Listar perfiles ────────────────────────────────────────
    if args.list_profiles:
        print("\nPerfiles de síntesis disponibles:")
        print("-" * 60)
        for key, prof in VEHICLE_PROFILES.items():
            print(f"  {key:<25} {prof['description']}")
        print()
        return

    # ── Generar sweep ──────────────────────────────────────────
    if args.gen_sweep:
        generate_sweep(output_path=args.gen_sweep, sample_rate=args.samplerate)
        return

    # ── Obtener IR ─────────────────────────────────────────────
    if args.synth:
        print(f"[wav_to_impulse] Sintetizando IR: '{args.synth}'")
        prof = VEHICLE_PROFILES.get(args.synth)
        if prof:
            print(f"  {prof['description']}")
        ir = synth_ir(args.synth, args.samples, args.samplerate)
        source_info = f"síntesis procedural: {args.synth}"
        description = VEHICLE_PROFILES.get(args.synth, {}).get("description", "")

    elif args.input_wav:
        print(f"[wav_to_impulse] Cargando: {args.input_wav}")
        response, sr = load_wav_mono(args.input_wav)
        print(f"  {len(response)} muestras @ {sr} Hz")

        if args.sweep:
            print(f"[wav_to_impulse] Deconvolucionando con sweep: {args.sweep}")
            sweep, _ = load_wav_mono(args.sweep)
            ir = deconvolve_sweep(response, sweep, args.samples)
            source_info = f"deconvolución: {args.input_wav} / {args.sweep}"
        else:
            # Usar directamente como IR (truncar/padear)
            if len(response) >= args.samples:
                ir = response[:args.samples]
            else:
                ir = response + [0.0] * (args.samples - len(response))
            source_info = f"WAV directo: {args.input_wav}"

        if not args.no_normalize:
            peak = max(abs(x) for x in ir)
            if peak > 0:
                ir = [x / peak for x in ir]

        description = f"IR grabada de escape real"

    else:
        print("Error: especificá --synth TIPO o un archivo WAV de entrada.")
        print("  Ejemplo: python3 wav_to_impulse.py --synth car_4cyl -o auto.h -n auto_4cil")
        print("  Tipos disponibles: --list-profiles")
        sys.exit(1)

    # ── Generar header ─────────────────────────────────────────
    header = ir_to_cpp_header(ir, args.name, description, source_info)

    if args.output:
        mode = "a" if args.append else "w"
        with open(args.output, mode) as f:
            f.write(header)
        print(f"[wav_to_impulse] Guardado: {args.output} ({len(ir)} coeficientes)")
        if args.update_library:
            update_impulse_library(args.output, header, args.name)
    else:
        print(header)


if __name__ == "__main__":
    main()
