# Simulación Física Completa de Escape — Guía Técnica Profunda
## Para ensim4 — Fiat 1600 16v, colector 4-en-1

---

## Índice

1. [Por qué la convolución no puede capturar la física real](#1-por-qué-la-convolución-no-puede-capturar-la-física-real)
2. [Fundamentos: qué viaja por el caño](#2-fundamentos-qué-viaja-por-el-caño)
3. [Digital Waveguide: la teoría completa](#3-digital-waveguide-la-teoría-completa)
4. [Retardo fraccionario: el problema de la cuantización](#4-retardo-fraccionario-el-problema-de-la-cuantización)
5. [El colector 4-en-1: scattering matrix en profundidad](#5-el-colector-4-en-1-scattering-matrix-en-profundidad)
6. [Impedancia de radiación: el extremo abierto no es solo inversión de signo](#6-impedancia-de-radiación-el-extremo-abierto-no-es-solo-inversión-de-signo)
7. [Temperatura: gradiente dinámico y su efecto en el sonido](#7-temperatura-gradiente-dinámico-y-su-efecto-en-el-sonido)
8. [Curvas y cambios de sección: discontinuidades acústicas](#8-curvas-y-cambios-de-sección-discontinuidades-acústicas)
9. [Nonlinearidades: por qué suena distinto a full throttle](#9-nonlinearidades-por-qué-suena-distinto-a-full-throttle)
10. [Vibración mecánica del caño: coupling estructural-acústico](#10-vibración-mecánica-del-caño-coupling-estructural-acústico)
11. [La admisión: el lado olvidado que modela el timbre de baja](#11-la-admisión-el-lado-olvidado-que-modela-el-timbre-de-baja)
12. [El pulso de combustión: la señal de entrada real](#12-el-pulso-de-combustión-la-señal-de-entrada-real)
13. [Efecto scavenging: la resonancia que da potencia y timbre](#13-efecto-scavenging-la-resonancia-que-da-potencia-y-timbre)
14. [El silenciador: modelo de cámara con pérdidas](#14-el-silenciador-modelo-de-cámara-con-pérdidas)
15. [Flujo de señal completo](#15-flujo-de-señal-completo)
16. [Parámetros JSON para el Fiat 4a1](#16-parámetros-json-para-el-fiat-4a1)
17. [Costo computacional y optimizaciones](#17-costo-computacional-y-optimizaciones)
18. [Ruta de implementación por etapas](#18-ruta-de-implementación-por-etapas)

---

## 1. Por qué la convolución no puede capturar la física real

Una respuesta impulsional (IR) es una **fotografía temporal** del sistema. Captura cómo el caño responde a un impulso en una condición fija. El problema es que la condición no es fija:

| Variable física | Cómo cambia | Efecto en el sonido |
|---|---|---|
| Temperatura del gas | Con RPM, carga, tiempo de calentamiento | Corre todas las resonancias ±15% |
| Caudal de gas | Con aceleración | Crea no-linearidades, distorsión armónica |
| Presión media | Con carga | Modifica impedancia acústica efectiva |
| Velocidad del gas (DC flow) | Con apertura de válvulas | Desplaza Doppler las frecuencias |

La IR grabada a 3000 RPM en caliente es incorrecta a 1000 RPM en frío, incorrecta a 6000 RPM en aceleración, e incorrecta en cualquier punto de transición. No por un margen pequeño — puede ser varios dB de diferencia en las frecuencias características del caño.

Más importante: la IR no tiene **causalidad temporal dinámica**. No puede modelar que cuando el cilindro 1 abre la válvula de escape, el pulso de presión tarda 1.2ms en llegar al colector, y que esa onda, al llegar al extremo abierto del caño de escape, se refleja y tarda otros 3ms en volver. Ese rebote puede (o no) llegar justo cuando el cilindro 4 abre su válvula — y eso depende de las RPM exactas. La IR no lo puede saber.

---

## 2. Fundamentos: qué viaja por el caño

En un caño de escape, lo que viaja es una **onda de presión en gas comprimible**. La física base es la ecuación de onda 1D:

```
∂²p/∂t² = c² · ∂²p/∂x²
```

donde `c = √(γ · R · T / M)` es la velocidad del sonido, que depende de:
- `γ`: razón de calores específicos del gas de escape (~1.35, vs 1.4 del aire)
- `R`: constante universal de gases (8.314 J/mol·K)
- `T`: temperatura absoluta en Kelvin
- `M`: masa molar del gas de escape (~0.029 kg/mol, similar al aire)

La solución general de esta ecuación es la superposición de dos ondas que viajan en direcciones opuestas:

```
p(x, t) = p⁺(t - x/c) + p⁻(t + x/c)
```

`p⁺` viaja hacia el extremo abierto (presión de avance).
`p⁻` viaja hacia el motor (onda reflejada).

Esto es **todo el modelo**. El waveguide digital es simplemente cómo representamos estas dos funciones en tiempo discreto.

La velocidad de partícula del gas (relacionada con el flujo de masa) es:

```
v(x, t) = [p⁺(t - x/c) - p⁻(t + x/c)] / (ρ · c)
```

donde `ρ · c` es la **impedancia acústica característica** del tubo: `Z₀ = ρ · c`. Esto va a ser clave cuando conectemos secciones de diferente diámetro.

---

## 3. Digital Waveguide: la teoría completa

### El buffer circular como línea de retardo

En tiempo discreto, una onda que viaja por un tubo de longitud `L` a velocidad `c` tarda `N = L·fs/c` muestras en llegar al otro extremo. Eso es exactamente un buffer circular de `N` muestras:

```c
// Waveguide de un solo caño
typedef struct {
    float *buf_fwd;     // p+: onda que avanza hacia la salida
    float *buf_rev;     // p-: onda que vuelve hacia el motor
    int    length;      // N = L * fs / c
    int    write_pos;   // cabeza de escritura
} waveguide_tube_t;
```

Para avanzar un sample:

```c
void wg_tick(waveguide_tube_t *wg,
             float in_fwd, float in_rev,
             float *out_fwd, float *out_rev)
{
    // Leer extremo opuesto (lo que llegó hace N muestras)
    int read_pos = (wg->write_pos + 1) % wg->length;
    *out_fwd = wg->buf_fwd[read_pos];
    *out_rev = wg->buf_rev[wg->write_pos];

    // Escribir nuevo input
    wg->buf_fwd[wg->write_pos] = in_fwd;
    wg->buf_rev[read_pos]      = in_rev;

    wg->write_pos = read_pos;
}
```

La presión total en cualquier punto del caño es `p = p⁺ + p⁻`.  
La velocidad de partícula es `v = (p⁺ - p⁻) / Z₀`.

### Por qué esto captura la física real

Cada sample que metés en `buf_fwd` viaja exactamente `N` muestras hasta el otro extremo. Si el extremo abierto lo refleja con coeficiente `-1` (inversión de signo), esa muestra vuelve por `buf_rev` otros `N` samples hasta el motor. El round-trip total es `2N` samples = `2L/c` segundos. Eso corresponde exactamente a las frecuencias de resonancia del tubo abierto: `f_n = n·c/(2L)` para n = 1, 2, 3...

No hay nada hardcodeado sobre esas frecuencias — emergen naturalmente de la geometría y la velocidad del sonido.

---

## 4. Retardo fraccionario: el problema de la cuantización

El gran problema práctico: `N = L·fs/c` no es entero. Por ejemplo, para un caño primario de 420mm a 48000 Hz con c=600 m/s (a 580°C):

```
N = 0.420 * 48000 / 600 = 33.6 muestras
```

Si redondeás a 34, la frecuencia fundamental del caño es `c/(2*34/48000) = 600*48000/(2*34) = 423.5 Hz`.  
Si redondeás a 33, es `600*48000/(2*33) = 436.4 Hz`.

La diferencia es **12.9 Hz** — claramente audible, y se magnifica en los armónicos superiores.

### Filtro de retardo fraccionario (Allpass de primer orden)

La solución es un **filtro allpass de Thiran**, que tiene ganancia unitaria en todas las frecuencias pero fase variable, de modo que el retardo de grupo total sea exactamente `N+η` muestras, donde `η` es la fracción:

Para un allpass de primer orden:
```
H(z) = (a + z⁻¹) / (1 + a·z⁻¹)
```

donde el coeficiente se calcula como:
```
η = N - floor(N)
a = (1 - η) / (1 + η)
```

Implementación:
```c
typedef struct {
    float coeff;  // a
    float state;  // y[n-1]
} allpass_t;

float allpass_tick(allpass_t *ap, float x)
{
    float y = ap->coeff * (x - ap->state) + ap->state;
    // Corrección: y = a*(x - y_prev) + x_prev
    float y_new = ap->coeff * x + ap->state;
    ap->state = y_new - ap->coeff * ap->state;
    return y_new;
}
```

En la práctica: cada extremo del waveguide (donde entra o sale la onda) tiene un allpass de primer orden. Así el retardo total es fraccionario con precisión de punto flotante.

Para cambios de temperatura dinámicos (donde `c` cambia), hay que actualizar `a` en tiempo real. Un cambio brusco de `a` crea un click — hay que rampearlo con un filtro de primer orden (constante de tiempo ~10ms).

---

## 5. El colector 4-en-1: scattering matrix en profundidad

### La junction como problema de impedancia

Cuando cuatro tubos se unen en un punto, la física exige **continuidad de presión** y **conservación de flujo de volumen**:

```
p₁ = p₂ = p₃ = p₄ = p₄ₒ   (la presión en el nodo es la misma para todos)
q₁ + q₂ + q₃ + q₄ = q_out  (flujo de volumen total = flujo hacia el caño secundario)
```

donde `qᵢ = A·v = A·(pᵢ⁺ - pᵢ⁻)/Z₀ᵢ` y `A` es el área de la sección.

Definimos la **admitancia** de cada caño: `Yᵢ = Aᵢ/Z₀ᵢ = Aᵢ²/(ρ·c)`.

La presión en el nodo es:
```
p_nodo = (2 · Σᵢ Yᵢ · pᵢ⁺) / (Σᵢ Yᵢ + Y_out)
```

La onda reflejada de vuelta a cada caño primario es:
```
pᵢ⁻ = p_nodo - pᵢ⁺
```

La onda transmitida hacia el caño secundario:
```
p_out⁺ = p_nodo
```

### Implementación de la scattering matrix para 4-en-1 con secciones iguales

Si los cuatro caños primarios tienen el mismo diámetro y el caño secundario tiene área `A_sec`:

```
Y_pri = A_pri² / (ρ·c)
Y_sec = A_sec² / (ρ·c)
Y_total = 4·Y_pri + Y_sec
```

```c
void scatter_4to1(
    float p_fwd[4],    // ondas entrantes de cada cilindro
    float p_sec_rev,   // onda que vuelve del caño secundario
    float p_ref[4],    // reflexiones de vuelta a cada cilindro
    float *p_sec_fwd,  // onda que avanza al caño secundario
    float Y_pri, float Y_sec)
{
    float Y_total = 4.0f * Y_pri + Y_sec;

    // Sumar todas las ondas incidentes ponderadas por admitancia
    float sum = 0.0f;
    for (int i = 0; i < 4; i++) sum += Y_pri * p_fwd[i];
    sum += Y_sec * p_sec_rev;

    float p_node = 2.0f * sum / Y_total;

    // Reflexiones
    for (int i = 0; i < 4; i++) p_ref[i] = p_node - p_fwd[i];
    *p_sec_fwd = p_node - p_sec_rev;
}
```

### El detalle que cambia todo: secciones no iguales

En la realidad, los caños primarios del Fiat 1600 no tienen exactamente el mismo diámetro a lo largo de todo su recorrido. El primer tramo (cerca de la culata) es más estrecho (~38mm) y se expande antes del colector (~42mm). Cada transición de diámetro **agrega una reflexión parcial** — una parte de la onda se refleja, otra se transmite. Esto está detallado en la sección 8.

### Orden de encendido en el tiempo

El colector 4-en-1 recibe un pulso de presión cada 180° de cigüeñal (en un motor de 4 cilindros de 4 tiempos, hay una explosión cada 180° ya que 720°/4 = 180°). A 3000 RPM, eso es una explosión cada 10ms. 

El **orden de llegada al colector** en el Fiat 1600 (orden de encendido 1-3-4-2) produce pulsos a 0ms, 10ms, 20ms, 30ms, 40ms, ...

Si los caños primarios tienen longitudes ligeramente diferentes (420, 415, 425, 418mm), los tiempos de arribo se desplazan en microsegundos. Eso no suena diferente por sí solo — lo importante es el rebote desde el extremo abierto.

---

## 6. Impedancia de radiación: el extremo abierto no es solo inversión de signo

El modelo simplificado de "inversión de signo en el extremo abierto" es válido solo para ondas cuya longitud de onda es mucho mayor que el diámetro del tubo (`ka << 1`, donde `k = 2πf/c` y `a` es el radio).

Para un tubo de 60mm de diámetro (`a = 0.03m`), esto falla por encima de:
```
f = c / (2π·a) = 343 / (2π·0.03) ≈ 1820 Hz
```

Por encima de ~2kHz, el extremo abierto **no refleja perfectamente** — parte de la energía se irradia como sonido al exterior (eso es lo que escuchamos) y la reflexión es incompleta y tiene una fase distinta.

### Modelo de impedancia de radiación

La impedancia acústica al final de un tubo circular abierto es:

```
Z_rad = ρc/A · [R_rad + j·X_rad]
```

donde para `ka < 1`:
```
R_rad ≈ (ka)²/2         (componente resistiva, energía irradiada)
X_rad ≈ 0.6133·ka       (componente reactiva, "end correction")
```

La **end correction** es importante: efectivamente el tubo actúa como si fuera un poco más largo de lo que es físicamente. Para un tubo de 60mm de diámetro, la corrección es:
```
ΔL = 0.6133 · a = 0.6133 · 0.03 = 18.4mm
```

Es decir, el caño "suena" como si midiera ~18mm más de lo que realmente mide.

### Implementación como filtro digital

La reflexión en el extremo abierto no es `r = -1` sino un filtro:

```
r(f) = (Z_rad - Z₀) / (Z_rad + Z₀)
```

Para implementar esto en tiempo discreto, se puede aproximar la impedancia de radiación con un filtro IIR de primer orden que capture correctamente el comportamiento hasta ~3kHz:

```c
// Filtro de terminación de extremo abierto
// Aproximación de Levine & Schwinger
typedef struct {
    float a1, b0, b1;  // coeficientes IIR
    float x1, y1;      // estados
} radiation_filter_t;

// Inicialización para tubo de radio 'a' metros, fs muestras/segundo
void radiation_filter_init(radiation_filter_t *rf, float a_m, float fs) {
    // Coeficiente de end correction
    float ka_nyq = M_PI * a_m * fs / 343.0f;  // ka en Nyquist
    float alpha = 0.6133f * a_m;               // end correction en metros
    // Delay adicional en muestras por end correction
    float n_end = alpha / (343.0f / fs);

    // Aproximación bilinear del filtro de Levine-Schwinger de primer orden
    float T = 1.0f / fs;
    float omega_c = 343.0f / (M_PI * a_m);   // frecuencia de corte aprox
    float wT = omega_c * T;
    rf->b0 = -(1.0f - wT/2.0f) / (1.0f + wT/2.0f);
    rf->b1 = 1.0f;
    rf->a1 = -rf->b0;
    rf->x1 = rf->y1 = 0.0f;
}

float radiation_filter_tick(radiation_filter_t *rf, float x) {
    float y = rf->b0 * x + rf->b1 * rf->x1 - rf->a1 * rf->y1;
    rf->x1 = x;
    rf->y1 = y;
    return y;
}
```

La señal de audio que se escucha es la **onda irradiada** en el extremo abierto, que es proporcional a la presión en el extremo multiplicada por `R_rad`. Esto explica por qué el escape suena más brillante a altas RPM (más energía en frecuencias donde `R_rad` es mayor).

---

## 7. Temperatura: gradiente dinámico y su efecto en el sonido

### El perfil de temperatura a lo largo del caño

La temperatura no es uniforme. Sigue aproximadamente este perfil:

```
Culata → caño primario → colector → caño secundario → silenciador → salida
850°C     600°C          500°C       380°C              280°C         230°C
```

Estos valores son para un motor en carga a altas RPM. En ralentí:

```
500°C     380°C          320°C       240°C              180°C         150°C
```

La velocidad del sonido a cada temperatura:
```
c(T°C) = 343 · √((T + 273.15) / 293.15)

c(850°C) = 343 · √(1123/293) = 343 · 1.957 = 671 m/s
c(500°C) = 343 · √(773/293)  = 343 · 1.623 = 557 m/s
c(230°C) = 343 · √(503/293)  = 343 · 1.309 = 449 m/s
c(20°C)  = 343 m/s            (referencia, aire frío)
```

Diferencia entre caño frío y caliente: **~95% más rápido**. Eso significa que el retardo de un caño primario de 420mm pasa de:
```
N_frio  = 0.420 · 48000 / 343 = 58.8 muestras
N_caliente = 0.420 · 48000 / 671 = 30.1 muestras
```

Las frecuencias de resonancia se **casi duplican** con el calentamiento. Esto es lo que hace que el escape de un motor frío suene oscuro y gruñidor, y el motor caliente suene más brillante y agudo.

### Modelo de calentamiento dinámico

La temperatura de los gases en el caño no es instantánea — depende del historial de carga del motor. Un modelo de primer orden es suficiente:

```c
typedef struct {
    float T_head;    // temperatura en culata (°C) — función de RPM y carga
    float T_outlet;  // temperatura en salida (°C)
    float tau_up;    // constante de tiempo subida (s), aprox 8-15s
    float tau_down;  // constante de tiempo bajada (s), aprox 30-60s
    float T_current_head;
    float T_current_outlet;
} exhaust_temp_t;

void exhaust_temp_tick(exhaust_temp_t *et, float T_target_head,
                       float T_target_outlet, float dt)
{
    float tau;
    // Subida más rápida que bajada (el gas caliente entra rápido, se enfría lento)
    tau = (T_target_head > et->T_current_head) ? et->tau_up : et->tau_down;
    et->T_current_head   += (T_target_head   - et->T_current_head)   * (dt/tau);
    et->T_current_outlet += (T_target_outlet - et->T_current_outlet) * (dt/tau);
}
```

La temperatura objetivo en culata como función de RPM y carga (throttle):
```
T_head_target = 300 + 550 · (RPM/RPM_max)^0.7 · throttle^0.5
```

(empírico, validar contra telemetría real si disponible)

### Gradiente a lo largo del caño: temperatura como función de posición

Cada sección del waveguide tiene su propia temperatura y por tanto su propio retardo. Para un caño primario dividido en 3 segmentos:

```
Seg1 (cerca culata, 140mm): T₁ = T_head
Seg2 (curva, 140mm):        T₂ = T_head · 0.75 + T_collector · 0.25
Seg3 (antes colector, 140mm): T₃ = T_collector
```

Cada segmento tiene su propio waveguide con su propio `N(T)`. Esto permite que el gradiente sea no lineal, que es más realista.

---

## 8. Curvas y cambios de sección: discontinuidades acústicas

### Cambio de sección circular

En cualquier punto donde el área del caño cambia de `A₁` a `A₂`, hay reflexión y transmisión:

```
Coef. de reflexión:    r  = (Z₂ - Z₁) / (Z₂ + Z₁) = (A₁ - A₂) / (A₁ + A₂)
Coef. de transmisión:  t  = 2·A₁ / (A₁ + A₂)        (en presión)
Coef. de transmisión:  t' = 2·A₂ / (A₁ + A₂)        (en velocidad/flujo)
```

Para un ensanchamiento de 38mm a 42mm:
```
A₁ = π·(0.019)² = 1.134 × 10⁻³ m²
A₂ = π·(0.021)² = 1.385 × 10⁻³ m²
r = (1.134 - 1.385) / (1.134 + 1.385) = -0.0995  ≈ -10%
```

Una reflexión del 10% en presión = 1% en energía. Pequeña pero audiblemente presente — crea una coloración en las frecuencias donde el retardo del segmento reflectante coincide con múltiplos de half-wavelength.

Implementación como junction de 2 puertos:

```c
void junction_area_change(float p_fwd_in, float p_rev_in,
                          float A1, float A2,
                          float *p_fwd_out, float *p_rev_out)
{
    float denom = A1 + A2;
    *p_rev_out = ((A1 - A2) / denom) * p_fwd_in + (2*A2 / denom) * p_rev_in;
    *p_fwd_out = (2*A1 / denom) * p_fwd_in + ((A2 - A1) / denom) * p_rev_in;
}
```

### Curvas: modelo completo

Una curva es acústicamente un cambio de dirección del campo de presión. Los efectos dependen del **parámetro de curvatura** `δ = a/R`, donde `a` es el radio del tubo y `R` es el radio de curvatura de la curva.

Para el Fiat con caños primarios de 21mm de radio y curvas de 55mm de radio:
```
δ = 21/55 = 0.38
```

Esto es una curva **apretada** (δ > 0.3 se considera curva cerrada), con efectos notables.

#### Efecto 1: Longitud de trayecto aumentada

La longitud adicional por una curva de ángulo θ (en radianes) con radio de curvatura R:
```
L_curva = R · θ
```

Para una curva de 120° (θ = 2.09 rad) con R = 55mm:
```
L_curva = 0.055 · 2.09 = 115mm
```

Esta longitud reemplaza la longitud recta que hubiera habido.

#### Efecto 2: Pérdida de alta frecuencia (bend filter)

La onda no puede girar sin perder energía en las frecuencias donde la longitud de onda es comparable al radio de curvatura. La frecuencia de corte del "bend filter":

```
f_bend = c / (2π · R)
```

Para R = 55mm y c = 600 m/s (caliente):
```
f_bend = 600 / (2π · 0.055) = 1736 Hz
```

Por encima de ~1.7 kHz, la curva actúa como filtro pasa-bajos con pendiente suave. Implementación como filtro de primer orden:

```c
// Filtro de curva: primera orden, frecuencia de corte f_bend
// Aplicar en el punto de la curva en el waveguide
typedef struct { float a1, b0, b1, x1, y1; } bend_filter_t;

void bend_filter_init(bend_filter_t *bf, float R_m, float c, float fs) {
    float f_c = c / (2.0f * M_PI * R_m);
    float wc_T = 2.0f * M_PI * f_c / fs;
    // Bilinear transform del primer orden
    float K = wc_T / 2.0f;
    bf->b0 = K / (1.0f + K);
    bf->b1 = bf->b0;
    bf->a1 = -(1.0f - K) / (1.0f + K);
    bf->x1 = bf->y1 = 0.0f;
}
```

#### Efecto 3: Reflexión parcial en la curva

La curva actúa como una discontinuidad acústica. El coeficiente de reflexión depende de `δ` y la frecuencia, pero una aproximación de primer orden es:

```
r_bend ≈ -0.5 · δ² = -0.5 · (a/R)² = -0.5 · (0.38)² ≈ -0.072
```

Es decir, ~7% de la onda se refleja de vuelta. Esto contribuye al timbre "propio" de cada geometría de escape.

---

## 9. Nonlinearidades: por qué suena distinto a full throttle

### Steepening: endurecimiento del frente de onda

En la acústica lineal, la velocidad del sonido es constante. Pero para amplitudes grandes (como los pulsos de escape bajo carga), la velocidad de propagación depende de la presión local:

```
c_local = c₀ + (γ+1)/2 · v_partícula
```

donde `v_partícula = (p⁺ - p⁻) / (ρ·c)` es la velocidad local del gas.

Las partes del pulso de mayor presión viajan más rápido que las partes de menor presión. Esto **endurece el frente de onda** con el tiempo — el frente de ataque se hace más abrupto, generando armónicos superiores que no estaban en el pulso original. Es la misma física que convierte una onda sinusoidal en una onda triangular y finalmente en una onda de choque (shockwave).

En escapes de motores de competición (F1, MotoGP) se produce un fenómeno audible conocido como el "crack" del escape — literalmente micro-shockwaves viajando por el caño.

### Implementación: Burgers equation simplificada

El modelo de Burgers es la ecuación de onda con corrección no-lineal de primer orden. En el dominio discreto, se puede implementar como una corrección del retardo del waveguide en función de la amplitud local:

```c
// Corrección no-lineal: el retardo efectivo del waveguide cambia con la amplitud
float nonlinear_delay_correction(float p_plus, float Z0, float gamma) {
    float v_particle = p_plus / Z0;  // velocidad de partícula aproximada
    float dc_fraction = (gamma + 1.0f) / 2.0f * v_particle / 343.0f;
    return dc_fraction;  // fracción de muestra de corrección
}
```

Esto se suma a la corrección de retardo fraccionario del allpass. El efecto es que el frente del pulso llega unos microsegundos antes de lo que indica el modelo lineal, generando discontinuidades que el waveguide convierte en armónicos. Solo se activa cuando la amplitud local supera un umbral (~5% de la presión de referencia, ~5kPa), ya que a bajas amplitudes la física es lineal.

### Saturación de velocidad de partícula

En el caso extremo (motor a plena carga, aceleración brusca), la velocidad del gas puede aproximarse a la velocidad del sonido. Cuando `M = v/c > 0.3` (número de Mach local), hay que incorporar correcciones de Mach. En la práctica para simulación de audio, lo más simple es una saturación suave de la amplitud de onda:

```c
float soft_clip(float x, float threshold) {
    if (fabsf(x) < threshold) return x;
    float sign = (x > 0) ? 1.0f : -1.0f;
    return sign * (threshold + (1.0f - threshold) * tanhf((fabsf(x) - threshold) / (1.0f - threshold)));
}
```

Aplicar esto a cada onda `p⁺` y `p⁻` antes de insertarla en el waveguide limita físicamente las amplitudes a valores realistas y agrega armónicos de compresión audibles.

---

## 10. Vibración mecánica del caño: coupling estructural-acústico

### Por qué el caño vibra de manera audible

La onda de presión interna ejerce fuerzas sobre las paredes del caño. El caño de acero tiene resonancias propias que pueden amplificar ciertos modos. Hay dos tipos relevantes:

**Modos axiales (caño como viga):** El caño entero vibra como una viga apoyada en los puntos de sujeción. La frecuencia de los modos depende de la longitud entre soportes y del momento de inercia de la sección:

```
f_beam_n = (n·π)²/2 · √(EI / (ρ_steel · A_steel)) / L²
```

Para acero, E = 200 GPa, ρ = 7850 kg/m³. Para un caño de 42mm de diámetro y 1.5mm de pared, entre soportes separados 250mm:

```
I = π/64 · (D₀⁴ - Dᵢ⁴) = π/64 · ((0.042)⁴ - (0.039)⁴) = 4.87 × 10⁻⁸ m⁴
A_steel = π/4 · (D₀² - Dᵢ²) = 1.92 × 10⁻⁴ m²
f_beam_1 ≈ 420 Hz  (depende fuertemente de la longitud libre)
```

**Modos de anillo (breathing):** El diámetro del caño oscila. Frecuencia ~16 kHz para geometría típica — fuera del rango de interés.

### El acoplamiento acústico-estructural

La presión interna `p_int` genera una fuerza de expansión radial en el caño. La respuesta del caño es un oscilador de segundo orden:

```
m·ẍ + c·ẋ + k·x = F = p_int · A_int
```

Donde `x` es el desplazamiento de la pared, `m` es la masa del segmento, `c` es el amortiguamiento (material + sujeciones), y `k` es la rigidez.

La vibración de la pared re-irradia presión al exterior:

```
p_ext_rad = -ρ_air · (∂²x/∂t²) · A_ext / (2π·r)
```

(modelo de piston simple a distancia r).

Esta señal es **distinta** a la que sale por el extremo del tubo — tiene una coloración propia de las resonancias del caño. A ciertas RPM, cuando la frecuencia de excitación del motor coincide con `f_beam`, el caño literalmente resuena y esa resonancia se escucha como un buzz o ring metálico.

### Implementación como oscilador por segmento

```c
typedef struct {
    float omega_0;  // frecuencia natural (rad/s)
    float zeta;     // factor de amortiguamiento (~0.01-0.05 para acero)
    float x;        // desplazamiento
    float v;        // velocidad
    float mass_per_area;  // masa / área (kg/m²)
} pipe_wall_osc_t;

float pipe_wall_tick(pipe_wall_osc_t *pw, float p_internal, float dt)
{
    // Fuerza normalizada
    float F = p_internal / pw->mass_per_area;
    // Integrador de Verlet (más estable que Euler para osciladores)
    float a = F - 2*pw->zeta*pw->omega_0*pw->v - pw->omega_0*pw->omega_0*pw->x;
    pw->v += a * dt;
    pw->x += pw->v * dt;
    // Re-irradiación exterior proporcional a aceleración
    return pw->mass_per_area * a;  // presión radiada (escalar de mezcla externo)
}
```

La señal de salida del oscilador se mezcla con la señal del extremo abierto con un coeficiente muy pequeño (~0.02-0.05). Su efecto principal no es en amplitud sino en timbre: agrega resonancias metálicas específicas de esta geometría de caño.

---

## 11. La admisión: el lado olvidado que modela el timbre de baja

La admisión también es un sistema de ondas. El colector de admisión del Fiat 1600 es un 4-en-1 (o en muchos casos un plenum + cuatro tubos individuales de longitud calculada). 

La onda de presión negativa que genera el pistón al bajar (carrera de admisión) viaja por el tubo de admisión, llega al plenum, y se refleja. Si esa reflexión llega cuando la válvula de admisión aún está abierta, empuja aire extra hacia el cilindro — es el **efecto de "charging" de la admisión**, análogo al scavenging del escape.

Para la simulación de audio, el colector de admisión importa por otra razón: el ruido de admisión (inlet noise) es audible como un "aspirado" a altas RPM. En los motorsport sonoros, el ruido de admisión es tan importante como el de escape.

### Modelo simplificado de admisión

El sistema de admisión es un waveguide por cilindro (longitud del tubo de admisión) conectado a un volumen (plenum, modelado como concentración de presión sin retardo, de capacitancia `C = V/ρc²`).

La presión en el plenum:
```
dp_plenum/dt = (q_total_in - q_throttle) / C
```

donde `q_throttle` es el flujo que entra por el acelerador (controlado por su apertura). Esta dinámica agrega el sonido característico de succión y la variación de presión en el intake que modifica la curva de par.

---

## 12. El pulso de combustión: la señal de entrada real

### La forma del pulso de presión en válvula de escape

El pulso que entra al caño de escape cuando se abre la válvula no es un impulso limpio — tiene una forma específica que depende de la carrera termodinámica:

**Fase 1 — Blowdown (10-30° antes del PMI):** La válvula de escape se abre mientras el pistón todavía está bajando. La presión en el cilindro es mucho mayor que la del caño (puede ser 3-5 bar vs. 1.1 bar). Se produce un **chorro de gas superpresionado** que entra al caño. El pulso tiene un frente de ataque muy abrupto (tiempo de subida ~0.5ms a altas RPM) seguido de una caída exponencial.

**Fase 2 — Displacement (barrio del pistón):** El pistón sube empujando los gases. La presión en el cilindro cae rápidamente hasta igualarse con la del caño. El flujo de masa es ahora más uniforme pero el gradiente de presión cae.

**Fase 3 — Cierre de válvula:** La válvula cierra. La columna de gas en movimiento crea un pulso de presión negativa breve (agua hammer effect).

### Modelo del pulso de blowdown

```c
typedef struct {
    float amplitude;     // presión pico en Pa (función de RPM y carga)
    float t_rise;        // tiempo de subida en segundos (~0.0005s)
    float t_decay;       // tiempo de caída exponencial (~0.003-0.006s)
    float t_negative;    // tiempo del pulso negativo de cierre (~0.001s)
    float neg_fraction;  // fracción negativa (~0.15-0.25)
} exhaust_pulse_params_t;

// Forma normalizada del pulso (en tiempo continuo):
// p(t) = A · [ (t/t_rise) · exp(-(t-t_rise)/t_decay)  para t > 0
//            - neg_fraction · (t-t_total)/t_negative  en el cierre ]
```

La **amplitud** del pulso está directamente relacionada con la presión en el cilindro al momento de apertura de la válvula:
```
A ≈ p_cylinder_at_EVO × (cylinder_bore² / pipe_diameter²)
```

A bajas RPM, la válvula se abre más tarde (el motor tiene tiempo de bajar más la presión), así que el pulso es más suave. A altas RPM, la apertura es relativamente más temprana en el ciclo y el pulso es más violento — eso es lo que genera el sonido de "crack" característico.

### Conexión con el modelo del motor existente

En `synth_s.h` ya existe un modelo de combustión que genera presión en función de RPM y throttle. Para conectarlo al waveguide, hay que extraer el **momento de apertura de válvula de escape** (EVO: Exhaust Valve Opening) y la **presión en ese momento**:

```c
// En synth_s.h o en el nuevo waveguide_exhaust_s.h:
float get_blowdown_pulse(synth_state_t *s, int cylinder) {
    float crank_angle = s->crank_angle_deg;
    float evo_angle = s->evo_deg[cylinder];  // ~50° antes PMI inferior
    
    if (is_valve_opening_event(crank_angle, evo_angle, s->prev_crank_angle)) {
        float p_cylinder = s->cylinder_pressure[cylinder];
        float p_exhaust  = s->exhaust_back_pressure;
        return (p_cylinder - p_exhaust) * PULSE_AMPLITUDE_FACTOR;
    }
    return 0.0f;
}
```

---

## 13. Efecto scavenging: la resonancia que da potencia y timbre

### Qué es el scavenging

Cuando la onda de presión del escape llega al extremo abierto del caño, se refleja como una **onda de rarefacción** (presión negativa). Esa onda de rarefacción viaja de vuelta hacia el motor. Si llega justo cuando la válvula de escape está cerrando y la de admisión está abriendo (período de overlap), la presión negativa "jala" los gases de escape residuales hacia afuera del cilindro y ayuda a que entre aire fresco.

Esto aumenta el llenado del cilindro (volumetric efficiency) — es decir, el motor hace más potencia. Y es por eso que los tubos de escape de alta performance tienen longitudes calculadas para que el timing sea correcto a la RPM de máxima potencia.

### La resonancia crea el "cant" característico

A las RPM donde el scavenging es efectivo, el motor produce más potencia. Entre esas RPM y las RPM adyacentes, el timing es incorrecto y el motor produce menos. Esto crea la **curva de torque ondulada** característica de los motores con escape de longitud sintonizada — lo que en motorsport se llama "powerband".

Para el audio, esto significa que a las RPM de resonancia del escape, la presión en el colector tiene **un máximo de amplitud** — el motor está "en el rango". Fuera del rango, la amplitud cae. Esto se escucha: el motor suena más lleno, más vibrante en las RPM donde el escape está sintonizado.

### Cálculo del tuning para el Fiat 4a1

La longitud óptima del caño primario para que el scavenging sea efectivo a una RPM dada:

```
L_opt = (c_hot · (240 - EVO_deg)) / (6 · RPM_target)
```

donde `EVO_deg` es el ángulo de apertura de la válvula de escape medido desde el PMI inferior (típicamente 40-60°), y `c_hot` es la velocidad del sonido en el caño caliente.

Para el Fiat a 5500 RPM con EVO = 50°, c = 600 m/s:
```
L_opt = (600 · (240 - 50)) / (6 · 5500) = 600 · 190 / 33000 = 3.45m
```

Wait — eso es la longitud total del recorrido de la onda (ida y vuelta parcial), incluyendo el caño secundario. La longitud del caño primario solo es típicamente 35-40% de ese total.

En la práctica para el Fiat 1600, los caños primarios de 415-425mm apuntan a un tuning alrededor de 5000-5500 RPM.

En el modelo waveguide, el scavenging emerge **naturalmente** sin ningún modelo especial: si los retardos son correctos, las reflexiones llegan cuando deben y el efecto se produce. No hay que programarlo explícitamente.

---

## 14. El silenciador: modelo de cámara con pérdidas

El silenciador es un volumen de expansión (cámara) que actúa como filtro acústico de paso bajo. El modelo más simple es una **cámara de expansión simple**:

```
Efecto de inserción: IL(f) = 10·log₁₀[1 + (m-1/m)² · sin²(kL)/4]
```

donde `m = S₂/S₁` es la razón de áreas (expansión) y `L` es la longitud de la cámara.

Para el waveguide, el silenciador se modela como:
1. Una **junction de expansión** (cambio de área de 60mm a 120mm de diámetro): refleja la mitad de la energía
2. Una **cámara de volumen** con retardo (la longitud de la cámara ~200mm)
3. Una **junction de contracción** de vuelta a 60mm: refleja la mitad
4. Un filtro de absorción de material absorbente (lana de vidrio, si hay): pasa-bajos de primer orden con f_c ~800-1200 Hz

La combinación de las dos reflexiones (una positiva, una negativa) crea cancelaciones en frecuencias específicas — los "stopbands" del silenciador. Esas son las frecuencias que el silenciador atenúa mejor, y no son planas — hay frecuencias que pasan casi sin atenuación ("passbands").

Un silenciador bien sintonizado no atenúa las frecuencias fundamentales del escape (que le dan el "carácter") sino los tonos de alta frecuencia que son irritantes.

---

## 15. Flujo de señal completo

```
MOTOR (synth_s.h)
    │
    ├── RPM, throttle, crank_angle
    │
    ▼
GENERADOR DE PULSOS (por cilindro)
    │ blowdown_pulse[4]  (muestras de audio, una por cilindro)
    │
    ▼
WAVEGUIDE CAÑOS PRIMARIOS (×4, en paralelo)
    │ Cada uno: 3 segmentos de waveguide + bend filter en curva + allpass fraccionario
    │ Temperatura diferente por segmento (dinámica, función de RPM)
    │ Nonlinear correction en amplitudes altas
    │
    ▼
SCATTERING MATRIX 4-EN-1 (junction del colector)
    │ Mezcla las 4 ondas avanzantes, devuelve 4 reflexiones a cada caño
    │ Una onda avanza al caño secundario
    │
    ▼
WAVEGUIDE CAÑO SECUNDARIO
    │ Segmento más frío (500°C → 380°C)
    │ Mayor diámetro (52mm)
    │
    ▼
SILENCIADOR
    │ Junction de expansión → cámara → junction de contracción
    │ Filtro de absorción interno
    │
    ▼
WAVEGUIDE CAÑO DE SALIDA (cat-back)
    │
    ▼
FILTRO DE RADIACIÓN (impedancia de extremo abierto)
    │ Reflexión de vuelta al waveguide
    │ Señal radiada = salida de audio principal
    │
    ▼
SUMA DE SEÑALES
    ├── Radiación por extremo abierto (componente principal, 70-80%)
    ├── Vibración de pared del caño (timbre metálico, 5-15%)
    └── Radiación del caño secundario (si hay fugas, 5-10%)
    │
    ▼
SEÑAL DE AUDIO FINAL
```

---

## 16. Parámetros JSON para el Fiat 4a1

```json
{
  "exhaust_model": "waveguide_physical",
  "engine": {
    "cylinders": 4,
    "firing_order": [1, 3, 4, 2],
    "evo_deg": 50,
    "ivo_deg": 15,
    "ivc_deg": 55,
    "evc_deg": 15,
    "overlap_deg": 30,
    "bore_mm": 79.0,
    "stroke_mm": 81.5,
    "compression_ratio": 10.1
  },
  "primary_pipes": [
    { "cylinder": 1, "length_mm": 420, "diameter_mm": 42, "bend_angle_deg": 120, "bend_radius_mm": 55 },
    { "cylinder": 2, "length_mm": 415, "diameter_mm": 42, "bend_angle_deg": 115, "bend_radius_mm": 55 },
    { "cylinder": 3, "length_mm": 425, "diameter_mm": 42, "bend_angle_deg": 125, "bend_radius_mm": 55 },
    { "cylinder": 4, "length_mm": 418, "diameter_mm": 42, "bend_angle_deg": 118, "bend_radius_mm": 55 }
  ],
  "collector": {
    "type": "4into1",
    "cone_half_angle_deg": 18
  },
  "secondary_pipe": {
    "length_mm": 380,
    "diameter_mm": 52
  },
  "muffler": {
    "type": "expansion_chamber",
    "inlet_diameter_mm": 52,
    "chamber_diameter_mm": 120,
    "chamber_length_mm": 200,
    "outlet_diameter_mm": 52,
    "absorption_fc_hz": 1000,
    "absorption_coefficient": 0.4
  },
  "tailpipe": {
    "length_mm": 250,
    "diameter_mm": 52
  },
  "temperature": {
    "head_hot_C": 850,
    "head_cold_C": 300,
    "outlet_hot_C": 260,
    "outlet_cold_C": 120,
    "warmup_tau_s": 12,
    "cooldown_tau_s": 45
  },
  "pipe_wall": {
    "material": "steel",
    "thickness_mm": 1.5,
    "support_spacing_mm": 250,
    "damping_ratio": 0.025,
    "wall_radiation_mix": 0.04
  },
  "nonlinear": {
    "enabled": true,
    "steepening_gamma": 1.35,
    "clip_threshold": 0.85
  },
  "sample_rate": 48000,
  "output_level": 0.8
}
```

---

## 17. Costo computacional y optimizaciones

### Por qué el waveguide es MÁS económico que la convolución

La convolución actual en `convo_filter_s.h` con 8192 coeficientes requiere:
```
8192 multiplicaciones + 8191 sumas = ~16.000 operaciones por muestra
```

El waveguide completo (4 caños primarios × 3 segmentos + scattering + secundario + silenciador) requiere aproximadamente:
```
4 × 3 × 2 reads/writes = 24 buffer ops
+ 4 allpass filters = 4 × 4 ops = 16 ops
+ scattering = 20 ops
+ bend filters = 4 × 4 = 16 ops
+ radiation filter = 4 ops
+ wall oscillators = 4 × 6 = 24 ops
Total: ~105 operaciones por muestra
```

Es decir, el modelo físico completo consume **150 veces menos CPU** que la convolución actual, y modela la física real con dependencias dinámicas que la convolución no puede tener.

### Tamaño de memoria

El waveguide más largo (caño secundario frío, c = 449 m/s, L = 380mm):
```
N = 0.380 × 48000 / 449 = 40.6 ≈ 41 muestras
```

Los 4 caños primarios (calientes, c ~600 m/s, L ~420mm):
```
N = 0.420 × 48000 / 600 = 33.6 ≈ 34 muestras
```

Memoria total de buffers (float × 2 direcciones × N muestras por waveguide):
```
≈ 4 × (3 × 34 × 2) + 41×2 + silenciador ≈ 900 floats ≈ 3.6 kB
```

Comparado con los 8192 doubles de la IR actual: **18× menos memoria**.

### Optimizaciones clave

1. **SIMD para el scattering**: Las 4 sumas de admitancias son perfectas para vectorización (SSE/NEON 4-wide float).

2. **Temperatura por bloque**: En lugar de recalcular el retardo fraccionario cada muestra, recalcular cada bloque de 64 o 128 muestras. La temperatura no cambia en 1.3ms — el error de interpolación es inaudible.

3. **Precomputation del coeficiente allpass**: Cuando la temperatura cambia suavemente, el coeficiente `a` del allpass puede rampearse linealmente entre actualizaciones del bloque en lugar de recalcularse cada muestra.

4. **Muffler en frecuencia**: Si el silenciador tiene más de 2 cámaras, conviene moverlo a dominio de frecuencia (FFT-based convolution de longitud corta, <64 coeficientes) mientras el waveguide principal queda en tiempo.

---

## 18. Ruta de implementación por etapas

### Etapa 0 — Base: waveguide de tubo simple (1 semana)
**Archivo nuevo:** `waveguide_tube_s.h`

Implementar el waveguide básico de 1 tubo: buffer circular de ida y vuelta, reflexión pura en los extremos (r=−1 en abierto, r=+1 en cerrado), sin temperatura variable. Conectar la salida de presión del motor (un único cilindro) a la entrada. Verificar que las resonancias del caño emergen correctamente a las frecuencias esperadas.

**Test:** tubo de 500mm con r=-1 debe resonar a c/(2×0.5) = 343 Hz y sus armónicos.

---

### Etapa 1 — Retardo fraccionario (2-3 días)
**Modificar:** `waveguide_tube_s.h`

Agregar allpass de primer orden en cada extremo del waveguide. Verificar que la frecuencia de resonancia con L=420mm es exacta al Hz.

---

### Etapa 2 — Temperatura dinámica (3-4 días)
**Archivo nuevo:** `exhaust_temp_s.h`

Implementar el modelo de calentamiento de primer orden. Conectar `T_head` y `T_outlet` al cálculo de `N` de cada waveguide. Rampar el coeficiente allpass cuando cambia la temperatura. Verificar que el sonido cambia entre motor frío y caliente.

---

### Etapa 3 — Scattering 4-en-1 (1 semana)
**Archivo nuevo:** `waveguide_junction_s.h`

Implementar la scattering matrix. Conectar los 4 caños primarios al colector y el colector al caño secundario. Verificar que el efecto scavenging emerge a las RPM correctas comparando con datos de dinamómetro del Fiat 1600.

---

### Etapa 4 — Curvas y geometría (4-5 días)
**Modificar:** `waveguide_tube_s.h`, `waveguide_junction_s.h`

Agregar bend filter y corrección de longitud por curvas. Agregar junctions de cambio de sección (38→42mm). Cargar geometría desde JSON.

---

### Etapa 5 — Extremo abierto real (3-4 días)
**Archivo nuevo:** `radiation_filter_s.h`

Implementar filtro de impedancia de radiación. Extraer la señal de audio como la componente irradiada (proporcional a `R_rad × p_extremo`).

---

### Etapa 6 — Silenciador (4-5 días)
**Archivo nuevo:** `muffler_s.h`

Implementar modelo de cámara de expansión: junction de entrada + waveguide interno + junction de salida + filtro de absorción.

---

### Etapa 7 — No-linealidades (1 semana)
**Modificar:** `waveguide_tube_s.h`

Agregar corrección de steepening sobre la onda `p+` en segmentos calientes (donde las amplitudes son más altas). Agregar soft clip. Verificar comportamiento diferenciado entre ralentí y full throttle.

---

### Etapa 8 — Vibración mecánica (3-4 días)
**Archivo nuevo:** `pipe_wall_s.h`

Implementar el oscilador de segundo orden por segmento de caño. Mezclar la señal con el nivel configurado en JSON. Verificar que los modos de bending del caño aparecen como resonancias en el espectro de la señal de salida.

---

### Etapa 9 — Admisión (1 semana, opcional)
**Archivo nuevo:** `waveguide_intake_s.h`

Modelo del colector de admisión para generar el ruido de intake característico. Conectar el plenum a los cilindros. Mezclar la señal de intake con la señal de escape (con atenuación, el intake es más suave).

---

**Resultado final:** un escape simulado que cambia dinámicamente con la temperatura (motor frío vs. caliente), las RPM (scavenging en la powerband), la carga (nonlinearidades), y que tiene el carácter específico de la geometría de caños del Fiat 4a1. Sin un solo sample pregrabado.
