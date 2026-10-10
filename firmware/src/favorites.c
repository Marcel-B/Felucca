/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Leo Kuroshita (@kurogedelic), Hügelton Instruments */
#define FELUCCA_FAVORITES 1
/* Stable engine/preset references; fixed capacity independent of optional engines. */
typedef struct {
    uint8_t factory[16][32]; /* engine 0..15, preset 0..255 */
    uint32_t user, filter;
} favorites_t;
static favorites_t favorites __attribute__((section(".pool")));
/* the byte of engine e's preset k, 0 = none. factory[14] and [15] are taken (MIDI LEARN, the UI's settings), so
 * engine 14 (WAVE, this fork's) keeps its stars in factory[13]'s second half: SLICE has 2 presets, 128 fit there */
static uint8_t *favorite_byte(uint32_t e, uint32_t k)
{
    if (e == 14u)
        return k < 128u ? &favorites.factory[13][16u + k / 8u] : 0;
    return e < 14u && k < 256u && !(e == 13u && k >= 128u) ? &favorites.factory[e][k / 8u] : 0;
}
static int favorite_has(uint32_t engine, uint32_t preset)
{
    const uint8_t *b;
    if (engine == NENGINES)
        return preset < UP_SLOTS && ((favorites.user >> preset) & 1u);
    b = engine < NENGINES ? favorite_byte(engine, preset) : 0;
    return b && ((*b >> (preset % 8u)) & 1u);
}
static int favorite_set(uint32_t engine, uint32_t preset, int on)
{
    uint8_t *b;
    if (engine > NENGINES || (engine == NENGINES ? preset >= UP_SLOTS : !favorite_byte(engine, preset)))
        return 0;
    if (favorite_has(engine, preset) == !!on) return 0;
    if (engine == NENGINES) {
        if (on) favorites.user |= 1u << preset;
        else favorites.user &= ~(1u << preset);
    } else {
        b = favorite_byte(engine, preset);
        if (on) *b |= (uint8_t)(1u << (preset % 8u));
        else *b &= (uint8_t)~(1u << (preset % 8u));
    }
    return 1;
}
