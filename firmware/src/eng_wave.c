/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Marcel Benders */
/* WAVE (engine 14, this fork's): two wavetable oscillators and ANALOG's filter.
 * A table (TBL) holds 8 frames, one cycle each; POS (0..127) moves through them, crossfading between the two frames
 * it stands between, as a PPG or Serum does. The tables are this firmware's own, made from formulas
 * (tools/gen_wavetables.py: BASIC PWM SYNC VOX ORGAN FM HARM DIGI), one band-limited copy per octave band; each
 * oscillator reads the copy its pitch needs (WT_INC_MAX), so no harmonic of it goes past 19.5 kHz.
 *   POS   the frame position of OSC 1 plus the SHP routings: ENV DEST SHP, LFO DEST SHP and the matrix's SHP move it
 *         (64 steps of shape a full sweep up), so a synced LFO on SHP is the wobble, an envelope on it the sweep of a
 *         pluck. The position glides over each block: no steps when it moves
 *   DTN   OSC 2 above OSC 1 in cents (0: the same pitch)
 *   OFS   OSC 2 reads at POS + OFS (0..127, at most the last frame): two frames at once
 *   MIX   OSC 2's share (0: OSC 2 off, no cost)
 *   CUT RES KTR and the filter TYPE (EDIT > FILTER, P_FTYPE) as ANALOG's: E4 E5 E7 sit where ANALOG has them
 * The presets set some of the track's modulation too (wave_track_loaded: the LFO, its SHP and FLT depths, the
 * INSERT), since a wavetable sound is mostly its movement */
#include "felucca_wavetables.h"
#define ENGI_WAVE 14u
static const char *const N_WT_TBL[] = {"BASIC", "PWM", "SYNC", "VOX", "ORGAN", "FM", "HARM", "DIGI"};
typedef char wt_names_fit[NELEM(N_WT_TBL) == WT_NTBL ? 1 : -1];

/* table tbl's band for phase increment inc: its frame 0, *lg = log2 of its length */
static inline const int16_t *wt_band(uint32_t tbl, uint32_t inc, uint32_t *lg)
{
    uint32_t b = 0, w;
    while (b < WT_NBAND - 1u && inc > WT_INC_MAX[b])
        b++;
    w = WT_BAND[tbl][b];
    *lg = w >> 24;
    return &WT_DATA[w & 0xFFFFFu];
}

/* POS (0..127 << 8, with the shape) -> the frame position, 7 frames in Q16, short of the last frame's end (so the
 * frame and the one after it are always both in the table) */
static inline int32_t wt_xpos(int32_t pos) { return clamp((pos * 14449) >> 10, 0, (7 << 16) - 1); }

/* one sample: frame f0 and the next (lg: their length), at phase ph, crossfaded by fr (Q14) */
static inline int32_t wt_read(const int16_t *f0, uint32_t lg, uint32_t ph, int32_t fr)
{
    const int16_t *f1 = f0 + (1u << lg);
    uint32_t j = ph >> (32u - lg), k = (j + 1u) & ((1u << lg) - 1u);
    int32_t pf = (int32_t)((ph << lg) >> 17);                       /* between two samples, Q15 */
    int32_t a = f0[j] + (((f0[k] - f0[j]) * pf) >> 15);
    int32_t b = f1[j] + (((f1[k] - f1[j]) * pf) >> 15);
    return a + (((b - a) * fr) >> 14);
}

static void wave_note_on(track_t *t, voice_t *v)
{
    (void)t;
    v->ph[1] = v->ph[0] + 0x40000000u;
    v->s[0] = v->s[1] = 0;                            /* filter */
    v->s[2] = v->s[3] = -1;                           /* frame positions: none yet (the first block starts on them) */
}

static void wave_render(track_t *t, voice_t *v, int32_t *out, uint32_t n, const vmod_t *m)
{
    const int16_t *p = t->p;
    uint32_t tbl = (uint32_t)p[P_E0] % WT_NTBL, lg1, lg2, i;
    int32_t pos = (p[P_E1] << 8) + m->shape - (64 << 8);
    int32_t x1 = wt_xpos(pos), x2 = wt_xpos(pos + (p[P_E3] << 8));
    int32_t det = p[P_E2], mix = p[P_E6];
    int32_t cut = (p[P_E4] << 8) + m->cutoff + (p[P_E7] * (v->pitch16 - 60 * 16) >> 4);
    uint32_t inc1 = m->inc, inc2 = det ? cents_inc(m->pitch16, det, m->fine) : inc1;
    const int16_t *b1 = wt_band(tbl, inc1, &lg1), *b2 = wt_band(tbl, inc2, &lg2);
    int32_t m2 = mix * 258, m1 = 32767 - m2;
    uint32_t ph0 = v->ph[0], ph1 = v->ph[1], ft = (uint32_t)p[P_FTYPE];
    int32_t ic1 = v->s[0], ic2 = v->s[1];             /* state in locals: out[] may alias v->s[] */
    int32_t a1 = v->s[2] < 0 ? x1 : v->s[2], a2 = v->s[3] < 0 ? x2 : v->s[3];
    int32_t d1 = (x1 - a1) / (int32_t)n, d2 = (x2 - a2) / (int32_t)n;
    int32_t fk = ANALOG_K(p[P_E5]);
    tsvf_t flt;
    tsvf_coef(&flt, cut, p[P_E5]);
    for (i = 0; i < n; i++) {
        int32_t s = wt_read(b1 + ((uint32_t)(a1 >> 16) << lg1), lg1, ph0, (a1 >> 2) & 0x3FFF);
        ph0 += inc1;
        a1 += d1;
        if (mix) {
            int32_t s2 = wt_read(b2 + ((uint32_t)(a2 >> 16) << lg2), lg2, ph1, (a2 >> 2) & 0x3FFF);
            s = mulq15(s, m1) + mulq15(s2, m2);
        }
        ph1 += inc2;
        a2 += d2;
        s = soft_knee(analog_tap(&flt, fk, s >> 1, &ic1, &ic2, ft), 16000) << 1;
        out[i] += voice_amp(s, m, i) << 1;
    }
    v->ph[0] = ph0;
    v->ph[1] = ph1;
    v->s[0] = ic1;
    v->s[1] = ic2;
    v->s[2] = x1;                                     /* where the next block starts (a1 is x1 less the rounding) */
    v->s[3] = x2;
}

/* {TBL, POS, DTN, OFS, CUT, RES, MIX, KTR}, {ATK DEC SUS REL}, ENV -> FILTER, mono; the sends; a pattern. The
 * dubstep basses want a phrase with long notes (PATTERNS 14 WOBBLE) and a tempo near 140 */
static const preset_t WAVE_PRESETS[] = {
    {"WOBBLE", {0, 96, 8, 0, 40, 70, 40, 64}, {0, 80, 110, 20}, 0, 1, FX(25, 0, 0, 10), PAT(14)},
    {"GROWL", {5, 50, 0, 10, 62, 50, 60, 64}, {0, 80, 115, 25}, 10, 1, FX(35, 0, 0, 10), PAT(14)},
    {"YOI BASS", {3, 64, 6, 18, 80, 40, 50, 64}, {0, 80, 115, 25}, 0, 1, FX(20, 0, 0, 15), PAT(14)},
    {"RIDDIM", {7, 30, 0, 0, 70, 30, 0, 64}, {0, 70, 110, 15}, 0, 1, FX(30, 0, 0, 5), PAT(14)},
    {"REESE", {0, 76, 20, 0, 58, 25, 64, 64}, {2, 80, 115, 40}, 0, 1, FX(10, 30, 0, 20), PAT(8)},
    {"SCREECH", {2, 20, 12, 30, 95, 75, 50, 64}, {0, 90, 100, 30}, 15, 1, FX(30, 0, 30, 25), PAT(4)},
    {"SUB WAVE", {0, 0, 0, 0, 60, 0, 0, 0}, {0, 60, 110, 20}, 0, 1, FX(0, 0, 0, 0), PAT(8)},
    {"PPG PAD", {6, 40, 10, 12, 85, 15, 64, 32}, {70, 90, 110, 95}, 5, 0, FX(0, 60, 25, 75), PAT(5)},
    {"VOX PAD", {3, 0, 7, 30, 90, 10, 64, 32}, {80, 90, 115, 95}, 0, 0, FX(0, 50, 20, 80), PAT(5)},
    {"DIGI BEL", {7, 0, 0, 40, 110, 10, 40, 64}, {0, 85, 0, 70}, 0, 0, FX(0, 20, 40, 45), PAT(7)},
    {"ORGAN", {4, 70, 4, 0, 110, 0, 40, 32}, {2, 70, 120, 30}, 0, 0, FX(0, 40, 10, 30), PAT(6)},
    {"PWM LEAD", {1, 20, 10, 25, 90, 25, 50, 64}, {5, 70, 100, 50}, 15, 1, FX(10, 20, 40, 30), PAT(4)},
};

/* the rest of a preset: track parameters it sets beyond preset_t (id, value; 0 ends). LFO SYNC 6 = 1/8, 8 = 1/16,
 * 5 = 1/4; LFO WAVE 0 SIN 1 TRI 3 SQR; POL 1 UNI; INSERT 1 SOFT 2 HARD 3 FOLD 4 FUZZ (A drive, B tone, C level: 112
 * 0 dB, two steps a dB; the basses come out near -16 dBFS rms, as loud as ANALOG's SQR BASS) */
typedef struct { uint8_t id; int8_t v; } wave_set_t;
static const wave_set_t WAVE_MOD[NELEM(WAVE_PRESETS)][10] = {
    {{P_LSYNC, 6}, {P_LWAVE, 0}, {P_LD_SHP, 30}, {P_LD_FLT, 50}, {P_ITYPE, 4}, {P_IA, 60}, {P_IB, 80},
     {P_IC, 108}},                                                         /* WOBBLE: 1/8, filter and frames */
    {{P_LSYNC, 8}, {P_LWAVE, 1}, {P_LD_SHP, 40}, {P_LD_FLT, 25}, {P_ITYPE, 3}, {P_IA, 70}, {P_IB, 90},
     {P_IC, 104}},                                                         /* GROWL: 1/16, the FM index, folded */
    {{P_LSYNC, 5}, {P_LWAVE, 1}, {P_LD_SHP, 63}, {P_ITYPE, 1}, {P_IA, 80}, {P_IB, 90}, {P_IC, 100}},
                                                                           /* YOI BASS: 1/4 through the vowels */
    {{P_LSYNC, 8}, {P_LWAVE, 3}, {P_LPOL, 1}, {P_LD_SHP, 50}, {P_ITYPE, 2}, {P_IA, 70}, {P_IB, 70}, {P_IC, 98}},
                                                                           /* RIDDIM: square steps of DIGI, 1/16 */
    {{P_LRATE, 20}, {P_LD_SHP, 6}, {P_ITYPE, 1}, {P_IA, 50}, {P_IB, 80}, {P_IC, 100}},   /* REESE: a slow drift */
    {{P_ED_SHP, 40}, {P_LRATE, 92}, {P_LD_SHP, 12}, {P_GLIDE, 30}, {P_ITYPE, 2}, {P_IA, 60}, {P_IB, 100},
     {P_IC, 98}},                                                          /* SCREECH: the sync sweeps up, fast */
    {{0, 0}},                                                              /* SUB WAVE */
    {{P_LRATE, 30}, {P_LWAVE, 1}, {P_LD_SHP, 20}, {P_ED_SHP, 20}},          /* PPG PAD: through the harmonics */
    {{P_LRATE, 25}, {P_LWAVE, 1}, {P_LPOL, 1}, {P_LD_SHP, 63}},            /* VOX PAD: A to U and back */
    {{P_ED_SHP, 63}},                                                      /* DIGI BEL: the envelope through DIGI */
    {{P_LRATE, 70}, {P_LD_PIT, 3}},                                        /* ORGAN: a little vibrato */
    {{P_LRATE, 50}, {P_LWAVE, 1}, {P_LD_SHP, 20}, {P_GLIDE, 25}},           /* PWM LEAD: the width moves */
};

/* a factory sound load (a preset, an engine change; ui.c apply_preset_to): the preset's modulation. Every other
 * track parameter of the sound is at its default already (apply_preset_to) */
static void wave_track_loaded(track_t *t)
{
    const wave_set_t *s;
    uint32_t k;
    if (t->eng_req != ENGI_WAVE || t->preset >= NELEM(WAVE_PRESETS))
        return;
    s = WAVE_MOD[t->preset];
    for (k = 0; k < NELEM(WAVE_MOD[0]) && s[k].id; k++)
        t->p[s[k].id] = s[k].v;
}

static const engine_t ENG_WAVE = {
    .name = "WAVE",
    .page_title = {"OSC", "FLT"},
    .edit = {
        {"TBL", F_ENUM, 0, WT_NTBL - 1, 0, N_WT_TBL, 0},
        {"POS", F_INT, 0, 127, 0, 0, 0},
        {"DTN", F_INT, 0, 127, 10, 0, "ct"},
        {"OFS", F_INT, 0, 127, 0, 0, 0},
        {"CUT", F_CUTOFF, 0, 127, 100, 0, 0},
        {"RES", F_PCT, 0, 127, 20, 0, 0},
        {"MIX", F_PCT, 0, 127, 64, 0, 0},
        {"KTR", F_PCT, 0, 127, 64, 0, 0},
    },
    .presets = WAVE_PRESETS,
    .npresets = NELEM(WAVE_PRESETS),
    .note_on = wave_note_on,
    .render = wave_render,
    .knob = {P_E1, P_E4, P_ATK, P_REL},
    .keep = 0x03,                /* the filter */
};
