/* SPDX-License-Identifier: GPL-3.0-only
 * Copyright (C) 2026 Marcel Benders */
/* Host test of the WAVE engine (engine 14, src/eng_wave.c; this fork's), the firmware's sources (hostsim.c):
 *   build/host/wave_test [DEMO_DIR]      (run_tests.sh builds and runs it; demos in build/wave_demo/)
 * 1. the tables (tools/gen_wavetables.py): 8 x 8 frames, every band of every table without DC and below full scale,
 *    a band's cycle never longer than the one below it.
 * 2. the sound: POS 0 of BASIC a sine (harmonics 2..10 under -40 dB), POS 127 a square (odd harmonics, the 3rd at
 *    -9.5 dB, the 2nd under -40 dB); the pitch (OSC 1, and OSC 2 with DTN 100 a semitone above); OFS (OSC 2 on
 *    another frame); the aliasing: the brightest frames (BASIC square, SYNC, DIGI) from C3 to C8, the energy off the
 *    harmonics below 10 kHz under -30 dB of the note's (the bands: no harmonic past 19.5 kHz).
 * 3. the modulation: ENV DEST SHP and LFO DEST SHP move POS (the spectrum changes over a note), a POS jump glides
 *    over a block (no larger step than the square's own edge), the filter TYPE.
 * 4. the presets: each bounded (no clip, no DC), audible, its voices free after the release; their modulation (LFO,
 *    INSERT) loaded with them and only with them; every preset's name and its category.
 * 5. favourites of engine 14 stay out of MIDI LEARN's bytes (favorites.factory[14]).
 * 6. the cost: 8 voices of WAVE (MIX on) against ANALOG's SAW LEAD, host ns per sample.
 * 7. demos (DEMO_DIR): every preset playing its suggested phrase (the dubstep ones at 140 BPM with drums), and a
 *    POS sweep through each table. */
#define main hostsim_main
#include "hostsim.c"
#undef main
#include "../firmware/src/category.c"
#include "../firmware/src/favorites.c"

static int bad;
static void check(const char *what, int ok)
{
    printf("wave: %-82s %s\n", what, ok ? "ok" : "FAIL");
    bad += !ok;
}

static int32_t out_buf[2 * CTL];
static void fresh(uint32_t e, uint32_t pi)        /* the boot state (no FX tails), track 1 = engine e preset pi */
{
    uint32_t k;
    memset(dly_buf, 0, sizeof dly_buf);
    memset(cho_buf, 0, sizeof cho_buf);
    memset(rev_comb, 0, sizeof rev_comb);
    memset(rev_ap, 0, sizeof rev_ap);
    memset(&fx, 0, sizeof fx);
    memset(&pf, 0, sizeof pf);
    for (k = 0; k < NTRK; k++)
        pf.mg[k] = 32768;
    perf_held = perf_latched = 0;
    lim_env = LIM_T;
    dc_l = dc_r = dce_l = dce_r = 0;
    vage = 0;
    rng_state = 0x1234567u;
    mod_seed = 0x2545F491u;
    memset(trk, 0, sizeof trk);
    memset(&mod, 0, sizeof mod);
    memset(sl, 0, sizeof sl);
    host_tracks_init();
    for (k = 0; k < NPART; k++)
        host_preset(&trk[k], k ? 0u : e, k ? 0u : pi);
    song.sel = 0;
    song.g[G_CLOCK] = 0;
    song.g[G_BPM] = 120;
    midi_beat_samples = 0;
}
static void dry(track_t *t) { t->p[P_DIST] = t->p[P_CHOR] = t->p[P_DLY] = t->p[P_REV] = 0; }

/* ------------------------------------------------------------- spectra --- */
#define NFFT 16384u
static double re_[NFFT], im_[NFFT];
static void fft(double *re, double *im, uint32_t n)
{
    uint32_t i, j = 0, len, k;
    for (i = 1; i < n; i++) {
        uint32_t bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            double t = re[i]; re[i] = re[j]; re[j] = t;
            t = im[i]; im[i] = im[j]; im[j] = t;
        }
    }
    for (len = 2; len <= n; len <<= 1) {
        double a = -2 * M_PI / len, wr = cos(a), wi = sin(a);
        for (i = 0; i < n; i += len) {
            double cr = 1, ci = 0;
            for (k = 0; k < len / 2; k++) {
                double ur = re[i + k], ui = im[i + k];
                double vr = re[i + k + len / 2] * cr - im[i + k + len / 2] * ci;
                double vi = re[i + k + len / 2] * ci + im[i + k + len / 2] * cr;
                re[i + k] = ur + vr; im[i + k] = ui + vi;
                re[i + k + len / 2] = ur - vr; im[i + k + len / 2] = ui - vi;
                double t = cr * wr - ci * wi;
                ci = cr * wi + ci * wr;
                cr = t;
            }
        }
    }
}

static int32_t ren[NFFT];
/* one note on track 1 (dry, POLY, the amp envelope open, velocity 40), setup(t) first; NFFT samples of the left output after 0.1 s */
static void one_note(uint32_t note, void (*setup)(track_t *t))
{
    track_t *t = &trk[0];
    uint32_t i, k;
    fresh(ENGI_WAVE, 6);                              /* SUB WAVE: no modulation */
    dry(t);
    t->p[P_VOICE] = V_POLY;
    t->p[P_ATK] = 0;
    t->p[P_SUS] = 127;
    t->p[P_E4] = 127;                                 /* the filter open, flat */
    t->p[P_E5] = 0;
    t->p[P_E7] = 0;
    t->p[P_ED_FLT] = 0;
    if (setup)
        setup(t);
    trk_note_on(t, note, 40);                         /* (quiet: the master's soft limit adds a 3rd at -33 dB at 100) */
    for (k = 0; k < FS / 10u / CTL; k++)
        mix_block(out_buf, CTL);
    for (k = 0; k < NFFT / CTL; k++) {
        mix_block(out_buf, CTL);
        for (i = 0; i < CTL; i++)
            ren[k * CTL + i] = out_buf[2u * i];
    }
    trk_note_off(t, note);
    for (k = 0; k < FS / CTL; k++)
        mix_block(out_buf, CTL);
}

/* the spectrum of ren (Blackman-Harris): the power of harmonic h (bins around h f0) into pw[h], 1 .. nh; returns the power off
 * the harmonics (DC's bins left out) */
static double off_below = FS / 2;                  /* the power off the harmonics counts below this (Hz) */
static double spectrum(double f0, double *pw, uint32_t nh)
{
    uint32_t i, h;
    double off = 0, bin = (double)FS / NFFT;
    for (i = 0; i < NFFT; i++) {
        double x = 2 * M_PI * i / NFFT;                /* Blackman-Harris: sidelobes under -92 dB */
        re_[i] = ren[i] * (0.35875 - 0.48829 * cos(x) + 0.14128 * cos(2 * x) - 0.01168 * cos(3 * x));
        im_[i] = 0;
    }
    fft(re_, im_, NFFT);
    for (h = 0; h <= nh; h++)
        pw[h] = 0;
    for (i = 4; i < NFFT / 2u; i++) {
        double p = re_[i] * re_[i] + im_[i] * im_[i], x = i * bin / f0;
        uint32_t hn = (uint32_t)(x + 0.5);
        if (hn >= 1 && fabs(i * bin - hn * f0) <= 6 * bin) {
            if (hn <= nh)
                pw[hn] += p;
        } else if (i * bin < off_below) {
            off += p;
        }
    }
    return off;
}
static double note_hz(uint32_t note) { return (double)pitch_inc(note * 16u) * FS / 4294967296.0; }
static double db(double a, double b) { return 10 * log10((a + 1e-30) / (b + 1e-30)); }

/* ---------------------------------------------------------------- 1 --- */
static void test_tables(void)
{
    uint32_t t, b, f, i, ok = 1, prev;
    double worst_dc = 0;
    int32_t peak = 0;
    for (t = 0; t < WT_NTBL; t++) {
        prev = 99;
        for (b = 0; b < WT_NBAND; b++) {
            uint32_t lg = WT_BAND[t][b] >> 24, n = 1u << lg;
            ok &= lg <= prev && (WT_BAND[t][b] & 0xFFFFFu) + (n << 3) <= NELEM(WT_DATA);
            prev = lg;
            for (f = 0; f < WT_NFRM; f++) {
                const int16_t *s = &WT_DATA[(WT_BAND[t][b] & 0xFFFFFu) + (f << lg)];
                double sum = 0;
                for (i = 0; i < n; i++) {
                    sum += s[i];
                    peak = abs(s[i]) > peak ? abs(s[i]) : peak;
                }
                worst_dc = fabs(sum / n) > worst_dc ? fabs(sum / n) : worst_dc;
            }
        }
    }
    printf("wave: tables: %u x %u frames, %u bands, %u B; peak %d, largest DC %.1f\n", WT_NTBL, WT_NFRM, WT_NBAND,
           (uint32_t)sizeof WT_DATA, peak, worst_dc);
    check("tables: 8 x 8 frames, each band within the data, no longer than the band below it", ok && WT_NTBL == 8 &&
          WT_NFRM == 8);
    check("tables: no DC (under 0.1 % of full scale), peaks at 92 % of full scale", worst_dc < 33 && peak <= 30200 &&
          peak > 29000);
}

/* ---------------------------------------------------------------- 2 --- */
static int32_t g_e[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
static void set_e(track_t *t)
{
    uint32_t k;
    for (k = 0; k < 8u; k++)
        if (g_e[k] >= 0)
            t->p[P_E0 + k] = (int16_t)g_e[k];
}
static void eset(int32_t tbl, int32_t pos, int32_t dtn, int32_t ofs, int32_t mix)
{
    g_e[0] = tbl;
    g_e[1] = pos;
    g_e[2] = dtn;
    g_e[3] = ofs;
    g_e[6] = mix;
}

/* the frequency of ren by its rising zero crossings (linear between samples) */
static double zc_hz(void)
{
    uint32_t i, n = 0;
    double first = 0, last = 0;
    for (i = 1; i < NFFT; i++)
        if (ren[i - 1] < 0 && ren[i] >= 0) {
            double x = i - 1 + (double)-ren[i - 1] / (ren[i] - ren[i - 1]);
            if (!n)
                first = x;
            last = x;
            n++;
        }
    return n > 1 ? (n - 1) * (double)FS / (last - first) : 0;
}

static void test_sound(void)
{
    static const uint8_t NOTES[] = {48, 60, 72, 84, 96, 108};
    double pw[64], off, f0, worst = -200, h2;
    uint32_t h, k, t, worst_note = 0, worst_t = 0, ok;
    /* POS 0: the sine; POS 127: the square */
    eset(0, 0, 0, 0, 0);
    one_note(57, set_e);
    f0 = note_hz(57);
    spectrum(f0, pw, 20);
    for (h = 2, ok = 1; h <= 10u; h++) {
        ok &= db(pw[h], pw[1]) < -40;
        printf("%s%.0f", h == 2u ? "wave: BASIC POS 0 at A3: harmonics 2..10 " : " ", db(pw[h], pw[1]));
    }
    printf(" dB\n");
    check("BASIC POS 0: a sine (harmonics 2..10 under -40 dB)", ok);
    eset(0, 127, 0, 0, 0);
    one_note(57, set_e);
    spectrum(f0, pw, 20);
    printf("wave: BASIC POS 127 at A3: harmonics 2 3 5 7 %.1f %.1f %.1f %.1f dB\n", db(pw[2], pw[1]), db(pw[3], pw[1]),
           db(pw[5], pw[1]), db(pw[7], pw[1]));
    check("BASIC POS 127: a square (3rd -9.5 dB, 5th -14 dB, +-1 dB; 2nd and 4th under -40 dB)",
          fabs(db(pw[3], pw[1]) + 9.54) < 1 && fabs(db(pw[5], pw[1]) + 13.98) < 1 && db(pw[2], pw[1]) < -40 &&
          db(pw[4], pw[1]) < -40);
    /* pitch: OSC 1; OSC 2 alone (MIX 127) with DTN 100 a semitone up */
    eset(0, 0, 0, 0, 0);
    one_note(69, set_e);
    f0 = zc_hz();
    eset(0, 0, 100, 0, 127);
    one_note(69, set_e);
    h2 = zc_hz();
    printf("wave: A4: OSC 1 %.2f Hz, OSC 2 with DTN 100 %.2f Hz (%.1f ct)\n", f0, h2, 1200 * log2(h2 / f0));
    check("pitch: OSC 1 at the note (A4 440 Hz +-1 ct), OSC 2 with DTN 100 a semitone above (+-2 ct)",
          fabs(1200 * log2(f0 / 440)) < 1 && fabs(1200 * log2(h2 / f0) - 100) < 2);
    /* OFS: OSC 2 alone reads POS + OFS: POS 0 OFS 127 is the square */
    eset(0, 0, 0, 127, 127);
    one_note(57, set_e);
    spectrum(note_hz(57), pw, 20);
    check("OFS: OSC 2 on POS + OFS (POS 0 + OFS 127 alone: the square's 3rd at -9.5 dB)", fabs(db(pw[3], pw[1]) + 9.54) < 1);
    /* aliasing: the brightest frames over the keyboard; what folds back to 10 .. 22 kHz is left out. Linear
     * interpolation's images are the most of it (harmonic h's at the band's length +- h, about (h / (length - h))^2):
     * -33 .. -56 dB, where ANALOG's polyBLEP saw is at -48 .. -65 dB */
    off_below = 10000;
    for (t = 0; t < 3u; t++) {
        static const int32_t TP_[3][2] = {{0, 127}, {2, 127}, {7, 127}};   /* BASIC square, SYNC x5.5, DIGI bits */
        for (k = 0; k < NELEM(NOTES); k++) {
            double tot = 0;
            eset(TP_[t][0], TP_[t][1], 0, 0, 0);
            one_note(NOTES[k], set_e);
            f0 = note_hz(NOTES[k]);
            off = spectrum(f0, pw, 63);
            for (h = 1; h <= 63u; h++)
                tot += pw[h];
            if (db(off, tot) > worst) {
                worst = db(off, tot);
                worst_note = NOTES[k];
                worst_t = t;
            }
        }
    }
    off_below = FS / 2;
    printf("wave: aliasing: the most off the harmonics below 10 kHz %.1f dB (%s, note %u)\n", worst,
           worst_t == 0 ? "BASIC square" : worst_t == 1 ? "SYNC x5.5" : "DIGI", worst_note);
    check("aliasing: BASIC square, SYNC x5.5, DIGI 1-bit, C3 to C8: off the harmonics below 10 kHz under -30 dB",
          worst < -30);
    eset(-1, -1, -1, -1, -1);
}

/* ---------------------------------------------------------------- 3 --- */
static double centroid(uint32_t from, uint32_t n, double f0)
{
    double pw[64], a = 0, b = 0;
    uint32_t h;
    memmove(ren, ren + from, n * sizeof ren[0]);
    memset(ren + n, 0, (NFFT - n) * sizeof ren[0]);
    spectrum(f0, pw, 63);
    for (h = 1; h <= 63u; h++) {
        a += h * pw[h];
        b += pw[h];
    }
    return a / (b + 1e-30);
}
static void env_shp(track_t *t)
{
    t->p[P_E0] = 6;                                   /* HARM: the more POS, the more harmonics */
    t->p[P_E1] = 0;
    t->p[P_ED_SHP] = 63;
    t->p[P_ATK] = 80;                                 /* ~0.33 s */
}
static void lfo_shp(track_t *t)
{
    t->p[P_E0] = 6;
    t->p[P_E1] = 0;
    t->p[P_LD_SHP] = 63;
    t->p[P_LPOL] = 1;                                 /* UNI: 0 .. +63 */
    t->p[P_LWAVE] = 3;                                /* SQR: the halves of a cycle on 0 and 63 */
    t->p[P_LSYNC] = 3;                                /* 1/1 at 120 BPM: 2 s a cycle */
}
static void pos_jump(track_t *t) { t->p[P_E0] = 0; t->p[P_E1] = 0; }

static void test_mod(void)
{
    double c0, c1, f0 = note_hz(57);
    int32_t step = 0, edge = 0;
    uint32_t i, k;
    one_note(57, env_shp);                            /* the attack sweeps POS 0 -> 63 */
    c1 = centroid(NFFT - 4096u, 4096u, f0);
    one_note(57, env_shp);
    c0 = centroid(0, 2048u, f0);
    printf("wave: ENV DEST SHP 63 on HARM: centroid harmonic %.2f at 0.1 s, %.2f at 0.4 s\n", c0, c1);
    check("ENV DEST SHP moves POS: HARM brighter as the attack rises", c1 > c0 * 1.5);
    one_note(57, lfo_shp);                            /* 0.1 .. 0.47 s: the LFO's first half (0) then .. */
    c0 = centroid(0, 4096u, f0);
    {
        track_t *t = &trk[0];
        fresh(ENGI_WAVE, 6);
        dry(t);
        t->p[P_ATK] = 0;
        t->p[P_SUS] = 127;
        t->p[P_E4] = 127;
        t->p[P_E5] = 0;
        t->p[P_E7] = 0;
        lfo_shp(t);
        trk_note_on(t, 57, 100);
        for (k = 0; k < (FS * 3u / 2u) / CTL; k++)    /* 1.5 s: the second half of the cycle */
            mix_block(out_buf, CTL);
        for (k = 0; k < NFFT / CTL; k++) {
            mix_block(out_buf, CTL);
            for (i = 0; i < CTL; i++)
                ren[k * CTL + i] = out_buf[2u * i];
        }
        trk_note_off(t, 57);
        c1 = centroid(0, 4096u, f0);
    }
    printf("wave: LFO DEST SHP 63 (SQR, 1/1): centroid harmonic %.2f, then %.2f half a cycle later\n", c0, c1);
    check("LFO DEST SHP moves POS: the two halves of a square LFO differ", fabs(c1 - c0) > 1.0);
    /* a POS jump of 127 between blocks glides: no step larger than the square's own edge */
    {
        track_t *t = &trk[0];
        fresh(ENGI_WAVE, 6);
        dry(t);
        t->p[P_ATK] = 0;
        t->p[P_SUS] = 127;
        t->p[P_E4] = 127;
        t->p[P_E5] = 0;
        t->p[P_E7] = 0;
        pos_jump(t);
        t->p[P_E1] = 127;
        trk_note_on(t, 45, 100);
        for (k = 0; k < FS / 2u / CTL; k++) {
            mix_block(out_buf, CTL);
            for (i = 1; i < CTL; i++)
                edge = abs(out_buf[2u * i] - out_buf[2u * i - 2u]) > edge ? abs(out_buf[2u * i] - out_buf[2u * i - 2u]) : edge;
        }
        for (k = 0; k < FS / 2u / CTL; k++) {
            t->p[P_E1] = (int16_t)(k & 1u ? 127 : 0);  /* sine / square every block */
            mix_block(out_buf, CTL);
            for (i = 1; i < CTL; i++)
                step = abs(out_buf[2u * i] - out_buf[2u * i - 2u]) > step ? abs(out_buf[2u * i] - out_buf[2u * i - 2u]) : step;
        }
        trk_note_off(t, 45);
        for (k = 0; k < FS / CTL; k++)
            mix_block(out_buf, CTL);
    }
    printf("wave: POS 0 <-> 127 every block: largest step %d (the square's edge %d)\n", step, edge);
    check("POS glides over a block: jumping 0 <-> 127 every block, no step past the square's edge", step <= edge);
}

/* ---------------------------------------------------------------- 4 --- */
static void test_presets(void)
{
    uint32_t pi, k, i, ok = 1, quiet = 0, loud = 0, dc = 0, free_ = 1, modok = 1, names = 1;
    for (pi = 0; pi < ENG_WAVE.npresets; pi++) {
        track_t *t = &trk[0];
        double sum = 0, sq = 0;
        int32_t peak = 0;
        uint32_t n = 0;
        static const uint8_t CH[3] = {41, 48, 53};
        fresh(ENGI_WAVE, pi);
        for (k = 0; k < (ENG_WAVE.presets[pi].mono ? 1u : 3u); k++)
            trk_note_on(t, CH[k] + (ENG_WAVE.presets[pi].mono ? 0u : 12u), 110);
        for (k = 0; k < FS / CTL; k++) {
            mix_block(out_buf, CTL);
            for (i = 0; i < 2u * CTL; i++) {
                sum += out_buf[i];
                sq += (double)out_buf[i] * out_buf[i];
                peak = abs(out_buf[i]) > peak ? abs(out_buf[i]) : peak;
                n++;
            }
        }
        for (k = 0; k < 3u; k++)
            trk_note_off(t, CH[k] + (ENG_WAVE.presets[pi].mono ? 0u : 12u));
        for (k = 0; k < 8u * FS / CTL; k++)
            mix_block(out_buf, CTL);
        printf("wave: preset %-9s rms %6.0f peak %6d DC %5.1f\n", ENG_WAVE.presets[pi].name, sqrt(sq / n), peak, sum / n);
        loud += peak >= 32767;
        quiet += sqrt(sq / n) < 1000;
        dc += fabs(sum / n) > sqrt(sq / n) * 0.02;
        for (k = 0; k < NVOICE; k++)
            free_ &= !t->v[k].active;
        names &= strlen(ENG_WAVE.presets[pi].name) <= 8u && preset_cat(ENGI_WAVE, pi) != CAT_OTHER;
        {   /* its modulation loaded, the rest of the track's at the defaults */
            const wave_set_t *s = WAVE_MOD[pi];
            for (k = 0; k < NELEM(WAVE_MOD[0]) && s[k].id; k++)
                modok &= t->p[s[k].id] == s[k].v;
        }
    }
    check("presets: none clips, none silent (rms over 1000), no DC (under 2 % of rms)", !loud && !quiet && !dc);
    check("presets: every voice free after the release", free_);
    check("presets: their modulation (LFO, INSERT, glide) is loaded with them", modok);
    check("presets: names of at most 8 letters, a category each (the dubstep ones BASS)", names &&
          preset_cat(ENGI_WAVE, 0) == CAT_BASS && preset_cat(ENGI_WAVE, 1) == CAT_BASS);
    /* the modulation only with WAVE: another engine's preset after a WAVE one is its own */
    fresh(ENGI_WAVE, 0);
    ok = trk[0].p[P_LD_SHP] == 30 && trk[0].p[P_LSYNC] == 6;
    host_tracks_init();
    host_preset(&trk[0], 0, 0);
    ok &= trk[0].p[P_LD_SHP] == 0;
    check("WOBBLE loads LFO SYNC 1/8 and SHP 30; ANALOG after it (a fresh track) none", ok);
}

/* ---------------------------------------------------------------- 5 --- */
static void test_favorites(void)
{
    uint8_t ml[32], sl;
    int ok;
    memset(&favorites, 0, sizeof favorites);
    memset(favorites.factory[14], 0xA5, 32);          /* MIDI LEARN's entries */
    memcpy(ml, favorites.factory[14], 32);
    ok = favorite_set(ENGI_WAVE, 3, 1) && favorite_set(ENGI_WAVE, 11, 1) && favorite_has(ENGI_WAVE, 3) &&
         favorite_has(ENGI_WAVE, 11) && !favorite_has(ENGI_WAVE, 4);
    ok &= !memcmp(ml, favorites.factory[14], 32);
    ok &= favorite_set(13, 1, 1) && favorite_has(13, 1) && !favorite_has(ENGI_WAVE, 1);   /* SLICE's own */
    sl = favorites.factory[13][0];
    ok &= favorite_set(ENGI_WAVE, 3, 0) && !favorite_has(ENGI_WAVE, 3) && favorites.factory[13][0] == sl;
    ok &= !favorite_set(ENGI_WAVE, 200, 1);
    check("favourites: WAVE's in factory[13]'s second half; MIDI LEARN's bytes and SLICE's stars untouched", ok);
    memset(&favorites, 0, sizeof favorites);
}

/* ---------------------------------------------------------------- 6 --- */
static double ns_per_sample(uint32_t e, uint32_t pi)
{
    static const uint8_t CH[8] = {48, 52, 55, 59, 60, 64, 67, 71};
    track_t *t = &trk[0];
    uint32_t k, r;
    uint64_t t0, best = ~0ull;
    for (r = 0; r < 5u; r++) {
        fresh(e, pi);
        t->p[P_VOICE] = V_POLY;
        t->p[P_SUS] = 127;
        for (k = 0; k < 8u; k++)
            trk_note_on(t, CH[k], 100);
        t0 = now_ns();
        for (k = 0; k < FS / 4u / CTL; k++)
            mix_block(out_buf, CTL);
        t0 = now_ns() - t0;
        best = t0 < best ? t0 : best;
    }
    return (double)best / (FS / 4u);
}
static void test_cost(void)
{
    double a = ns_per_sample(0, 0), w = ns_per_sample(ENGI_WAVE, 7);   /* ANALOG SAW LEAD, WAVE PPG PAD (MIX on) */
    printf("wave: cost, 8 voices (host -O2, the whole mix): ANALOG SAW LEAD %.1f ns, WAVE PPG PAD %.1f ns per sample "
           "(%.2f x)\n", a, w, w / a);
    check("cost: WAVE at most 1.5 x ANALOG (host)", w < a * 1.5);
}

/* ---------------------------------------------------------------- 7 --- */
static void put_frames(FILE *w, uint32_t *frames, uint32_t n)
{
    uint32_t k, j;
    for (k = 0; k < n / CTL; k++) {
        mix_block(out_buf, CTL);
        for (j = 0; j < CTL; j++)
            wav_put(w, out_buf[2u * j], out_buf[2u * j + 1u]);
        *frames += CTL;
    }
}
static void demo_preset(const char *dir, uint32_t pi)
{
    const preset_t *pr = &ENG_WAVE.presets[pi];
    uint32_t pat = pr->pat ? pr->pat - 1u : 0u, i, frames = 0, dub = pr->pat == 14u, bars = 4;
    track_t *t = &trk[0], *td = &trk[3];
    char path[512], nm[32];
    FILE *w;
    for (i = 0; pr->name[i] && i < 31u; i++)
        nm[i] = pr->name[i] == ' ' ? '_' : pr->name[i];
    nm[i] = 0;
    snprintf(path, sizeof path, "%s/%02u_%s.wav", dir, pi + 1u, nm);
    if (!(w = fopen(path, "wb")))
        return;
    wav_hdr(w, 0);
    fresh(ENGI_WAVE, pi);
    song.g[G_BPM] = dub ? 140 : 120;
    for (i = 0; i < 16u; i++) {                       /* as ui.c load_pat16 */
        uint8_t n = PATTERNS[pat].note[i], fl = PATTERNS[pat].flags[i];
        put_step(t, i, n ? 1u : 0u, &n, fl & 4u ? ST_TIE : n ? ST_NOTE : ST_REST, n ? fl & (SF_ACCENT | SF_SLIDE) : 0u);
    }
    t->p[P_SLEN] = 16;
    host_drums(td);
    if (dub) {                                        /* half time: kick on 1 and the "and" of 3, snare on 3, hats */
        for (i = 0; i < 16u; i++) {
            uint8_t n[4];
            uint32_t k = 0;
            if (i == 0u || i == 10u)
                n[k++] = 36;
            if (i == 8u)
                n[k++] = 38;
            if (i % 2u == 0u)
                n[k++] = 42;
            put_step(td, i, k, n, k ? ST_NOTE : ST_REST, i == 8u ? SF_ACCENT : 0u);
        }
        td->p[P_SLEN] = 16;
    } else {
        td->p[P_LEVEL] = 0;
    }
    transport_req = 1;
    put_frames(w, &frames, bars * 16u * (uint32_t)div_samples(2));
    transport_req = 2;
    put_frames(w, &frames, 2u * FS);
    fseek(w, 0, SEEK_SET);
    wav_hdr(w, frames);
    fclose(w);
}
static void demo_tables(const char *dir)
{
    uint32_t tb, k, frames = 0;
    char path[512];
    FILE *w;
    snprintf(path, sizeof path, "%s/tables_pos_sweep.wav", dir);
    if (!(w = fopen(path, "wb")))
        return;
    wav_hdr(w, 0);
    for (tb = 0; tb < WT_NTBL; tb++) {                /* each table: C3, POS 0 -> 127 over 3 s, then 0.5 s silence */
        track_t *t = &trk[0];
        fresh(ENGI_WAVE, 6);
        dry(t);
        t->p[P_REV] = 20;
        t->p[P_ATK] = 5;
        t->p[P_SUS] = 127;
        t->p[P_REL] = 40;
        t->p[P_E0] = (int16_t)tb;
        t->p[P_E4] = 127;
        t->p[P_E7] = 0;
        trk_note_on(t, 48, 100);
        for (k = 0; k < 3u * FS / CTL; k++) {
            t->p[P_E1] = (int16_t)(k * 127u / (3u * FS / CTL));
            put_frames(w, &frames, CTL);
        }
        trk_note_off(t, 48);
        put_frames(w, &frames, FS / 2u);
    }
    fseek(w, 0, SEEK_SET);
    wav_hdr(w, frames);
    fclose(w);
}

int main(int argc, char **argv)
{
    uint32_t pi;
    test_tables();
    test_sound();
    test_mod();
    test_presets();
    test_favorites();
    if (!getenv("NOCOST"))
        test_cost();
    if (argc > 1) {
        for (pi = 0; pi < ENG_WAVE.npresets; pi++)
            demo_preset(argv[1], pi);
        demo_tables(argv[1]);
        printf("wave: demos in %s: the %u presets playing their phrases (the dubstep ones at 140 BPM with drums), "
               "tables_pos_sweep\n", argv[1], (uint32_t)ENG_WAVE.npresets);
    }
    printf("wave_test: %s\n", bad ? "FAILED" : "all checks ok");
    return bad != 0;
}
