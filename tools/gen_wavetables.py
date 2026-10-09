#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Marcel Benders
"""The WAVE engine's wavetables (firmware/src/eng_wave.c): 8 tables of 8 frames, all from formulas in this file.

  tools/gen_wavetables.py OUT.h [--wav DIR]

Each frame is a spectrum (harmonics 1..127, amplitude and phase), made from a formula: exact Fourier series of
piecewise-linear cycles (saw, pulse, sync, steps), additive sums (organ, harmonics, vowels) or the DFT of a smooth
cycle (2-operator FM). From the spectrum each frame is synthesised once per octave band, band-limited so that its
highest harmonic stays below 19.5 kHz over the band (WT_INC_MAX), in a power-of-two length of at least twice the
highest harmonic + 1, four times up to 15 harmonics, eight times up to 7. Linear interpolation's images are harmonics
too (harmonic h's at length - h, about (h / (length - h))^2 as loud), above the band's top one; the high bands, whose
harmonics are few and strong, get the longer cycles so the images that fold back past 22 kHz stay small
(tests/wave_test.c measures it).
A band whose content equals the next smaller one's (a table without high harmonics: ORGAN, VOX, FM) shares it.
One scale per frame (the peak over all its bands) keeps the level the same across the bands. No DC.

--wav DIR writes each table as a WAV of its 8 frames (band 0, each frame 32 cycles) to listen to.
"""
import cmath
import math
import random
import struct
import sys
from pathlib import Path

FS = 44100
H_MAX = 127                       # band 0's harmonics (256 samples)
BANDS = [(256, 127), (128, 63), (64, 31), (64, 15), (64, 7), (32, 3)]   # (length, highest harmonic)
TOP_HZ = 19500.0                  # a band is used while its highest harmonic stays below this
NFRAME = 8
NAMES = ["BASIC", "PWM", "SYNC", "VOX", "ORGAN", "FM", "HARM", "DIGI"]


# ---------------------------------------------------------------- spectra (index h: harmonic h, complex c_h)

def pwl(points):
    """exact Fourier coefficients c_1..c_127 of a piecewise-linear cycle: points (t, v), t in 0..1 ascending,
    the first at 0 and the last at 1; two points at the same t make a jump"""
    c = [0j] * (H_MAX + 1)
    for (t0, v0), (t1, v1) in zip(points, points[1:]):
        dt = t1 - t0
        if dt <= 0:
            continue
        s = (v1 - v0) / dt
        for h in range(1, H_MAX + 1):
            w = 2 * math.pi * h
            e0, e1 = cmath.exp(-1j * w * t0), cmath.exp(-1j * w * t1)
            i0 = (e0 - e1) / (1j * w)                          # integral of e^-iwt over the segment
            i1 = -dt * e1 / (1j * w) + (e0 - e1) / (1j * w) ** 2   # integral of (t - t0) e^-iwt
            c[h] += v0 * i0 + s * i1
    return c


def additive(amps, phase=-math.pi / 2):
    """c_h from amplitudes a_h (h from 1): a_h sin(h x) by default"""
    c = [0j] * (H_MAX + 1)
    for h, a in enumerate(amps, 1):
        if h <= H_MAX and a:
            c[h] = 0.5 * a * cmath.exp(1j * phase)
    return c


def dft(f, n=4096):
    """c_h of a smooth cycle f(x), x in 0..2 pi"""
    xs = [f(2 * math.pi * i / n) for i in range(n)]
    c = [0j] * (H_MAX + 1)
    for h in range(1, H_MAX + 1):
        acc = 0j
        for i, x in enumerate(xs):
            acc += x * cmath.exp(-2j * math.pi * h * i / n)
        c[h] = acc / n
    return c


def mix(a, b, k):
    return [x * (1 - k) + y * k for x, y in zip(a, b)]


# ---------------------------------------------------------------- the tables

def saw():
    return pwl([(0, 0), (0.5, 1), (0.5, -1), (1, 0)])


def square(d=0.5):
    return pwl([(0, 1), (d, 1), (d, -1), (1, -1)])


def tri():
    return pwl([(0, 0), (0.25, 1), (0.75, -1), (1, 0)])


def sine():
    return additive([1.0])


def t_basic():
    """sine, triangle, saw, square, morphing over the 8 frames (key shapes at frames 0, 7/3, 14/3, 7)"""
    keys = [sine(), tri(), saw(), square()]
    out = []
    for f in range(NFRAME):
        x = f * 3 / 7
        i = min(int(x), 2)
        out.append(mix(keys[i], keys[i + 1], x - i))
    return out


def t_pwm():
    """a pulse from 50 % to 5 % (its DC dropped)"""
    return [square(d) for d in (0.5, 0.42, 0.35, 0.28, 0.21, 0.15, 0.1, 0.05)]


def t_sync():
    """a saw hard-synced to the fundamental: the slave at 1 .. 5.5 times the pitch, restarted every cycle"""
    out = []
    for r in (1.0, 1.5, 2.0, 2.5, 3.0, 3.75, 4.5, 5.5):
        pts, k = [], 0
        while k / r < 1:                                       # the slave's cycles within the master's
            t0, t1 = k / r, min((k + 1) / r, 1.0)
            pts += [(t0, -1.0), (t1, -1.0 + 2.0 * (t1 - t0) * r)]
            k += 1
        out.append(pwl(pts))
    return out


def t_vox():
    """vowels A E I O U (and between) over a buzz: three formants (Hz, bandwidth) at a 110 Hz reference pitch"""
    vowels = {"A": [(800, 80), (1150, 90), (2900, 120)], "E": [(400, 70), (2000, 100), (2550, 120)],
              "I": [(280, 60), (2250, 100), (3000, 120)], "O": [(450, 70), (800, 80), (2830, 100)],
              "U": [(325, 60), (700, 70), (2530, 100)]}
    path = ["A", "E", "I", "O", "U"]
    out = []
    for f in range(NFRAME):
        x = f * 4 / 7
        i = min(int(x), 3)
        k = x - i
        fm = [(a[0] * (1 - k) + b[0] * k, a[1] * (1 - k) + b[1] * k)
              for a, b in zip(vowels[path[i]], vowels[path[i + 1]])]
        amps = []
        for h in range(1, 48):                                 # 47 x 110 Hz: up to 5.2 kHz
            hz = 110.0 * h
            env = sum(g / (1 + ((hz - fc) / bw) ** 2) for (fc, bw), g in zip(fm, (1.0, 0.6, 0.3)))
            amps.append(env / h ** 0.5 + 0.002)
        out.append(additive(amps, phase=0.0))                  # cosine phases: a glottal-like pulse
    return out


def t_organ():
    """drawbar registrations (8' 4' 2 2/3' 2' 1 3/5' 1 1/3' 1': harmonics 1 2 3 4 5 6 8), mellow to full"""
    regs = ["80000000", "80800000", "86800000", "88600000", "88808000", "88848000", "88868040", "88888888"]
    out = []
    for r in regs:
        amps = [0.0] * 8
        for d, h in zip(r[1:], (2, 3, 4, 5, 6, 8)):
            amps[h - 1] = 10 ** ((int(d) - 8) * 3 / 20) if int(d) else 0.0
        amps[0] = 10 ** ((int(r[0]) - 8) * 3 / 20)
        out.append(additive(amps))
    return out


def t_fm():
    """two-operator FM, the modulator at the carrier's pitch, its index 0 .. 8"""
    out = []
    for f in range(NFRAME):
        idx = 8.0 * (f / 7) ** 1.5
        c = dft(lambda x: math.sin(x + idx * math.sin(x)))
        out.append([v if abs(v) > 1e-5 else 0j for v in c])
    return out


def t_harm():
    """harmonics added: 1, then up to 2, 4, 8, .. 127 of them, each 1 / sqrt(h) (brighter than a saw)"""
    return [additive([h ** -0.5 if h <= min(2 ** f, H_MAX) else 0.0 for h in range(1, H_MAX + 1)])
            for f in range(NFRAME)]


def steps(levels):
    """a cycle of equal steps at the given levels"""
    n = len(levels)
    pts = []
    for i, v in enumerate(levels):
        pts += [(i / n, v), ((i + 1) / n, v)]
    return pwl(pts)


def quant_sine(q):
    """a sine rounded to q levels per half cycle: the steps at its exact crossings"""
    edges = {0.0, 1.0}
    for k in range(1, q + 1):
        t = math.asin((k - 0.5) / q) / (2 * math.pi)
        edges |= {t, 0.5 - t, 0.5 + t, 1 - t}
    edges = sorted(edges)
    pts = []
    for a, b in zip(edges, edges[1:]):
        v = round(math.sin(math.pi * (a + b)) * q) / q
        pts += [(a, v), (b, v)]
    return pwl(pts)


def t_digi():
    """hard digital spectra: a sine at 4, 2 and 1 levels, then stepped patterns (fixed seeds) and a 1-bit pattern"""
    rnd = random.Random(1234)
    pat16 = [rnd.uniform(-1, 1) for _ in range(16)]
    pat8 = [rnd.choice((-1, -0.5, 0.5, 1)) for _ in range(8)]
    bits = [1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, -1]
    return [quant_sine(4), quant_sine(2), quant_sine(1), steps([1, 0.33, -0.33, -1]), steps(pat8), steps(pat16),
            steps([-1 if i % 3 else 1 for i in range(12)]), steps(bits)]


TABLES = [t_basic, t_pwm, t_sync, t_vox, t_organ, t_fm, t_harm, t_digi]


# ---------------------------------------------------------------- synthesis

def synth(c, n, hmax):
    out = []
    for i in range(n):
        x = 0.0
        for h in range(1, hmax + 1):
            if c[h]:
                x += 2 * (c[h] * cmath.exp(2j * math.pi * h * i / n)).real
        out.append(x)
    return out


def table_hmax(frames):
    top = max(abs(v) for c in frames for v in c[1:])
    return max(h for c in frames for h in range(1, H_MAX + 1) if abs(c[h]) > top * 1e-4)


def band_plan(hm):
    """per band: (length, harmonics) the table needs there, smaller when the table has fewer harmonics"""
    plan = []
    for n, h in BANDS:
        he = min(h, hm)
        ln = 16
        while ln < (8 if he <= 7 else 4 if he <= 15 else 2) * (he + 1):
            ln *= 2
        plan.append((min(ln, n), he))
    return plan


def main(out, wav_dir=None):
    data, lvl, store = [], [], {}
    for ti, make in enumerate(TABLES):
        frames = make()
        assert len(frames) == NFRAME
        plan = band_plan(table_hmax(frames))
        waves = {key: [synth(c, key[0], key[1]) for c in frames] for key in set(plan)}
        scale = [0.92 * 32767 / max(abs(x) for w in waves.values() for x in w[f]) for f in range(NFRAME)]
        row = []
        for key in plan:
            if (ti, key) not in store:
                store[(ti, key)] = len(data)
                for f in range(NFRAME):
                    data += [max(-32767, min(32767, int(round(x * scale[f])))) for x in waves[key][f]]
            row.append((store[(ti, key)], key[0].bit_length() - 1))
        lvl.append(row)
        if wav_dir:
            w = [int(round(x * scale[f])) for f in range(NFRAME) for _ in range(32) for x in waves[plan[0]][f]]
            Path(wav_dir).mkdir(parents=True, exist_ok=True)
            pcm = struct.pack(f"<{len(w)}h", *w)
            hdr = struct.pack("<4sI4s4sIHHIIHH4sI", b"RIFF", 36 + len(pcm), b"WAVE", b"fmt ", 16, 1, 1, FS, FS * 2, 2,
                              16, b"data", len(pcm))
            (Path(wav_dir) / f"{ti + 1}_{NAMES[ti]}.wav").write_bytes(hdr + pcm)
    inc_max = [min(0x7FFFFFFF, int(TOP_HZ / h / FS * 2 ** 32)) for _, h in BANDS]
    L = ["/* generated by tools/gen_wavetables.py: the WAVE engine's tables (eng_wave.c) */", "#pragma once",
         "#include <stdint.h>", f"#define WT_NTBL {len(TABLES)}", f"#define WT_NFRM {NFRAME}",
         f"#define WT_NBAND {len(BANDS)}",
         f"/* {len(data)} samples, {2 * len(data)} B */",
         "/* band b of table t: frame f's samples at WT_DATA[WT_BAND[t][b] & 0xFFFFF] + (f << (WT_BAND[t][b] >> 24)),",
         " * 1 << (WT_BAND[t][b] >> 24) of them (one cycle) */",
         f"static const uint32_t WT_BAND[{len(TABLES)}][{len(BANDS)}] = {{"]
    for row in lvl:
        L.append("    {" + ", ".join(f"{o | lg << 24:#x}" for o, lg in row) + "},")
    L += ["};", "/* the largest phase increment band b serves (its highest harmonic below "
          f"{TOP_HZ / 1000:g} kHz); above the last: the last band */",
          f"static const uint32_t WT_INC_MAX[{len(BANDS)}] = {{" + ", ".join(f"{v}u" for v in inc_max) + "};",
          f"static const int16_t WT_DATA[{len(data)}] = {{"]
    for i in range(0, len(data), 16):
        L.append("    " + ", ".join(str(v) for v in data[i:i + 16]) + ",")
    L.append("};")
    Path(out).write_text("\n".join(L) + "\n")
    print(f"wavetables: {len(TABLES)} x {NFRAME} frames, {2 * len(data)} B -> {out}")


if __name__ == "__main__":
    a = sys.argv[1:]
    wav = None
    if "--wav" in a:
        i = a.index("--wav")
        wav = a[i + 1]
        del a[i:i + 2]
    main(a[0] if a else "felucca_wavetables.h", wav)
