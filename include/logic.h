// Pure logic, no Arduino headers: tested on the host with `pio test -e native`.
#pragma once
#include <math.h>
#include <stdint.h>

namespace logic {

// Wrap-safe elapsed time; a `since` stamped after `now` was read reports 0, not ~4.29e9.
inline uint32_t elapsedMs(uint32_t now, uint32_t since) {
  int32_t d = (int32_t)(now - since);
  return d < 0 ? 0 : (uint32_t)d;
}

// XPT2046 presence without a touch: an internal temperature conversion from a real chip is mid-range, steady, and
// followed by the chip's 3 trailing zero bits. A floating or stuck MISO reads 0/4095, jitters, or (with clock
// crosstalk) repeats a bit pattern that doesn't end in zeros.
// Samples are "normalised": 12 data bits << 4 | the trailing bits (see touch.cpp).
inline bool xptLooksAlive(const uint16_t* s, int n) {
  if (n < 4) return false;
  int lo = 4096, hi = -1;
  for (int i = 0; i < n; i++) {
    if (s[i] & 0x000E) return false;
    int v = s[i] >> 4;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  return lo >= 50 && hi <= 4045 && hi - lo <= 150;
}

// Raw touch -> screen: sx = a*rx + b*ry + c, sy = d*rx + e*ry + f.
// Covers swapped axes, mirrored axes and slight rotation, so one model fits every chip/orientation.
struct Affine {
  float a, b, c, d, e, f;
};

inline void apply(const Affine& m, float rx, float ry, float& sx, float& sy) {
  sx = m.a * rx + m.b * ry + m.c;
  sy = m.d * rx + m.e * ry + m.f;
}

// Least-squares fit over n >= 3 points. False when the raw points are (nearly) collinear, e.g. the same raw
// value came back for every target because one axis isn't wired.
inline bool fitAffine(const float* rx, const float* ry, const float* sx, const float* sy, int n, Affine& out) {
  if (n < 3) return false;
  double mx = 0, my = 0, msx = 0, msy = 0;
  for (int i = 0; i < n; i++) { mx += rx[i]; my += ry[i]; msx += sx[i]; msy += sy[i]; }
  mx /= n; my /= n; msx /= n; msy /= n;
  double sxx = 0, syy = 0, sxy = 0, xsx = 0, ysx = 0, xsy = 0, ysy = 0;
  for (int i = 0; i < n; i++) {
    double dx = rx[i] - mx, dy = ry[i] - my, ds = sx[i] - msx, dt = sy[i] - msy;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
    xsx += dx * ds; ysx += dy * ds; xsy += dx * dt; ysy += dy * dt;
  }
  double det = sxx * syy - sxy * sxy;
  if (sxx <= 0 || syy <= 0 || det <= 1e-6 * sxx * syy) return false;
  double a = (syy * xsx - sxy * ysx) / det, b = (sxx * ysx - sxy * xsx) / det;
  double d = (syy * xsy - sxy * ysy) / det, e = (sxx * ysy - sxy * xsy) / det;
  out = {(float)a, (float)b, (float)(msx - a * mx - b * my), (float)d, (float)e, (float)(msy - d * mx - e * my)};
  return true;
}

// Largest distance (px) between a fitted point and its target.
inline float maxError(const Affine& m, const float* rx, const float* ry, const float* sx, const float* sy, int n) {
  float worst = 0;
  for (int i = 0; i < n; i++) {
    float x, y;
    apply(m, rx[i], ry[i], x, y);
    float err = sqrtf((x - sx[i]) * (x - sx[i]) + (y - sy[i]) * (y - sy[i]));
    if (err > worst) worst = err;
  }
  return worst;
}

// The same calibration in the "map()" form most CYD projects use:
//   x = map(swapXY ? ry : rx, xMin, xMax, 0, w - 1);   y = map(swapXY ? rx : ry, yMin, yMax, 0, h - 1);
// (drops the small rotation term the affine model can carry).
struct SimpleCal {
  bool swapXY;
  int xMin, xMax, yMin, yMax;
};

inline bool toSimple(const Affine& m, int w, int h, SimpleCal& out) {
  double det = (double)m.a * m.e - (double)m.b * m.d;
  if (fabs(det) < 1e-12) return false;
  // inverse: raw = inv * (screen - offset)
  auto raw = [&](double sx, double sy, double& rx, double& ry) {
    double u = sx - m.c, v = sy - m.f;
    rx = (m.e * u - m.b * v) / det;
    ry = (-m.d * u + m.a * v) / det;
  };
  out.swapXY = fabs(m.b) > fabs(m.a);
  double l[2], r[2], t[2], btm[2];
  raw(0, (h - 1) / 2.0, l[0], l[1]);
  raw(w - 1, (h - 1) / 2.0, r[0], r[1]);
  raw((w - 1) / 2.0, 0, t[0], t[1]);
  raw((w - 1) / 2.0, h - 1, btm[0], btm[1]);
  int xi = out.swapXY ? 1 : 0, yi = 1 - xi;
  out.xMin = (int)lround(l[xi]);
  out.xMax = (int)lround(r[xi]);
  out.yMin = (int)lround(t[yi]);
  out.yMax = (int)lround(btm[yi]);
  return true;
}

// A tap: finger held >= 40 ms on-screen, reported once per press (resistive panels bounce at the edges).
struct TapFilter {
  static constexpr uint32_t TAP_HOLD_MS = 40;
  uint32_t downAt = 0;
  bool down = false, fired = false;
  bool update(bool pressed, uint32_t now, int x, int y, int w, int h) {
    if (!pressed) { down = fired = false; return false; }
    if (!down) { down = true; downAt = now; return false; }
    if (fired || elapsedMs(now, downAt) < TAP_HOLD_MS) return false;
    if (x < 0 || x >= w || y < 0 || y >= h) return false;
    fired = true;
    return true;
  }
};

// Median of up to 32 ints (sorts a copy).
inline int median(const int* v, int n) {
  int s[32];
  if (n > 32) n = 32;
  if (n <= 0) return 0;
  for (int i = 0; i < n; i++) s[i] = v[i];
  for (int i = 1; i < n; i++)
    for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { int t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
  return s[n / 2];
}

}  // namespace logic
