#include <unity.h>

#include "logic.h"

using namespace logic;

void setUp() {}
void tearDown() {}

// Synthetic panel: screen x follows raw Y inverted, screen y follows raw X (swapped + mirrored, like many CYDs).
static void rawFor(float sx, float sy, float& rx, float& ry) {
  ry = 3800 - sx * 10.0f;
  rx = 300 + sy * 14.0f;
}

void test_fit_swapped_mirrored() {
  const float tx[4] = {30, 290, 290, 30}, ty[4] = {30, 30, 210, 210};
  float rx[4], ry[4];
  for (int i = 0; i < 4; i++) rawFor(tx[i], ty[i], rx[i], ry[i]);
  Affine m;
  TEST_ASSERT_TRUE(fitAffine(rx, ry, tx, ty, 4, m));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, maxError(m, rx, ry, tx, ty, 4));
  float cx, cy, x, y;
  rawFor(160, 120, cx, cy);
  apply(m, cx, cy, x, y);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 160, x);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 120, y);

  SimpleCal s;
  TEST_ASSERT_TRUE(toSimple(m, 320, 240, s));
  TEST_ASSERT_TRUE(s.swapXY);
  TEST_ASSERT_INT_WITHIN(1, 3800, s.xMin);         // raw Y at screen x = 0
  TEST_ASSERT_INT_WITHIN(1, 3800 - 3190, s.xMax);  // raw Y at screen x = 319
  TEST_ASSERT_INT_WITHIN(1, 300, s.yMin);
  TEST_ASSERT_INT_WITHIN(1, 300 + 239 * 14, s.yMax);
}

void test_fit_rejects_dead_axis() {
  const float tx[4] = {30, 290, 290, 30}, ty[4] = {30, 30, 210, 210};
  const float rx[4] = {500, 3500, 3500, 500}, ry[4] = {2000, 2000, 2000, 2000};  // Y never moves
  Affine m;
  TEST_ASSERT_FALSE(fitAffine(rx, ry, tx, ty, 4, m));
}

void test_fit_flags_bad_corners() {
  const float tx[4] = {30, 290, 290, 30}, ty[4] = {30, 30, 210, 210};
  float rx[4] = {500, 3500, 3500, 500}, ry[4] = {600, 600, 3400, 3400};
  rx[2] = 2000;  // one corner way off
  Affine m;
  TEST_ASSERT_TRUE(fitAffine(rx, ry, tx, ty, 4, m));
  TEST_ASSERT_TRUE(maxError(m, rx, ry, tx, ty, 4) > 20);
}

static uint16_t norm(int v, int trailing = 0) { return (uint16_t)((v << 4) | trailing); }

void test_xpt_alive() {
  uint16_t ok[8], rail0[8], rail1[8], jitter[8], pattern[8];
  for (int i = 0; i < 8; i++) {
    ok[i] = norm(760 + (i % 3));
    rail0[i] = 0;
    rail1[i] = 0xFFFF;
    jitter[i] = norm(i * 500 + 100);
    pattern[i] = 0xAAAA;  // clock crosstalk: in range and steady, but trailing bits aren't zero
  }
  TEST_ASSERT_TRUE(xptLooksAlive(ok, 8));
  TEST_ASSERT_FALSE(xptLooksAlive(rail0, 8));
  TEST_ASSERT_FALSE(xptLooksAlive(rail1, 8));
  TEST_ASSERT_FALSE(xptLooksAlive(jitter, 8));
  TEST_ASSERT_FALSE(xptLooksAlive(pattern, 8));
}

void test_tap_filter() {
  TapFilter f;
  TEST_ASSERT_FALSE(f.update(true, 1000, 10, 10, 320, 240));
  TEST_ASSERT_FALSE(f.update(true, 1020, 10, 10, 320, 240));
  TEST_ASSERT_TRUE(f.update(true, 1045, 10, 10, 320, 240));
  TEST_ASSERT_FALSE(f.update(true, 1100, 10, 10, 320, 240));  // once per press
  TEST_ASSERT_FALSE(f.update(false, 1120, 0, 0, 320, 240));
}

void test_elapsed_wrap() {
  TEST_ASSERT_EQUAL_UINT32(0, elapsedMs(100, 200));
  TEST_ASSERT_EQUAL_UINT32(20, elapsedMs(10, 0xFFFFFFF6u));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_fit_swapped_mirrored);
  RUN_TEST(test_fit_rejects_dead_axis);
  RUN_TEST(test_fit_flags_bad_corners);
  RUN_TEST(test_xpt_alive);
  RUN_TEST(test_tap_filter);
  RUN_TEST(test_elapsed_wrap);
  return UNITY_END();
}
