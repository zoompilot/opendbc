"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The cylinder-status state machine decoded from MORE_GAS byte 7: the mode-to-state mapping
through the real parsers (both exit transients), the entry-ramp progress, the inferred
fuel-cut state with its confirm time and coupling gate, threshold boundaries, clearing
conditions, and the fallback for modes never observed on the wire.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import car_interface
from opendbc.sunnypilot.car.mazda.carstate_ext import (CYL_MODE_DEACTIVATED, CYL_MODE_ENTRY_FIRST,
                                                       CYL_MODE_ENTRY_LAST, CYL_MODE_NORMAL,
                                                       EB_CONFIRM_FRAMES, EB_LOCK_FRAMES,
                                                       EB_MIN_V_EGO, CylinderState)

MORE_GAS = 0x167

MODE_NORMAL = CYL_MODE_NORMAL
MODE_ENTRY_MID = 0x0D
MODE_EXIT = 0x1E
MODE_EXIT_2 = 0x28

# One gear line: 72 kph is 20 m/s, and 30 rpm/kph keeps the ratio flat like a locked
# torque converter does. Drifting or stepping the ratio emulates unlock or a shift.
KPH = 72.0
LINE = 30.0


def t_ns(i):
  return int(i * DT_CTRL * 1e9)


def more_gas(mode: int, ctr: int = 0) -> tuple[int, bytes, int]:
  dat = bytearray(8)
  dat[3] = ctr & 0xF
  dat[7] = mode
  return (MORE_GAS, bytes(dat), 0)


def feed_mode(CI, i: int, mode: int):
  """Three frames so the lazily registered message is decoded before the assert."""
  for j in range(3):
    CI.update([(t_ns(i + j), [more_gas(mode, i + j)])])
  return CI.CS.cyl_state


def frame(mode: int = MODE_NORMAL, gas: bool = False, v: float = 20.0,
          ratio: float = LINE, kph: float = KPH):
  return (mode, gas, v, ratio * kph, kph)


def prime_coupling(CS, ratio: float = LINE):
  """Fill the coupling window while powered, as a real lift-off does."""
  for _ in range(EB_LOCK_FRAMES + 5):
    CS.update_cylinder_deactivation(*frame(gas=True, ratio=ratio))


@pytest.fixture
def CI():
  return car_interface()


def test_more_gas_modes_through_parsers(CI):
  # The first feed also registers MORE_GAS with the parser.
  assert feed_mode(CI, 0, MODE_NORMAL) == CylinderState.normal
  assert feed_mode(CI, 10, CYL_MODE_ENTRY_FIRST) == CylinderState.entry
  assert feed_mode(CI, 20, MODE_ENTRY_MID) == CylinderState.entry
  assert feed_mode(CI, 30, CYL_MODE_ENTRY_LAST) == CylinderState.entry
  assert feed_mode(CI, 40, CYL_MODE_DEACTIVATED) == CylinderState.deactivated
  assert feed_mode(CI, 50, MODE_EXIT) == CylinderState.normal
  assert feed_mode(CI, 60, MODE_EXIT_2) == CylinderState.normal
  assert feed_mode(CI, 70, MODE_NORMAL) == CylinderState.normal


def test_entry_progress_bounds(CI):
  for mode, progress in ((CYL_MODE_ENTRY_FIRST, 0.0), (MODE_ENTRY_MID, 3 / 7), (CYL_MODE_ENTRY_LAST, 1.0)):
    CI.CS.update_cylinder_deactivation(mode, gas_pressed=True, v_ego=20.0,
                                       rpm=LINE * KPH, speed_kph=KPH)
    assert CI.CS.cyl_entry_progress == pytest.approx(progress)
  CI.CS.update_cylinder_deactivation(CYL_MODE_DEACTIVATED, gas_pressed=True, v_ego=20.0,
                                     rpm=LINE * KPH, speed_kph=KPH)
  assert CI.CS.cyl_entry_progress == 1.0
  CI.CS.update_cylinder_deactivation(MODE_NORMAL, gas_pressed=True, v_ego=20.0,
                                     rpm=LINE * KPH, speed_kph=KPH)
  assert CI.CS.cyl_entry_progress == 0.0


def test_fuel_cut_inference_needs_confirm_time(CI):
  prime_coupling(CI.CS)
  for _ in range(EB_CONFIRM_FRAMES - 1):
    CI.CS.update_cylinder_deactivation(*frame())
  assert CI.CS.cyl_state == CylinderState.normal
  CI.CS.update_cylinder_deactivation(*frame())
  assert CI.CS.cyl_state == CylinderState.engineBraking


def test_fuel_cut_inference_conditions(CI):
  # gas or low speed keeps the state normal; there is no decel gate (fuel cut
  # survives grades where the car holds or gains speed)
  for gas_pressed, v_ego in ((True, 20.0), (False, 2.0)):
    prime_coupling(CI.CS)
    for _ in range(EB_CONFIRM_FRAMES * 3):
      CI.CS.update_cylinder_deactivation(MODE_NORMAL, gas_pressed, v_ego,
                                         rpm=LINE * KPH, speed_kph=KPH)
    assert CI.CS.cyl_state == CylinderState.normal


def test_fuel_cut_latches_on_grade(CI):
  # downhill: speed climbs, pedal stays up, ratio stays pinned to the gear line
  prime_coupling(CI.CS)
  kph = KPH
  for _ in range(EB_CONFIRM_FRAMES + 5):
    kph += 0.02
    CI.CS.update_cylinder_deactivation(MODE_NORMAL, False, kph / 3.6,
                                       rpm=LINE * kph, speed_kph=kph)
  assert CI.CS.cyl_state == CylinderState.engineBraking


def test_fuel_cut_clears_on_gas(CI):
  prime_coupling(CI.CS)
  for _ in range(EB_CONFIRM_FRAMES + 5):
    CI.CS.update_cylinder_deactivation(*frame())
  assert CI.CS.cyl_state == CylinderState.engineBraking
  CI.CS.update_cylinder_deactivation(*frame(gas=True))
  assert CI.CS.cyl_state == CylinderState.normal


def test_fuel_cut_needs_coupling(CI):
  # ratio drifting like an unlocked converter never latches, pedal up or not
  prime_coupling(CI.CS)
  for i in range(EB_CONFIRM_FRAMES * 3):
    CI.CS.update_cylinder_deactivation(*frame(ratio=LINE + 0.5 * i))
  assert CI.CS.cyl_state == CylinderState.normal


def test_coupling_breaks_on_gear_change_and_resumes(CI):
  # a step to another gear line breaks the flat window, then refills and re-latches
  prime_coupling(CI.CS)
  for _ in range(EB_CONFIRM_FRAMES + 5):
    CI.CS.update_cylinder_deactivation(*frame())
  assert CI.CS.cyl_state == CylinderState.engineBraking
  for _ in range(EB_LOCK_FRAMES):
    CI.CS.update_cylinder_deactivation(*frame(ratio=LINE - 6.0))
  assert CI.CS.cyl_state == CylinderState.normal
  for _ in range(EB_LOCK_FRAMES + EB_CONFIRM_FRAMES):
    CI.CS.update_cylinder_deactivation(*frame(ratio=LINE - 6.0))
  assert CI.CS.cyl_state == CylinderState.engineBraking


def test_coupling_clears_below_ratio_speed(CI):
  prime_coupling(CI.CS)
  for _ in range(EB_CONFIRM_FRAMES * 3):
    CI.CS.update_cylinder_deactivation(MODE_NORMAL, False, 20.0, rpm=900.0, speed_kph=5.0)
  assert CI.CS.cyl_state == CylinderState.normal
  assert len(CI.CS.eb_ratio_hist) == 0


def test_fuel_cut_speed_threshold_is_strict(CI):
  # exactly at the speed threshold never latches
  prime_coupling(CI.CS)
  for _ in range(EB_CONFIRM_FRAMES * 3):
    CI.CS.update_cylinder_deactivation(MODE_NORMAL, False, EB_MIN_V_EGO,
                                       rpm=LINE * KPH, speed_kph=KPH)
  assert CI.CS.cyl_state == CylinderState.normal
  # one step past latches after the confirm time
  fr = (MODE_NORMAL, False, EB_MIN_V_EGO + 0.01, LINE * KPH, KPH)
  for _ in range(EB_CONFIRM_FRAMES - 1):
    CI.CS.update_cylinder_deactivation(*fr)
  assert CI.CS.cyl_state == CylinderState.normal
  CI.CS.update_cylinder_deactivation(*fr)
  assert CI.CS.cyl_state == CylinderState.engineBraking


@pytest.mark.parametrize("mode", [0x00, 0x12, 0x13, 0x15, 0x20, 0xFF])
def test_unobserved_modes_fall_back_to_normal(CI, mode):
  # 0x12/0x13 sit between the ramp top and the latch and never appeared in the capture;
  # the byte was observed jumping 0x11 straight to 0x14.
  CI.CS.update_cylinder_deactivation(mode, True, 20.0, rpm=LINE * KPH, speed_kph=KPH)
  assert CI.CS.cyl_state == CylinderState.normal
  assert CI.CS.cyl_entry_progress == 0.0


def test_state_names_match_capnp_enumerants():
  # card_ext assigns these strings into CarStateZP.CylinderDeactivation.State
  assert {s.value for s in CylinderState} == {"normal", "entry", "deactivated", "engineBraking"}
