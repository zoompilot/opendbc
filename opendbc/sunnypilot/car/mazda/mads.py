"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.mazda import mazdacan
from opendbc.sunnypilot.car.mazda.icbm import BUTTONS
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from opendbc.sunnypilot.mads_base import MadsCarStateBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert

TJA_MRCC_MAX_TX_FRAMES = 3
TJA_MRCC_RAW_OFF_CONFIRM_FRAMES = 5
# The body ECU drops discrete presses faster than one per 200 ms (measured, docs/zoompilot/icbm.md);
# the arm clears in PEDALS ~80 ms after a press plus the 50 ms confirm.
TJA_MRCC_TX_PERIOD = 0.2  # s between undo presses
# The press-induced arm appears ~80 ms after the press edge (docs/zoompilot/mazda-lateral.md).
TJA_MRCC_ARM_WAIT_FRAMES = int(1.0 / DT_CTRL)
# The white wheel waits this long on a fully-off, quiet cruise before it displays.
MADS_WHITE_HUD_OFF_CONFIRM_FRAMES = int(0.5 / DT_CTRL)


class MadsCarController:
  """The TJA button as the MADS switch, on the CarController (its packer, frame and
  last_button_frame): the press-induced MRCC arm undone, and the white wheel on the camera's
  own HUD frame."""

  def __init__(self):
    self.tja_button_prev = False
    self.mrcc_armed_prev: bool | None = None
    self.mrcc_undo_pending = False
    self.mrcc_undo_saw_armed = False
    self.mrcc_undo_frames = 0
    self.mrcc_raw_off_frames = 0
    self.mrcc_arm_wait_frames = 0
    # The white wheel rides the alert frame; on_bus tracks that the white bit is on the wire.
    self.mads_white_hud_off_frames = 0
    self.mads_white_hud_on_bus = False

  def update_mrcc_cleanup(self, CC, CC_SP, CS, handback_active: bool):
    # Undo the MRCC arm a physical TJA press causes: the wheel's press reaches every bus-0
    # ECU directly, so the button the driver declared as the lateral switch also arms cruise.
    # Answer with the driver's own MRCC master press until raw PEDALS confirms the arm is gone.
    can_sends = []
    if not self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON:
      return can_sends

    raw_armed = bool(CS.mrcc_armed_raw)
    # PEDALS drops both cruise bits through a brake transition; the filtered state bridges
    # those samples, and only sustained raw-off is authoritative.
    self.mrcc_raw_off_frames = 0 if raw_armed else self.mrcc_raw_off_frames + 1
    raw_off_confirmed = self.mrcc_raw_off_frames >= TJA_MRCC_RAW_OFF_CONFIRM_FRAMES
    # PEDALS holds both cruise bits low under braking: the clock restarts there and runs
    # again once the pedal is off.
    self.mrcc_arm_wait_frames = 0 if CS.out.brakePressed else self.mrcc_arm_wait_frames + 1
    mrcc_armed = raw_armed or (CS.cruise_available and not raw_off_confirmed)

    if CS.tja_button and not self.tja_button_prev:
      # A press before the previous press's arm reconciled sees that arm as its own
      # artifact, not the driver's baseline.
      if self.mrcc_undo_pending:
        self.mrcc_undo_frames = 0
      else:
        # PEDALS can already show the press-induced arm in the same cycle as the edge; the
        # previous stable sample is the state that existed before the press.
        armed_before_press = self.mrcc_armed_prev if self.mrcc_armed_prev is not None else mrcc_armed
        self.mrcc_undo_pending = not armed_before_press
      self.mrcc_undo_saw_armed = False
      self.mrcc_arm_wait_frames = 0
    self.tja_button_prev = bool(CS.tja_button)
    self.mrcc_armed_prev = mrcc_armed

    # The arm is fully reconciled; the budget returns for the next press.
    if not mrcc_armed and not CS.cruise_enabled and not CS.out.cruiseState.enabled:
      self.mrcc_undo_frames = 0

    if not self.mrcc_undo_pending:
      return can_sends

    self.mrcc_undo_saw_armed |= raw_armed

    # The driver's own cruise presses own CRZ_BTNS from this cycle on, and restoring
    # stock ECU ownership changes who owns the arm.
    driver_activity = (CS.cancel_button or CS.resume_button or CS.accel_button or CS.decel_button or
                       CS.mrcc_button or CS.distance_button)
    if driver_activity or handback_active or CC_SP.stockEcuHandBack:
      self.mrcc_undo_pending = False
      return can_sends

    # The arm never appeared: a takeover already disarmed it, or this car does not arm
    # through the press. Past the bounded brake-free wait, stand down.
    if not self.mrcc_undo_saw_armed:
      if self.mrcc_arm_wait_frames >= TJA_MRCC_ARM_WAIT_FRAMES:
        self.mrcc_undo_pending = False
      return can_sends

    if raw_off_confirmed:
      self.mrcc_undo_pending = False
      return can_sends

    # openpilot's own cancel or resume interleaves button frames; wait it out rather
    # than race the counter stream.
    if CC.cruiseControl.cancel or CC.cruiseControl.resume:
      return can_sends

    # Never inside the driver's held press.
    if CS.tja_button:
      return can_sends

    # One press per body-paced slot, within the episode budget; never while PEDALS reads disarmed,
    # where the master press would arm instead of disarm. last_button_frame also paces ICBM.
    if raw_armed and self.mrcc_undo_frames < TJA_MRCC_MAX_TX_FRAMES and \
       (self.frame - self.last_button_frame) * DT_CTRL > TJA_MRCC_TX_PERIOD:
      can_sends.append(mazdacan.create_mrcc_off_cmd(self.packer, CS.crz_btns_counter))
      self.last_button_frame = self.frame
      self.mrcc_undo_frames += 1
      if self.mrcc_undo_frames >= TJA_MRCC_MAX_TX_FRAMES:
        self.mrcc_undo_pending = False

    return can_sends

  def update_white_hud(self, CC, CC_SP, CS, handback_active: bool) -> bool:
    """Whether the alert frame carries the white wheel.

    On a TJA-declared car with MADS active and cruise fully off, the white-wheel TJA=2 state
    is XORed into the camera's current payload, but only onto an exact allowlisted idle base:
    TJA=2 is not display-only, the body reads the same frame. Every cruise interaction fails
    closed, and a white wheel HUD state that became unsafe is withdrawn immediately, outside
    the cadence.
    """
    session_ambiguous = handback_active or CC_SP.stockEcuHandBack
    mrcc_off = (not session_ambiguous and not CS.mrcc_armed_raw and
                not CS.cruise_available and not CS.cruise_enabled)

    # Every button a TJA wheel carries: TJA, MRCC, SET+/-, RES, DISTANCE, plus the
    # synthesized ICBM set presses and openpilot's own cancel/resume.
    button_activity = (CS.tja_button or CS.mrcc_button or CS.cancel_button or CS.resume_button or
                       CS.accel_button or CS.decel_button or CS.distance_button or
                       CC_SP.intelligentCruiseButtonManagement.sendButton in BUTTONS or
                       CC.cruiseControl.cancel or CC.cruiseControl.resume)

    white_allowed = (
      bool(self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON) and
      CC_SP.mads.active and
      CS.cam_laneinfo_live and
      mazdacan.white_hud_allowlist_base(CS.cam_laneinfo_raw) is not None and
      CC.hudControl.visualAlert == VisualAlert.none and
      not button_activity and
      mrcc_off
    )
    if white_allowed:
      self.mads_white_hud_off_frames = min(self.mads_white_hud_off_frames + 1, MADS_WHITE_HUD_OFF_CONFIRM_FRAMES)
    else:
      self.mads_white_hud_off_frames = 0
    return white_allowed and self.mads_white_hud_off_frames >= MADS_WHITE_HUD_OFF_CONFIRM_FRAMES

  def white_hud_frame(self, CS, alert, white: bool):
    """The alert frame as sent: the camera's own allowlisted idle base with TJA=2 when white."""
    fsc_raw = CS.cam_laneinfo_raw
    payload = mazdacan.white_hud_allowlist_base(fsc_raw) if white else alert[1]
    self.mads_white_hud_on_bus = white
    return alert[0], mazdacan.apply_mads_white_hud(fsc_raw, payload, white), alert[2]


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    # Unfiltered PEDALS cruise state; the filtered public state bridges brake dropouts.
    self.mrcc_armed_raw = False
    self.tja_button = 0
    self.prev_tja_button = 0
    # Active-low wheel MRCC master (CRZ_BTNS.BIT1), read from the bus-0 parser: a 0 is a press.
    self.mrcc_button = 0
    self.prev_mrcc_button = 0
    self.crz_btns_seen = False

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    # Both longitudinal modes: the TJA-press cleanup and the white-wheel HUD gate read it.
    self.mrcc_armed_raw = cp.vl["PEDALS"]["ACC_OFF"] == 1 or cp.vl["PEDALS"]["ACC_ACTIVE"] == 1

    self.prev_mrcc_button = self.mrcc_button
    self.prev_tja_button = self.tja_button
    btns = cp.vl["CRZ_BTNS"]
    # BIT1 is active-low: a 0 on the bus-0 parser is the wheel's MRCC master press. Gated on
    # the declaration (an undeclared wheel's idle level is unknown) and held unpressed until
    # the wheel's first frame: parser zeros before it would decode as a phantom press.
    if self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON:
      self.crz_btns_seen = self.crz_btns_seen or len(cp.vl_all["CRZ_BTNS"]["BIT1"]) > 0
      self.mrcc_button = int(btns["BIT1"] == 0) if self.crz_btns_seen else 0
    else:
      self.mrcc_button = 0
    # Only a car declared to have the physical TJA button reports it as the MADS switch.
    self.tja_button = int(btns["TJA_BUTTON"] == 1) if self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON else 0
