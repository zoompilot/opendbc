"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car actuation profile for Intelligent Cruise Button Management (ICBM).

On button-actuated (non-pcmCruiseSpeed) cars the stock ECU integrates the cruise buttons,
and every ECU does it differently: how fast discrete presses register and whether the ACC
delays deceleration while the set speed is still changing. The servo that plans button
moves reads these characteristics from here.

Cars without a measured profile get DEFAULT_PROFILE: discrete taps only, no hold;
the long-standing ICBM behavior. A brand only changes behavior by adding a measured entry.
Measurements behind the Mazda entry: docs/zoompilot/icbm.md.
"""
from dataclasses import dataclass

from opendbc.car import structs

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# the servo's hold sends are a stream of presses, valid wherever taps are; a brand without a
# native hold cadence acts on them as the tap of the same direction
TAP_EQUIVALENT = {
  SendButtonState.increaseHold: SendButtonState.increase,
  SendButtonState.decreaseHold: SendButtonState.decrease,
}


def tap_equivalent(send_button: SendButtonState) -> SendButtonState:
  return TAP_EQUIVALENT.get(send_button, send_button)


@dataclass(frozen=True)
class ICBMActuationProfile:
  # fastest tap cadence the body ECU reliably registers; beyond it presses are dropped and
  # faster is slower. Tap size is not modeled: the servo is closed-loop on the dash.
  tap_rate_hz: float = 5.

  # the stock ACC does not commit to decelerating while the set speed is still moving, so
  # the servo makes one decisive move and goes quiet instead of tracking continuously
  decel_needs_stable_setpoint: bool = False


# Mazda CX-5 2022: taps register at 5 Hz and move 1 mph; MRCC will not start decelerating
# until the set speed stops changing.
ICBM_ACTUATION_PROFILES: dict[str, ICBMActuationProfile] = {
  'mazda': ICBMActuationProfile(
    tap_rate_hz=5.,
    decel_needs_stable_setpoint=True,
  ),
}

DEFAULT_PROFILE = ICBMActuationProfile()


def get_actuation_profile(brand: str) -> ICBMActuationProfile:
  return ICBM_ACTUATION_PROFILES.get(brand, DEFAULT_PROFILE)
