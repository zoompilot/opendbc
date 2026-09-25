"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

openpilot never presses the TJA button. On the car's side it toggles MADS and arms MRCC; on the
camera's side it switches the car's own lane-keep setting off (CAM_SETTINGS), after which the
EPS applies none of our request (route 7c735af5fce56485/00000105, 2026-09-12).
"""
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, car_controller, frames, mazda_car_state, step


def rig(alpha_long=False):
  cc = car_controller(alpha_long=alpha_long)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


class TestNoCameraPress:

  def test_no_button_frame_on_the_camera_bus(self):
    for alpha_long in (False, True):
      cc, cs = rig(alpha_long=alpha_long)
      for stock_tja in (0, 2, 3, 4):
        for lat_active in (False, True):
          for _ in range(150):
            _, sends = step(cc, cs, lat_active=lat_active, stock_tja=stock_tja, radar_was_silenced=alpha_long)
            assert not frames(sends, CRZ_BTNS, bus=2)
            for dat in frames(sends, CRZ_BTNS, bus=0):
              assert not dat[1] & 0x08, "the TJA bit must never go to the car"
