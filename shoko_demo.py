import shoko_ach as shoko
import numpy as np
import time


if __name__ == '__main__':
  T = 0.01
  while (True):
    # Get State
    shoko.getRefData()
    state = shoko.getState()

    # set all xSP
    xSP = state.joint[shoko.shoko.RSP].pos
    shoko.ref.joint[shoko.shoko.RSP].mode = shoko.shoko.SHOKO_REF_MODE_NO_TORQUE
    shoko.ref.joint[shoko.shoko.RSY].mode = shoko.shoko.SHOKO_REF_MODE_NO_TORQUE
    shoko.ref.joint[shoko.shoko.REP].mode = shoko.shoko.SHOKO_REF_MODE_NO_TORQUE

    shoko.ref.joint[shoko.shoko.RSP].ref = xSP
    shoko.ref.joint[shoko.shoko.LSP].ref = xSP
    shoko.ref.joint[shoko.shoko.LHP].ref = xSP
    shoko.ref.joint[shoko.shoko.RHP].ref = xSP


    # set all xSY
    xSY = state.joint[shoko.shoko.RSY].pos

    shoko.ref.joint[shoko.shoko.RSY].ref = xSY
    shoko.ref.joint[shoko.shoko.LSY].ref = xSY
    shoko.ref.joint[shoko.shoko.LHY].ref = xSY
    shoko.ref.joint[shoko.shoko.RHY].ref = xSY


    # set all xEP
    xEP = state.joint[shoko.shoko.REP].pos

    shoko.ref.joint[shoko.shoko.REP].ref = xEP
    shoko.ref.joint[shoko.shoko.LEP].ref = xEP
    shoko.ref.joint[shoko.shoko.LKP].ref = xEP
    shoko.ref.joint[shoko.shoko.RKP].ref = xEP

    shoko.setRefData()

    time.sleep(T)
  
