import shoko_ach as shoko
import numpy as np
import time


if __name__ == '__main__':
  T = 6.0
  val = 0.15
  while (True):
    # Get State
    shoko.getRefData()
    val = -val
    shoko.ref.joint[shoko.shoko.RSY].ref = val
    shoko.ref.joint[shoko.shoko.LSY].ref = val
    shoko.ref.joint[shoko.shoko.RHY].ref = val
    shoko.ref.joint[shoko.shoko.LHY].ref = val

    val_s = 0.0
    if( val < 0.0): val_s = -np.abs(val)
    else:  val_s = -0.1

    shoko.ref.joint[shoko.shoko.REP].ref = val_s
    shoko.ref.joint[shoko.shoko.LEP].ref = val_s
    shoko.ref.joint[shoko.shoko.RKP].ref = val_s
    shoko.ref.joint[shoko.shoko.LKP].ref = val_s

    shoko.ref.joint[shoko.shoko.RHP].ref = val_s
    shoko.ref.joint[shoko.shoko.LHP].ref = val_s
    shoko.ref.joint[shoko.shoko.RSP].ref = val_s
    shoko.ref.joint[shoko.shoko.LSP].ref = val_s

    shoko.setRefData()

    time.sleep(T)
  
