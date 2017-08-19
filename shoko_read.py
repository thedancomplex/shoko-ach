import shoko_ach as shoko
import numpy as np
import time
import os

if __name__ == '__main__':
  T = 0.1
  while (True):
    # Get State
    state = shoko.getState()
    os.system('clear')
    for i in range(shoko.shoko.SHOKO_JOINT_COUNT):
      print " enc for id: ", i," = ", state.joint[i].pos

    time.sleep(T)
  
