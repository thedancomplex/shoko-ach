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
#    for i in range(shoko.shoko.SHOKO_JOINT_COUNT):
#      print " enc for id: ", i," = ", state.joint[i].pos
    places = 3
    print "> RSP = ", round(state.joint[shoko.shoko.RSP].pos,places), "  \tRSY = ", round(state.joint[shoko.shoko.RSY].pos,places), "  \tREP = ", round(state.joint[shoko.shoko.REP].pos,places)
    print "> LSP = ", round(state.joint[shoko.shoko.LSP].pos,places), "  \tLSY = ", round(state.joint[shoko.shoko.LSY].pos,places), "  \tLEP = ", round(state.joint[shoko.shoko.LEP].pos,places)
    print "> RHP = ", round(state.joint[shoko.shoko.RHP].pos,places), "  \tRHY = ", round(state.joint[shoko.shoko.RHY].pos,places), "  \tRKP = ", round(state.joint[shoko.shoko.RKP].pos,places)
    print "> LHP = ", round(state.joint[shoko.shoko.LHP].pos,places), "  \tLHY = ", round(state.joint[shoko.shoko.LHY].pos,places), "  \tLKP = ", round(state.joint[shoko.shoko.LKP].pos,places)


    time.sleep(T)
  
