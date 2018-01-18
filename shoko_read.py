import shoko_ach as shoko
import numpy as np
import time
import os

if __name__ == '__main__':
  T = 0.1
  while (True):
    # Get State
    state = shoko.getState()
    ref = shoko.getRefData()
    os.system('clear')
#    for i in range(shoko.shoko.SHOKO_JOINT_COUNT):
#      print " enc for id: ", i," = ", state.joint[i].pos
    places = 3
    print "----[STATE]-----"
    print "> RSP = ", round(state.joint[shoko.shoko.RSP].pos,places), "   \tRSY = ", round(state.joint[shoko.shoko.RSY].pos,places), "      \tREP = ", round(state.joint[shoko.shoko.REP].pos,places)
    print "> LSP = ", round(state.joint[shoko.shoko.LSP].pos,places), "   \tLSY = ", round(state.joint[shoko.shoko.LSY].pos,places), "      \tLEP = ", round(state.joint[shoko.shoko.LEP].pos,places)
    print "> RHP = ", round(state.joint[shoko.shoko.RHP].pos,places), "   \tRHY = ", round(state.joint[shoko.shoko.RHY].pos,places), "      \tRKP = ", round(state.joint[shoko.shoko.RKP].pos,places)
    print "> LHP = ", round(state.joint[shoko.shoko.LHP].pos,places), "   \tLHY = ", round(state.joint[shoko.shoko.LHY].pos,places), "      \tLKP = ", round(state.joint[shoko.shoko.LKP].pos,places)
    print "-----[REF]-----"
    print "> RSP = ", round(ref.joint[shoko.shoko.RSP].ref,places), "   \tRSY = ", round(ref.joint[shoko.shoko.RSY].ref,places), "      \tREP = ", round(ref.joint[shoko.shoko.REP].ref,places)
    print "> LSP = ", round(ref.joint[shoko.shoko.LSP].ref,places), "   \tLSY = ", round(ref.joint[shoko.shoko.LSY].ref,places), "      \tLEP = ", round(ref.joint[shoko.shoko.LEP].ref,places)
    print "> RHP = ", round(ref.joint[shoko.shoko.RHP].ref,places), "   \tRHY = ", round(ref.joint[shoko.shoko.RHY].ref,places), "      \tRKP = ", round(ref.joint[shoko.shoko.RKP].ref,places)
    print "> LHP = ", round(ref.joint[shoko.shoko.LHP].ref,places), "   \tLHY = ", round(ref.joint[shoko.shoko.LHY].ref,places), "      \tLKP = ", round(ref.joint[shoko.shoko.LKP].ref,places)

    time.sleep(T)
  
