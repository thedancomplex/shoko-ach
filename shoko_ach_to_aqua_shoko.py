#import shoko_ach as shoko
#import aquashoko as aqua
import numpy as np

def ShokoToAquaRef(aqua,shoko):
  a = shoko.getRefData()
  aqua.aquashoko_set(0,0,a.joint[shoko.shoko.RSY].ref)
  aqua.aquashoko_set(0,1,a.joint[shoko.shoko.RSP].ref)
  aqua.aquashoko_set(0,2,a.joint[shoko.shoko.REP].ref)
 
  aqua.aquashoko_set(1,0,a.joint[shoko.shoko.LSY].ref)
  aqua.aquashoko_set(1,1,a.joint[shoko.shoko.LSP].ref)
  aqua.aquashoko_set(1,2,a.joint[shoko.shoko.LEP].ref)
 
  aqua.aquashoko_set(2,0,a.joint[shoko.shoko.RHY].ref)
  aqua.aquashoko_set(2,1,a.joint[shoko.shoko.RHP].ref)
  aqua.aquashoko_set(2,2,a.joint[shoko.shoko.RKP].ref)
 
  aqua.aquashoko_set(3,0,a.joint[shoko.shoko.LHY].ref)
  aqua.aquashoko_set(3,1,a.joint[shoko.shoko.LHP].ref)
  aqua.aquashoko_set(3,2,a.joint[shoko.shoko.LKP].ref)
  
  return aqua.aquashoko_put()

def AquaToShokoRef(aqua,shoko):
  shoko.getRefData()
  shoko.ref.joint[shoko.shoko.RSY].ref = aqua.aquashoko_get(0,0)
  shoko.ref.joint[shoko.shoko.RSP].ref = aqua.aquashoko_get(0,1)
  shoko.ref.joint[shoko.shoko.REP].ref = aqua.aquashoko_get(0,2)

  shoko.ref.joint[shoko.shoko.LSY].ref = aqua.aquashoko_get(1,0)
  shoko.ref.joint[shoko.shoko.LSP].ref = aqua.aquashoko_get(1,1)
  shoko.ref.joint[shoko.shoko.LEP].ref = aqua.aquashoko_get(1,2)

  shoko.ref.joint[shoko.shoko.RHY].ref = aqua.aquashoko_get(2,0)
  shoko.ref.joint[shoko.shoko.RHP].ref = aqua.aquashoko_get(2,1)
  shoko.ref.joint[shoko.shoko.RKP].ref = aqua.aquashoko_get(2,2)

  shoko.ref.joint[shoko.shoko.LHY].ref = aqua.aquashoko_get(3,0)
  shoko.ref.joint[shoko.shoko.LHP].ref = aqua.aquashoko_get(3,1)
  shoko.ref.joint[shoko.shoko.LKP].ref = aqua.aquashoko_get(3,2)
  
  print "shoko"
  for i in range(12):
    print shoko.ref.joint[i].ref
  print "aqua"
  for leg in range(4):
    for joint in range(3):
    	print aqua.aquashoko_get(leg,joint) 
  return shoko.setRefData()


def RosToShokoRef(ros,shoko):
  shoko.getRefData()

  shoko.ref.joint[shoko.shoko.RSY].ref = ros[0]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.RSP].ref = ros[1]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.REP].ref = ros[2]  * np.pi/180.0

  shoko.ref.joint[shoko.shoko.LSY].ref = ros[3]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.LSP].ref = ros[4]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.LEP].ref = ros[5]  * np.pi/180.0

  shoko.ref.joint[shoko.shoko.RHY].ref = ros[6]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.RHP].ref = ros[7]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.RKP].ref = ros[8]  * np.pi/180.0

  shoko.ref.joint[shoko.shoko.LHY].ref = ros[9]  * np.pi/180.0
  shoko.ref.joint[shoko.shoko.LHP].ref = ros[10] * np.pi/180.0
  shoko.ref.joint[shoko.shoko.LKP].ref = ros[11] * np.pi/180.0
  return shoko.setRefData()



  
