import shoko_ach as shoko
import numpy as np
import time



if __name__ == '__main__':
	state = shoko.getState()
	pos = np.zeros((4,3))
	#store current position of leg1 joints
	pos[0,0] = state.joint[shoko.shoko.RSY].pos
  	pos[0,1] = state.joint[shoko.shoko.RSP].pos
  	pos[0,2] = state.joint[shoko.shoko.REP].pos
  	
	#store current position of leg2 joints
	pos[1,0] = state.joint[shoko.shoko.LSY].pos
  	pos[1,1] = state.joint[shoko.shoko.LSP].pos
  	pos[1,2] = state.joint[shoko.shoko.LEP].pos

	#store current position of leg3 joints
	pos[2,0] = state.joint[shoko.shoko.LHY].pos
  	pos[2,1] = state.joint[shoko.shoko.LHP].pos
  	pos[2,2] = state.joint[shoko.shoko.LKP].pos

	#store current position of leg4 joints
	pos[3,0] = state.joint[shoko.shoko.RHY].pos
  	pos[3,1] = state.joint[shoko.shoko.RHP].pos
  	pos[3,2] = state.joint[shoko.shoko.RKP].pos
	
	#joint position at end 
  	goal = np.zeros((4,3))
  	goal[:,1] = 0.630
  	goal[:,2] = -0.546
  	
  	L = 10 #number of iterations to goal
  	
  	delta = np.zeros((4,3))
  	#calculate delta angle for each joint per iteration n
  	for n in range(4):
  		for m in range(3):
  			delta[n,m] = (goal[n,m] - pos[n,m])/float(L)

  	T = 0.01
	
	for x in range(0,n):
		for n in range(0,4):
			for m in range(0,3):
				pos[n,m] = pos[n,m] + delta[n,m]
	
	
		shoko.ref.joint[shoko.shoko.RSY].ref = pos[0,0]
		shoko.ref.joint[shoko.shoko.RSP].ref = pos[0,1]
		shoko.ref.joint[shoko.shoko.REP].ref = pos[0,2]
		
		shoko.ref.joint[shoko.shoko.LSY].ref = pos[1,0]
                shoko.ref.joint[shoko.shoko.LSP].ref = pos[1,1]
    	        shoko.ref.joint[shoko.shoko.LEP].ref = pos[1,2]
    	
    	        shoko.ref.joint[shoko.shoko.LHY].ref = pos[2,0]
    	        shoko.ref.joint[shoko.shoko.LHP].ref = pos[2,1]
    	        shoko.ref.joint[shoko.shoko.LKP].ref = pos[2,2]

    	        shoko.ref.joint[shoko.shoko.RHY].ref = pos[3,0]
    	        shoko.ref.joint[shoko.shoko.RHP].ref = pos[3,1]
    	        shoko.ref.joint[shoko.shoko.RKP].ref = pos[3,2]
    	
    	        shoko.setRefData()
    	        time.sleep(T)
    	 
    	#end for loop 
    	
  	shoko.ref.joint[shoko.shoko.RSY].ref = goal[0,0]
	shoko.ref.joint[shoko.shoko.RSP].ref = goal[0,1]
	shoko.ref.joint[shoko.shoko.REP].ref = goal[0,2]
		
	shoko.ref.joint[shoko.shoko.LSY].ref = goal[1,0]
	shoko.ref.joint[shoko.shoko.LSP].ref = goal[1,1]
        shoko.ref.joint[shoko.shoko.LEP].ref = goal[1,2]
    	
        shoko.ref.joint[shoko.shoko.LHY].ref = goal[2,0]
        shoko.ref.joint[shoko.shoko.LHP].ref = goal[2,1]
        shoko.ref.joint[shoko.shoko.LKP].ref = goal[2,2]

        shoko.ref.joint[shoko.shoko.RHY].ref = goal[3,0]
        shoko.ref.joint[shoko.shoko.RHP].ref = goal[3,1]
        shoko.ref.joint[shoko.shoko.RKP].ref = goal[3,2]

	shoko.setRefData()  		
  	
  	
