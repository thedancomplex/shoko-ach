import shoko_ach as shoko
import numpy as np
shoko.init()


s_id = shoko.shoko.REP
s_ref = -np.pi / 2.0
shoko.setRef(s_id, s_ref)
print "servo id = ", shoko.param.joint[s_id].id
print "shoko index = ", s_id
shoko.syncRef()
