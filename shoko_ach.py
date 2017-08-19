import shoko_ach_h as shoko
import dynamixel
import sys
import numpy as np
import time


ref     = shoko.SHOKO_REF()
state   = shoko.SHOKO_STATE()
param   = shoko.SHOKO_PARAM() 

robot   = None


def init(com = None, baud=None):
  global ref, state, param
  if(com == None):
    com = '/dev/ttyUSB0'

  if(baud == None):
    baud = 1000000

  # setting up setting (but not using a setup file)
  for i in range(shoko.SHOKO_JOINT_COUNT):
    param.joint[i].ticks     = 4096
    param.joint[i].offset    = 0.0
    param.joint[i].dir       = 1.0
    param.joint[i].torque    = 0.0045  # 4.5 mA per unit
    param.joint[i].theta_max =  np.PI/2.0 # max theta in rad
    param.joint[i].theta_min = -np.PI/2.0 # min theta in rad
    param.baud               = baud # baud rate
    param.com                = com  # com port     

  param.joint[shoko.RSY].id = 11
  param.joint[shoko.RSP].id = 12
  param.joint[shoko.REP].id = 13
  
  param.joint[shoko.LSY].id = 21
  param.joint[shoko.LSP].id = 22
  param.joint[shoko.LEP].id = 23

  param.joint[shoko.RHY].id = 41
  param.joint[shoko.RHP].id = 42
  param.joint[shoko.RKP].id = 43


  param.joint[shoko.LHY].id = 31
  param.joint[shoko.LHP].id = 32
  param.joint[shoko.LKP].id = 33

  dynSetup()

def getMaxID(p):
  r = 0
  for i in range(shoko.SHOKO_JOINT_COUNT):
    pp = p.joint[i].id
    if(pp >= r):  r = pp
  return r

def dynSetup():
    global param
# Look for a settings.yaml file
    settingsFile = 'settings.yaml'
    if not options.clean and os.path.exists(settingsFile):
        with open(settingsFile, 'r') as fh:
            settings = yaml.load(fh)
    # If we were asked to bypass, or don't have settings
    else:
        settings = {}
        settings['port'] = param.com
        
        # Baud rate
        baudRate = param.baud
        settings['baudRate'] = baudRate
        
        # Servo ID
        highestServoId = getMaxID(param)
        
        settings['highestServoId'] = highestServoId

        highestServoId = settings['highestServoId']

        # Establish a serial connection to the dynamixel network.
        # This usually requires a USB2Dynamixel
        serial = dynamixel.SerialStream(port=settings['port'],
                                        baudrate=settings['baudRate'],
                                        timeout=1)
        # Instantiate our network object
        net = dynamixel.DynamixelNetwork(serial)
        
        # Ping the range of servos that are attached
        print "Scanning for Dynamixels..."
        net.scan(1, highestServoId)

        settings['servoIds'] = []
        print "Found the following Dynamixels IDs: "
        for dyn in net.get_dynamixels():
            print dyn.id
            settings['servoIds'].append(dyn.id)

        # Make sure we actually found servos
        if not settings['servoIds']:
          print 'No Dynamixels Found!'
          sys.exit(0)

        # Save the output settings to a yaml file
        with open(settingsFile, 'w') as fh:
            yaml.dump(settings, fh)
            print("Your settings have been saved to 'settings.yaml'. \nTo " +
                   "change them in the future either edit that file or run " +
                   "this example with -c.")
    
    mainSetup(settings)

def mainSetup(settings):
    global robot
    # Establish a serial connection to the dynamixel network.
    # This usually requires a USB2Dynamixel
    serial = dynamixel.SerialStream(port=settings['port'], baudrate=settings['baudRate'], timeout=1)
    # Instantiate our network object
    net = dynamixel.DynamixelNetwork(serial)

    # Populate our network with dynamixel objects
    for servoId in settings['servoIds']:
        newDynamixel = dynamixel.Dynamixel(servoId, net)
        net._dynamixel_map[servoId] = newDynamixel
    
    if not net.get_dynamixels():
      print 'No Dynamixels Found!'
      sys.exit(0)
    else:
      print "...Done"
    
    robot = net
