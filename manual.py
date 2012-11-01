from joy import *
import sys
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical 

#ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfa131"
# Uncomment above line when using wireless transmitor


# TODO: ruoran or xiangyu: change all print into progress

class NoEnd ( Plan ):
    def __init__(self, app):
        Plan.__init__(self, app)

    def behavior(self):
        while True:
            progress("Hello Printing")
            yield self.forDuration(1)


class Rotate( Plan ):
    """ RotatePlan will handle the axis servo movement 
        Rorating axis servo is to adjust the platform (laser) 
        pointing direction, this happens when wheels are locked.
    """
    def __init__(self, app, direction=1, unit=220, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = direction
        self.unit = unit

    def onStart(self):
        if not self.app.testing:
            self.app.cur_axis_pos = self.app.robot.at.axis.get_pos()
        
    def behavior(self):
        while True:
            self.app.cur_axis_pos += self.direction * self.unit
            if self.app.testing:
                progress("Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
            else:
                progress("Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
                self.app.robot.at.axis.set_pos(self.app.cur_axis_pos)
            yield self.forDuration(0.05)

class Move( Plan ):
    """ Move is a basic plan that will be called whenever we want to
        move forward or backward for a certain amount of distance.

        attr:
            direction = 1 or -1, when 1 moving forward
        speed:

    """

    def __init__(self, app, direction=1, speed=0.2, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = direction
        self.speed = speed

    def change_direction(self):
        self.direction = -1 if self.direction == 1 else 1

    def behavior(self):
        while True:
            if self.app.testing:
                print "Move -- Direction %s, torque %s" % (self.direction, self.speed) 
            else:
                print "Move -- Direction %s, torque %s" % (self.direction, self.speed) 
                self.app.robot.at.left.set_torque(self.direction * self.speed)
                self.app.robot.at.right.set_torque(self.direction * -1 * self.speed)
            yield self.forDuration(0.1)

    def onStop(self):
        if not self.app.testing:
            self.app.robot.at.left.set_torque(0)
            self.app.robot.at.right.set_torque(0)
        

class Turn( Plan ):
    """ Turn will make two wleels spin at opposite direction, thus 
        turning the wheels. This need to make sure platform (laser)
        stays put.
    """
    def __init__(self, app, direction=1, speed=0.1, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = direction
        self.speed = speed

    def behavior(self):
        while True:
            if self.app.testing:
                print "Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) 
            else:
                print "Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) 
                self.app.robot.at.axis.go_slack()
                self.app.robot.at.left.set_torque(self.direction * self.speed)
                self.app.robot.at.right.set_torque(self.direction * self.speed)
            yield self.forDuration(0.3)

    def onStop(self):
        if not self.app.testing:
            progress("stopped turning")
            self.app.robot.at.left.set_torque(0)
            self.app.robot.at.right.set_torque(0)
    

class DrivingApp( JoyApp ):
    """ unit_speed is for the two wheels """
    def __init__(self,spec,unit_speed = 0.2, testing=False,*arg,**kw):
        if testing:
            JoyApp.__init__(self, *arg,**kw)
        else:
            progress("populating 3 robots!!!!")
            JoyApp.__init__(self, robot = {'count': 3},  *arg,**kw)
        
        self.testing = testing
        self.left_speed = unit_speed
        self.right_speed = -1 * unit_speed
        self.spec = spec

    def onStart(self):
        self.output = self.setterOf(self.spec)
        self.auto = 0
        self.state = None
        self.cur_axis_pos = -1042 # this is 'zero' 
        self.unit_turn = 200
        
        # Plan initialization
        self.move_plan = Move(self)
        self.turn_plan = Turn(self)
        self.rotate_plan = Rotate(self)

        # Set the module mode
        if not self.testing:
            self.robot.at.axis.set_mode(0) # Servo
            self.robot.at.right.set_mode(1) # Motor  
            self.robot.at.left.set_mode(1) # Motor

    def onEvent(self,evt):

        # First level events 
        # * Auto mode
        # * Manual mode
        # * Exit
        if evt.type == KEYDOWN and evt.key == K_n: #up
            self.auto = 1
            print "Entering auto"
    
        if evt.type == KEYDOWN and evt.key == K_m: #up
            self.auto = 0
            for plan in self.plans:
                plan.stop()
            print "Entering manual"

        # Exit
        if evt.type == KEYDOWN and evt.key in [ K_ESCAPE ]: # Esc 
            print "Exiting!"
            
            # Close socket connections
            # stop all motors if not in testing mode
            sys.exit(1)   

        if evt.type == KEYDOWN and evt.key in [ K_p, K_o, K_DELETE ]: # stop
            for i in range(4):
                self.robot.off()

        # Detail of Audo mode and Manual Mode
        if not self.auto:
            # Manual Mode
            if evt.type == KEYDOWN and evt.key == K_w: #up
                self.state = evt.key
                self.move_plan.direction = 1
                if not self.move_plan.isRunning():
                    self.move_plan.start()

            elif evt.type == KEYDOWN and evt.key == K_s: #down
                self.state = evt.key
                self.move_plan.direction = -1
                if not self.move_plan.isRunning():
                    self.move_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_a: # rotate left
                self.state = evt.key
                #progress(str(self.cur_axis_pos))
                self.rotate_plan.direction = 1
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_d: # rotate right
                self.state = evt.key
                self.rotate_plan.direction = -1
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_q: # Turn
                self.state = evt.key
                self.turn_plan.direction = 1
                if not self.turn_plan.isRunning():
                    self.turn_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_e: # Turn
                self.state = evt.key
                self.turn_plan.direction = -1
                if not self.turn_plan.isRunning():
                    self.turn_plan.start()
  
  
            # KEYUPs
            elif evt.type == KEYUP and evt.key in [K_w, K_s] : # stop moving
                self.move_plan.stop()

            elif evt.type == KEYUP and evt.key in [K_a, K_d] : # stop rotating
                self.rotate_plan.stop()

            elif evt.type == KEYUP and evt.key in [K_q, K_e] : # stop turning
                self.turn_plan.stop()


        else: # This is auto mode block
            # Auto Mode
            # self.auto_pan.start()
            print "in auto mode, press 'm' to switch to manual mode"
            self.no_end_plan = NoEnd(self)
            self.no_end_plan.start()

    def onStop(self):
        for plan in self.plans:
            plan.stop()

        self.move_plan.stop()
        self.rotate_plan.stop()
        self.turn_plan.stop()
        self.no_end_plan.stop()
        if not self.testing:
            for i in range(4):
                self.robot.off()
        progress("The application have been stopped.")
        return super( DrivingApp, self).onStop()
        


### Sensor Plan 
class SensorPlan( Plan ):
  def __init__( self, app, peer, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.peer = peer
 
  def _connect( self ):
    s = socket(AF_INET, SOCK_STREAM)
    try:
       s.connect( self.peer )
    except SocketError, se:
       progress("Failed to connect: "+str(se))
       return
    s.setblocking(0)
    self.sock = s

  def stop( self ):
    if self.sock is not None:
      self.sock.close()
    self.sock = None

  def behavior( self ):
    while True:
      # if not connected --> try to connect
      if self.sock is None:
        self._connect()
      # if not connected --> sleep for a bit
      if self.sock is None:
        yield self.forDuration(0.1)
        continue
      # receive an update / skip
      try:
        msg = self.sock.recv(1024)
      except SocketError, se:
        # If there was no data on the socket --> not a real error, else
        if se.errno != 35:
          progress("Connection failed: "+str(se))
          self.sock.close()
          self.sock = None
        yield
        continue
      ts = self.app.now
      dic = json_loads(msg)
      assert type(dic) is dict
      dic = dic.items()
      dic.sort()
      progress("Message received at: " + str(ts))
      for k,v in dic:
        progress("   %s : %s" % (k,repr(v)))
      yield self.forDuration(0.3)

class WaypointSensorApp( JoyApp ):
  def onStart( self ):
    # Set up the sensor receiver plan
    self.sensor = SensorPlan(self,("67.194.202.70",8080))
    self.sensor.start()
    
  def onEvent( self, evt ):
    # Punt to superclass
    # this is here to remind you to override it
    return super( WaypointSensorApp, self ).onEvent(evt)
  
  def onStop( self ):
    self.sensor.stop()
    return super( WaypointSensorApp, self ).onStop()

def main():

    # ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfa131"
    # uncomment this at line 7 

    if len(sys.argv) > 1 and sys.argv[1] == "-t":
        flag = True
    else:
        flag = False
        
    app = DrivingApp("#output ", testing=flag)
    app.run()

if __name__ == '__main__':
    main()
