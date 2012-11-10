from joy import *
import sys
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical 
import time
import math
from collections import deque
from numpy import arctan2, mean

#ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfd111"

# Uncomment above line when using wireless transmitor
# Important: Axis module's torque_limit need to be set to 50

def get_phi(cur_point, next_point):
    """ return the number for set_pos to use """
    from numpy import arctan2
    theta = arctan2(float(next_point[1] - cur_point[1]), (next_point[0] - cur_point[0]) )
    direction = None
    if theta < 0:
        theta = theta + math.pi
        direction = -1
    else:
        direction = 1
    phi = theta - math.pi / 2
    pos = phi * 180/math.pi * 100
    return pos, direction

class AutoPlan ( Plan ):
    def __init__(self, app):
        Plan.__init__(self, app)
        self.cur_point = None
        self.next_point = None
        self.wp_list = []              # dic['w']
        self.standard_f = None
        self.standard_b = None
        self.theta = None
        self.nwp = False
        self.count = 0

        # Two sub-plans belongs to AutoPlan
        self.move_plan = Move(self.app, direction=1)
        self.turn2_plan = Turn2(self.app)

    def behavior(self):
        while True:
            # If found first waypoint 
            if not self.next_point and self.app.latest_w:
                self.app.move_plan.stop()
                self.cur_point = self.app.latest_w[0]   
                self.next_point = self.app.latest_w[1] 
                progress(" Initializing waypoints! Cur: %s -- Next: %s" %\
                        (self.cur_point, self.next_point) )
                phi, direction = get_phi(self.cur_point, self.next_point) 
                self.app.direction = direction 
                self.phi = phi
                # action !!!
                self.turn2_plan.start(goal=phi)
                self.wp_list = self.app.latest_w
                #self.standard_f = mean([item['f'] for item in queue[:10]])
                #self.standard_b = mean([item['b'] for item in queue[:10]])

            # If found next waypoint
            if len(self.app.latest_w) != len(self.wp_list):
                self.cur_point = self.app.latest_w[0]   
                self.next_point = self.app.latest_w[1] 
                progress("Setting new waypoints! Cur: %s -- Next: %s" %\
                        (self.cur_point, self.next_point) )
                phi, direction = get_phi(self.cur_point, self.next_point)
                self.app.direction = direction     
                self.phi = phi           
                # action !!!
                self.move_plan.stop(force=True)
                self.turn2_plan.start(goal=phi)
                self.wp_list = self.app.latest_w
                self.nwp = True
                self.count = 0
                #self.standard_f = mean([item['f'] for item in queue[:10]])
                #self.standard_b = mean([item['b'] for item in queue[:10]])

            if not self.move_plan.isRunning() and len(self.wp_list)>1\
                    and not self.turn2_plan.isRunning():
                if not self.nwp and self.app.queue[0]['f'] <=5 and\
                        self.app.queue[0]['b'] <=5 and self.count >= 8:
                    progress( "Calibrating!" ) 
                    self.app.direction = -1 * self.app.direction
                    self.move_plan.direction = self.app.direction
                    while self.app.queue[0]['f'] <= 20 and self.app.queue[0]['b'] <= 20:
                        yield self.forDuration(1)
                        if self.app.queue[0]['f'] <= 20 and self.app.queue[0]['b'] <= 20:
                            self.move_plan.start(duration=0.7)
                        progress("backward.")
                        yield self.forDuration(1.5) 
                    self.move_plan.start(duration = 0.5)
                    yield self.forDuration(1.5)
                    progress("Turn! f: %s  b: %s" %\
                            (self.app.queue[0]['f'], self.app.queue[0]['b']) )

                    # Get close to the line 
                    if self.phi >= 0:
                        self.phi = self.phi - 9000
                    else:
                        self.phi = self.phi + 9000
                    
                    self.turn2_plan.start(goal=self.phi)
                    yield self.forDuration(10)
                    for i in range(10):
                        progress("Foward, Backward! %s" % str(i) )
                        if len(self.app.latest_w) != len(self.wp_list):
                            progress("Got it!")
                            break
                        self.move_plan.direction = self.app.direction
                        self.app.direction = -1 * self.app.direction
                        self.move_plan.start(duration=i+1)
                        yield self.forDuration(8 + i)
                    self.nwp = True
                    self.count = 0
                    # 90 go and 
                #    
                else:
                    ###
                    #list_f = get_latest_f()   # [1,2,3,4,5]
                    #list_b = get_latest_b()
                    #list_af = get_latest_af() 
                    #list_ab = get_latest_ab()

                    #if abs(list_af[0]-100) < 45 and abs(list_ab[0]-100) < 45:
                    #    # threshold = 45 
                    #    if list_af[0] > list_ab[0]:
                    #        turn_plan.start(goal=self.phi+100)
                    #    else:
                    #        turn_plan.start(goal=self.phi-100)
                    ###

                    self.move_plan.direction = self.app.direction
                    self.nwp = False
                    self.move_plan.start(duration=1)
                    self.count += 1

            yield self.forDuration(0.4) # change to 0.3 if necessary

class Rotate( Plan ):
    """ RotatePlan will handle the axis servo movement 
        Rorating axis servo is to adjust the platform (laser) 
        pointing direction, this happens when wheels are locked.

        direction = -1    Rotate left
        direction = 1     Rotate right 
    """
    def __init__(self, app, direction=1, unit=10, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = None 
        self.unit = unit
        self.lock_wheels = False

    def onStart(self):
        if not self.app.testing:
            self.app.cur_axis_pos = self.app.robot.at.axis.get_pos()

    def start(self, exe_time = 10000):
        self.exe_time = exe_time 
        if exe_time != 10000:
            progress("Started rotating plan with count = %d" % self.exe_time)
        Plan.start(self)
        
    def behavior(self):
        for count in range(self.exe_time):
            #progress("count %d" % count)
            if count >= self.exe_time -1:
                #progress("Count reached, Stop!")
                self.stop()
                break
            #self.app.cur_axis_pos += self.direction * self.unit
            if self.app.testing:
                progress("Rotate -- Direction %s pos %s" % (self.direction, self.app.cur_axis_pos))
            else:
                for i in range(20):
                    for i in range(10000):
                        aaa = 13123/ 123123
                    self.app.cur_axis_pos += self.direction * self.unit
                    self.app.robot.at.axis.set_pos(self.app.cur_axis_pos)

            yield 

    def onStop(self):
        Plan.onStop(self)

class Move( Plan ):
    """ Move is a basic plan that will be called whenever we want to
        move forward or backward for a certain amount of distance.

        attr:
            direction = 1 or -1, when 1 moving forward
        speed:
            set_torque value,  from -1 to 1
    """

    def __init__(self, app, direction=1, speed=0.2, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = self.app.direction 
        self.speed = speed
        self.duration = None 

    def change_direction(self):
        self.app.direction = -1 if self.app.direction == 1 else 1

    def start(self, exe_time = 10000, duration = None):
        self.duration = duration
   #     if self.duration:
   #         progress("starting turn plan with duration %s" % duration)
        self.start_time = self.app.now
        self.exe_time = exe_time 
        if self.exe_time != 10000:
            progress("started move plan with count = %d" % self.exe_time)
        Plan.start(self)

    def behavior(self):
        for count in range(self.exe_time):
            #progress("count %d" % count)
            if count >= self.exe_time -1:
                #progress("Count reached, Stop!")
                self.stop()
                break

            if self.duration:
                #progress(" %s %s %s" % (self.app.now, self.start_time, self.duration))
                if self.app.now - self.start_time > self.duration:
                    #progress("stop move because times up")
                    self.stop()
                    break

            if self.app.testing:
                progress("Move -- Direction %s, torque %s" % (self.direction, self.speed) )
            else:
      #          progress( "Unit Move -- Direction %s, torque %s" % (self.direction, self.speed) )
                self.app.robot.at.axis.set_pos(self.app.robot.at.axis.get_pos())
                self.app.robot.at.left.set_torque(self.direction * self.speed)
                self.app.robot.at.right.set_torque(self.direction * -1 * self.speed)
            yield self.forDuration(0.1)

    def onStop(self):
        self.app.robot.at.axis.go_slack()
        self.app.robot.at.axis.go_slack() 
        self.app.robot.at.left.set_torque(0)
        self.app.robot.at.right.set_torque(0)
        
class Turn( Plan ):
    """ Turn will make two wleels spin at opposite direction, thus 
        turning the wheels. This need to make sure platform (laser)
        stays put.
    """
    def __init__(self, app, direction=1, speed=0.105, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.direction = self.app.direction * -1
        self.speed = speed
        self.duration = None 

    def start(self, exe_time = 10000, duration = None):
        self.exe_time = exe_time 
        self.duration = duration
        if duration:
            progress("starting turn plan with duration %s" % duration)
        self.start_time = self.app.now
        if exe_time != 10000:
            progress("started rotating plan with count = %d" % self.exe_time)
        Plan.start(self)

    def behavior(self):
        for count in range(self.exe_time):
            if count >= self.exe_time -1:
                #progress("Count reached, Stop!")
                self.stop()
                break

            if self.duration:
                #progress(" %s %s %s" % (self.app.now, self.start_time, self.duration))
                if self.app.now - self.start_time > self.duration:
                    #progress("stop because times up")
                    self.stop()
                    break

            if self.app.testing:
                progress("Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) )
            else:
            #    self.app.robot.at.axis.go_slack()
                self.app.robot.at.left.set_torque(self.direction * self.speed)
                self.app.robot.at.right.set_torque(self.direction * self.speed)
                self.app.cur_axis_pos = self.app.robot.at.axis.get_pos()
            yield self.forDuration(0.3)

    def onStop(self):
        #progress("stopped turning")
        if not self.app.testing:
            self.app.robot.at.left.set_torque(0)
            self.app.robot.at.right.set_torque(0)
        Plan.onStop(self)

class Turn2( Plan ):
    def __init__(self, app, *arg, **kw):
        Plan.__init__(self, app, *arg, **kw)
        self.goal_pos = None         
        self.turn_plan = Turn(self.app, direction=1)
        self.rotate_plan = Rotate(self.app, direction=1)

    def start(self, goal = None):
        self.goal_pos = goal
        self.start_pos = self.app.robot.at.axis.get_pos() 
        Plan.start(self)

    def behavior(self):
        while True:
            if self.start_pos < self.goal_pos:
                #self.rotate_plan.direction = 1
                self.turn_plan.direction = -1
                #self.rotate_plan.start()
                self.turn_plan.start()
                self.app.robot.at.axis.set_pos(self.goal_pos)
                while self.app.robot.at.axis.get_pos() < self.goal_pos:
                    pass
      #              progress("turning left")
                    if abs(self.app.robot.at.axis.get_pos() - self.goal_pos) < 100:
                        self.turn_plan.stop()
                        self.stop()
                        break
                    yield self.forDuration(0.1)
                #self.rotate_plan.stop()
                self.turn_plan.stop()
                self.stop()
                break

            else: #start_pos > goal_pos
                #self.rotate_plan.direction = -1
                self.turn_plan.direction = 1
                #self.rotate_plan.start()
                self.turn_plan.start()
                self.app.robot.at.axis.set_pos(self.goal_pos)
                while self.app.robot.at.axis.get_pos() > self.goal_pos:
                    pass 
         #           progress("turning right")
                    if abs(self.app.robot.at.axis.get_pos() - self.goal_pos) < 100:
                        self.turn_plan.stop()
                        self.stop()
                        break
                    yield self.forDuration(0.1)
                #self.rotate_plan.stop()
                self.turn_plan.stop()
                self.stop()
                break

#    def onStop(self):
#        self.turn_plan.stop()
#        self.rotate_plan.stop()
    

class DrivingApp( JoyApp ):
    """ unit_speed is for the two wheels """
    def __init__(self,spec,unit_speed = 0.2, testing=False, no_sensor=False,*arg,**kw):
        if testing:
            JoyApp.__init__(self, *arg,**kw)
        else:
            progress("populating 3 robots!!!!")
            JoyApp.__init__(self, robot = {'count': 3},  *arg,**kw)
        
        self.testing = testing
        self.spec = spec
        self.direction = 1
        self.queue = [] 
        self.no_sensor = no_sensor
        self.latest_w = None 

        self.turn_plan = Turn(self, direction=1)
        self.rotate_plan = Rotate(self, direction=1)
        self.move_plan = Move(self, direction=1)
        self.auto_plan = AutoPlan(self)
        m = self.robot.at.axis
        m.mem[m.mcu.torque_limit] = 50

        self.sensor = SensorPlan(self,("67.194.202.70",8080))
        
        self.turn2_plan = Turn2(self)

    def onStart(self):
        self.output = self.setterOf(self.spec)
        self.auto = 0
        self.state = None
        self.cur_axis_pos = -122 # this is 'zero' 
        self.unit_turn = 200
        
        # Plan initialization
        if not self.no_sensor:
            self.sensor.start()

        # Set the module mode
        if not self.testing:
            self.robot.at.axis.set_mode(0) # Servo
            self.robot.at.right.set_mode(1) # Motor  
            self.robot.at.left.set_mode(1) # Motor
            #if raw_input("Reset axis? (y/n)") == 'y':
            #    self.robot.at.axis.set_pos(self.cur_axis_pos)

    def onEvent(self,evt):
        # First level events 
        # * Auto mode
        # * Manual mode
        # * Exit

        # Exit
        if evt.type == KEYDOWN and evt.key in [ K_ESCAPE ]: # Esc 
            progress("Exiting!")
            
            # Close socket connections
            # stop all motors if not in testing mode
            self.stop()

        if evt.type == KEYDOWN and evt.key == K_n: #
            progress( "Entering auto")
            self.auto_plan.start()
    
        if evt.type == KEYDOWN and evt.key == K_m: #up
            self.auto_plan.stop()
            progress( "Entering manual")


        if evt.type == KEYDOWN and evt.key in [ K_p, K_DELETE ]: # stop
            for i in range(4):
                self.robot.off()

        # Detail of Audo mode and Manual Mode
        if not self.auto:
 #           print "manual"
            # Manual Mode
            if evt.type == KEYDOWN and evt.key ==  K_u:  # Turn left
                a = int(raw_input("number -9000 to 9000: "))
                self.turn2_plan.start(goal=a)

            if evt.type == KEYDOWN and evt.key ==  K_o:  # Turn left
                progress("start axis pos %s" % self.robot.at.axis.get_pos())
                progress("pressed K_o")
                self.rotate_plan.direction = 1
                self.turn_plan.direction = -1
                self.turn_plan.start(duration = 1.6)
                self.rotate_plan.start(exe_time = 18)

            if evt.type == KEYDOWN and evt.key ==  K_i:  # Turn right 
                progress("start axis pos %s" % self.robot.at.axis.get_pos())
                progress("pressed K_i")
                self.rotate_plan.direction = -1
                self.turn_plan.direction = 1
                self.turn_plan.start(duration = 1.6)
                self.rotate_plan.start(exe_time = 18)

            if evt.type == KEYDOWN and evt.key ==  K_l: 
                progress("End %s" % self.robot.at.axis.get_pos())

            if evt.type == KEYDOWN and evt.key == K_z: #up
                progress("Current direction %s, changing direction %s" % (self.direction, self.direction*-1))
                self.move_plan.change_direction()

            if evt.type == KEYDOWN and evt.key == K_w: #up
                self.move_plan.direction = self.direction
                if not self.move_plan.isRunning():
                    self.move_plan.start()

            elif evt.type == KEYDOWN and evt.key == K_s: #down
                self.move_plan.direction = -1 * self.direction
                if not self.move_plan.isRunning():
                    self.move_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_a: # rotate left
                self.rotate_plan.direction = -1 
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_d: # rotate right
                self.rotate_plan.direction = 1 
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_LEFT: #lock wheels rotate left
                self.rotate_plan.direction = -1 
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_RIGHT: #lock wheels rotate right
                self.rotate_plan.direction = 1 
                if not self.rotate_plan.isRunning():
                    self.rotate_plan.start()

            elif evt.type == KEYDOWN and evt.key == K_q: # Turn
                self.turn_plan.direction = -1 
                if not self.turn_plan.isRunning():
                    self.turn_plan.start()
  
            elif evt.type == KEYDOWN and evt.key == K_e: # Turn
                self.turn_plan.direction = 1 
                if not self.turn_plan.isRunning():
                    self.turn_plan.start()
  
            # KEYUPs
            elif evt.type == KEYUP and evt.key in [K_y] : # stop moving
                self.move_plan.stop()
                self.turn_plan.stop()

            #elif evt.type == KEYUP and evt.key in [K_u] : # stop moving
            #    self.turn_plan.stop()

            elif evt.type == KEYUP and evt.key in [K_w, K_s] : # stop moving
                self.move_plan.stop()
                progress("move keyup event!")

            elif evt.type == KEYUP and evt.key in [K_a, K_d, K_LEFT, K_RIGHT] : # stop rotating
                self.rotate_plan.stop()
                progress("rotating keyup event!")

            elif evt.type == KEYUP and evt.key in [K_q, K_e] : # stop turning
                self.turn_plan.stop()
                progress("turning keyup event!")

    def onStop(self):
        self.move_plan.stop()
        for i in range(4):
            self.robot.off()
        progress("The application have been stopped.")
        return super( DrivingApp, self).onStop()



class SensorPlan( Plan ):
  """
  SensorPlan is a concrete Plan subclass that uses the self.app's
  remote plan to read and decode WayPoint Task sensor and waypoint
  updates.
  """
  def __init__( self, app, peer, *arg, **kw ):
    Plan.__init__(self, app, *arg, **kw )
    self.sock = None
    self.peer = peer
    self.lastSensorReading = None
 
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
        if se.errno != 11:
          progress("Connection failed: "+str(se))
          self.sock.close()
          self.sock = None
        yield
        continue
      ts = self.app.now
      dic = json_loads(msg)
      self.lastSensorReading = (ts, dic['f'], dic['b'])
      #assert type(dic) is dict
      #dic = dic.items()
      #dic.sort()
      #progress("Message received at: " + str(ts))
      #for k,v in dic:
      #  progress("   %s : %s" % (k,repr(v)))

      if len(self.app.queue) > 1024:
          self.app.queue.pop()
     # print "fb"
      self.app.queue.insert(0, dic) # store in queue
      if 'w' in dic:
          self.app.latest_w = dic['w']

      dic = dic.items()
      dic.sort()
      progress("Message received at: " + str(ts))
      for k,v in dic:
        progress("   %s : %s" % (k,repr(v)))

      yield self.forDuration(0.3)



def main():

    # ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfa131"
    # uncomment this at line 7 

    if len(sys.argv) > 1 and sys.argv[1] == "-t":
        flag = True
        
    elif len(sys.argv) > 1 and sys.argv[1] == "-n":
        no_sensor = True
        flag = False
    else:
        flag = False
        no_sensor = False
        
    app = DrivingApp("#output ", testing=flag, no_sensor=no_sensor)
    app.run()

if __name__ == '__main__':
    main()
