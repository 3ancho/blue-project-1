from joy import *
import sys
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical 
import time
import math
from collections import deque
from numpy import arctan2

#ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfd111"

# Uncomment above line when using wireless transmitor

# TODO: ruoran or xiangyu: change all print into progress

def get_phi(cur_point, next_point):
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
        self.plan_step = None
        self.cur_point = None
        self.next_point = None
        self.wp_list = []              # dic['w']
        self.ave_f = None
        self.ave_b = None
        self.theta = None

    def onStart(self):
        """ rest positions """
        self.plan_step = 0 

    def behavior(self):
        for i in range(100000000): # while not reaching the goal point
            progress("Auto mode step -- %d" % self.plan_step)
            self.plan_step += 1
            queue = self.app.queue

            if len(queue) > 0:
                # If found first waypoint 
                if not self.next_point and self.app.latest_w:
                    self.cur_point = self.app.latest_w[0]   
                    self.next_point = self.app.latest_w[1] 
                    progress(" queue[0] Initializing waypoints! Cur: %s -- Next: %s" %\
                            (self.cur_point, self.next_point) )
                    self.app.move_plan.stop()

                    phi, direction = get_phi(self.cur_point, self.next_point)

                    progress("phi =%s   and   direction=%s" % (phi, direction))

                    self.app.direction = direction
                    
                    # action !!!
                    if not self.app.turn2_plan.isRunning():
                        self.app.turn2_plan.start(goal=phi)

                # If found new waypoint
                #if 'w' in queue[0] and abs(self.cur_point[0] - queue[0]['w'][0]) < 5\
                #        and abs(self.cur_point[1] - queue[0]['w'][1]) < 5:
                if len(self.app.latest_w) != len(self.wp_list):
                    progress("Found next way point !!!")
                    self.app.move_plan.stop()

                    self.cur_point = self.app.latest_w[0]   
                    self.next_point = self.app.latest_w[1] 

                    progress("Setting new waypoints! Cur: %s -- Next: %s" %\
                            (self.cur_point, self.next_point) )

                    phi, direction = get_phi(self.cur_point, self.next_point)

                    progress("phi =%s   and   direction=%s" % (phi, direction))
                    
                    # action !!!
                    if not self.app.turn2_plan.isRunning():
                        self.app.turn2_plan.start(goal=phi)

                self.wp_list = self.app.latest_w
                if not self.app.move_plan.isRunning() and self.wp_list and not self.app.turn2_plan.isRunning():
                    self.app.move_plan.direction = self.app.direction
                    self.app.move_plan.start(duration = 1.5)

            # y-ok?
            # if self.should_repose():
            #    self.app.rotate_plan.start(exe_count = 2)
            #    while self.app.rotate_plan.isRunning():
            #        self.forDuration(0.01)
            #        progress("Reposing!")

            ## distance ok?
            #elif self.should_turn():
            #    self.app.turn_plan.start(exe_count = 2)
            #    while self.app.turn_plan.isRunning():
            #        self.forDuration(0.05)
            #        progress("Get close to line: Turning!")

            #    self.app.move_plan.start(exe_count = 2)
            #    while self.app.move_plan.isRunning():
            #        self.forDuration(0.05)
            #        progress("Get close to line: Moving!")

            ## move!

            yield self.forDuration(0.8) # change to 0.3 if necessary

class Rotate( Plan ):
    """ RotatePlan will handle the axis servo movement 
        Rorating axis servo is to adjust the platform (laser) 
        pointing direction, this happens when wheels are locked.

        direction = -1    Rotate left
        direction = 1     Rotate right 
    """
    def __init__(self, app, direction=1, unit=2, *arg, **kw):
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
                progress("Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
            else:
                progress("Unit Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
                for i in range(50):
                    for i in range(1000):
                        aaaaa = 100 * 100
                    self.app.cur_axis_pos += self.direction * self.unit
                    self.app.robot.at.axis.set_pos(self.app.cur_axis_pos)

            yield 

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
        if self.duration:
            progress("starting turn plan with duration %s" % duration)
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
                progress( "Unit Move -- Direction %s, torque %s" % (self.direction, self.speed) )
                self.app.robot.at.axis.set_pos(self.app.robot.at.axis.get_pos())
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
            #progress("count %d" % count)
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
                progress("Unit Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) )
                self.app.robot.at.axis.go_slack()
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

    def start(self, goal = None):
        self.goal_pos = goal
        self.start_pos = self.app.robot.at.axis.get_pos() 
        self.app.move_plan.stop()
        self.app.rotate_plan.stop()
        self.app.turn_plan.stop()
        Plan.start(self)

    def behavior(self):
        while True:
            self.app.move_plan.stop()
            if self.start_pos < self.goal_pos:
                self.app.rotate_plan.direction = 1
                self.app.turn_plan.direction = -1
                self.app.rotate_plan.start()
                self.app.turn_plan.start()
                while self.app.robot.at.axis.get_pos() < self.goal_pos:
                    pass
                    #progress("turning left")
                    yield self.forDuration(0.1)
                self.app.rotate_plan.stop()
                self.app.turn_plan.stop()
                self.stop()
                break

            else: #start_pos > goal_pos
                self.app.rotate_plan.direction = -1
                self.app.turn_plan.direction = 1
                self.app.rotate_plan.start()
                self.app.turn_plan.start()
                while self.app.robot.at.axis.get_pos() > self.goal_pos:
                    pass 
                    #progress("turning right")
                    yield self.forDuration(0.1)
                self.app.rotate_plan.stop()
                self.app.turn_plan.stop()
                self.stop()
                break

    def onStop(self):
        if not self.app.testing:
            self.app.turn_plan.stop()
            self.app.rotate_plan.stop()
    

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

    def onStart(self):
        self.output = self.setterOf(self.spec)
        self.auto = 0
        self.state = None
        self.cur_axis_pos = -122 # this is 'zero' 
        self.unit_turn = 200
        
        # Plan initialization
        self.move_plan = Move(self, direction=1)
        self.turn_plan = Turn(self, direction=1)
        self.rotate_plan = Rotate(self, direction=1)
        self.auto_plan = AutoPlan(self)
        self.turn2_plan = Turn2(self)

        if not self.no_sensor:
            self.sensor = SensorPlan(self,("67.194.202.70",8080))
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
            self.auto = 1
            for plan in self.plans:
                plan.stop()
            progress( "Entering auto")
    
        if evt.type == KEYDOWN and evt.key == K_m: #up
            self.auto = 0
            # when testing, I will manually drive the robot
            # while using AutoPlan to read the states
            # To do so, just comment out following two lines

            #for plan in self.plans:
            #    plan.stop()
            progress( "Entering manual")


        if evt.type == KEYDOWN and evt.key in [ K_p, K_DELETE ]: # stop
            for i in range(4):
                self.robot.off()

        # Detail of Audo mode and Manual Mode
        if not self.auto:
            # Manual Mode


            if evt.type == KEYDOWN and evt.key ==  K_u:  # Turn left
                self.turn2_plan.start(goal=-4500)

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

        else: # This is auto mode block
            # Auto Mode
            if not self.auto_plan.isRunning():
                self.auto_plan.start()
                #self.tag_turn.start()
            else:
                self.auto_plan.push(evt)

    def onStop(self):
        for plan in self.plans:
            plan.stop()

        self.move_plan.stop()
        self.rotate_plan.stop()
        self.turn_plan.stop()
        self.auto_plan.stop()
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
    self.app.queue = []
    self.latest = None 
 
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

      if len(self.app.queue) > 1024:
          self.app.queue.pop()
      self.app.queue.insert(0, dic) # store in queue
      if 'w' in dic:
          self.app.latest_w = dic['w']

      #progress("Message received at: " + str(ts))
      #progress("   %s " % dic)

      yield self.forDuration(0.4)

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
