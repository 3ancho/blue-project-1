from joy import *
import sys
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads
import ckbot.logical 
import time
import math
from collections import deque
from numpy import arctan2

#ckbot.logical.DEFAULT_PORT = "/dev/tty.usbmodemfa131"
# Uncomment above line when using wireless transmitor

# TODO: ruoran or xiangyu: change all print into progress

def get_phi3(cur_point, next_point):
    from numpy import arctan2
    """ 
        This fucntion will use self.cur_point and self.next_point 
        There is no return value, 
        app.direction will be overwitten
        self.theta will be overwitten


        if phi is negative rotate left

        (x,y)

        queue = [{'w': [[2,3][4,5][6,7]] }  {}  {}] 


        cur_point = queue[0][]
    """
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
        self.wp_list = None
        self.ave_f = None
        self.ave_b = None
        self.theta = None

    def onStart(self):
        """ rest positions """
        self.plan_step = 0 

    def get_phi2(self, cur_point, next_point):
        """ 
            This fucntion will use self.cur_point and self.next_point 
            There is no return value, 
            app.direction will be overwitten
            self.theta will be overwitten
        """
        theta = numpy.arctan2(float(next_point[1] - cur_point[1]), (next_point[0] - cur_point[0]) )

        direction = None

        if theta < 0:
            theta = theta + math.pi
            direction = -1
        else:
            direction = 1
            
        phi = theta - math.pi / 2

        return phi, direction

    def get_phi(self):
        """ 
            This fucntion will use self.cur_point and self.next_point 
            There is no return value, 
            app.direction will be overwitten
            self.theta will be overwitten
        """
        self.theta = numpy.arctan2(float(self.next_point[1] - self.cur_point[1]), (self.next_point[0] - self.cur_point[0]) )

        direction = None

        if self.theta < 0:
            self.theta = self.theta + math.pi
            direction = -1
        else:
            direction = 1
            
        phi = self.theta - math.pi / 2

        return phi, direction

    def should_repose(self):
        """ return True or False """
        flag = False
        
        self.forDuration(3)
        queue = self.app.queue  # check the latest queue
        sample = queue[:10] # sample the latest 10 points
        ave_f = average( [v for k, v in sample if v == "f"] )
        ave_b = average( [v for k, v in sample if v == "b"] )

        F = math.sqrt(256.0/(ave_f -1))
        B = math.sqrt(256.0/(ave_b -1))

        progress(queue[10:34])
        # check f,b on which side

        return flag

    def behavior(self):
        for i in range(100000000): # while not reaching the goal point
            progress("Auto mode step -- %d" % self.plan_step)
            self.plan_step += 1

#            queue = self.app.queue
#            progress( "start testing  %s" % str(queue[0:2]) )
#            progress( "start latest   %s" % str(self.app.sensor.latest) )
#            yield self.forDuration(3.5)
#            queue = self.app.queue
#            progress( "end   testing  %s" % str(queue[0:2]) )
#            progress( "end   latest   %s" % str(self.app.sensor.latest) )

            # check the sensor 
            self.app.queu[0]['b']
            self.app.queu[0]['f']

           
            if len(queue) > 0:
                # If found first waypoint 
                if not self.next_point and 'w' in queue[0]:
                    self.cur_point = queue[0]['w'][0]   
                    self.next_point = queue[0]['w'][1] 
                    self.wp_list = queue[0]['w']
                    progress(" queue[0] Initializing waypoints! Cur: %s -- Next: %s" %\
                            (self.cur_point, self.next_point) )

                    phi, direction = self.get_phi()
                    progress("phi =%s   and   direction=%s" % (phi, direction))


                # If found new waypoint
                #if 'w' in queue[0] and abs(self.cur_point[0] - queue[0]['w'][0]) < 5\
                #        and abs(self.cur_point[1] - queue[0]['w'][1]) < 5:
                if 'w' in queue[0] and len(queue[0]['w']) != len(self.wp_list):
                    progress("Found next way point !!!")
                    self.cur_point = queue[0]['w'][0]
                    self.next_point = queue[0]['w'][1]
                    self.wp_list = queue[0]['w']

                    progress("Setting new waypoints! Cur: %s -- Next: %s" %\
                            (self.cur_point, self.next_point) )

                    phi, direction = self.get_phi()

                    progress("phi =%s   and   direction=%s" % (phi, direction))

            # Running Every time
            #progress( str(queue[0]) )

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
            #else:
            #    self.app.move_plan.start(exe_count = 2)
            #    while self.app.move_plan.isRunning():
            #        self.forDuration(0.05)
            #        progress("Moving!")


            yield self.forDuration(0.4)

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
        progress("started rotating plan with count = %d" % self.exe_time)
        Plan.start(self)
        
    def behavior(self):
        for count in range(self.exe_time):
            progress("count %d" % count)
            if count >= self.exe_time -1:
                progress("Count reached, Stop!")
                self.stop()
            #self.app.cur_axis_pos += self.direction * self.unit
            if self.app.testing:
                progress("Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
            else:
                progress("Rotate -- Direction %s, pos %s" % (self.direction, self.app.cur_axis_pos) )
                for i in range(100):
                    #print "%.6f" % time.time()
                    self.forDuration(0.00005)
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

    def change_direction(self):
        self.app.direction = -1 if self.app.direction == 1 else 1

    def start(self, exe_time = 10000):
        self.exe_time = exe_time 
        progress("started move plan with count = %d" % self.exe_time)
        Plan.start(self)

    def behavior(self):
        for count in range(self.exe_time):
            progress("count %d" % count)
            if count >= self.exe_time -1:
                progress("Count reached, Stop!")
                self.stop()
            if self.app.testing:
                print "Move -- Direction %s, torque %s" % (self.direction, self.speed) 
            else:
                print "Move -- Direction %s, torque %s" % (self.direction, self.speed) 
                self.app.robot.at.axis.set_pos(self.app.cur_axis_pos)
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
        if not duration:
            self.start = -100000000 
        else:
            self.start = time.time()
            progress(str(self.start))
        progress("started rotating plan with count = %d" % self.exe_time)
        Plan.start(self)

    def behavior(self):
        for count in range(self.exe_time):
            progress("count %d" % count)
            if count >= self.exe_time -1:
                progress("Count reached, Stop!")
                self.stop()

            if time.time() - self.start > self.duration:
                progress("stop because times up")
                self.stop()

            if self.app.testing:
                print "Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) 
            else:
                print "Turn -- left %s, right %s, axis slack" % (self.direction, self.direction) 
                self.app.robot.at.axis.go_slack()
                self.app.robot.at.left.set_torque(self.direction * self.speed)
                self.app.robot.at.right.set_torque(self.direction * self.speed)
                self.app.cur_axis_pos = self.app.robot.at.axis.get_pos()
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
        self.spec = spec
        self.direction = 1
        self.queue = [] 

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
        #self.sensor = SensorPlan(self,("67.194.202.70",8080))
        #self.sensor.start()

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
            print "Exiting!"
            
            # Close socket connections
            # stop all motors if not in testing mode
            self.stop()

        if evt.type == KEYDOWN and evt.key == K_n: #
            self.auto = 1
            for plan in self.plans:
                plan.stop()
            print "Entering auto"
    
        if evt.type == KEYDOWN and evt.key == K_m: #up
            self.auto = 0
            # when testing, I will manually drive the robot
            # while using AutoPlan to read the states
            # To do so, just comment out following two lines

            #for plan in self.plans:
            #    plan.stop()
            print "Entering manual"


        if evt.type == KEYDOWN and evt.key in [ K_p, K_o, K_DELETE ]: # stop
            for i in range(4):
                self.robot.off()

        # Detail of Audo mode and Manual Mode
        if not self.auto:
            # Manual Mode
            if evt.type == KEYDOWN and evt.key ==  K_u: 
                 self.turn_plan.start(duration = 1)

#                self.robot.at.left.set_mode(0)
#                self.robot.at.right.set_mode(0)
#
#                axis = self.robot.at.axis.get_pos()
#                left = self.robot.at.left.get_pos()
#                right = self.robot.at.right.get_pos()
#                progress("Init Pos of left %s" % str(left))
#                progress("Init Pos of right %s" % str(right))
#
#
#                axis = axis + 100
#                progress(str(axis))
#                left = left - 190
#                progress(str(left))
#                right = right - 190
#                progress(str(right))
#                self.robot.at.axis.set_pos(axis)
#                self.robot.at.left.set_pos(left)
#                self.robot.at.right.set_pos(right)
#
#                self.robot.at.left.set_mode(1)
#                self.robot.at.right.set_mode(1)
                    
            # Testing new function
            if evt.type == KEYDOWN and evt.key ==  K_y : 
                goal = 4500
                progress("K_y")

                self.robot.at.axis.go_slack()
                avalue = self.robot.at.axis.get_pos()

                progress(str(avalue))
                if self.robot.at.axis.get_pos() < 4500:
                    self.turn_plan.direction = -1  # turn left
                    self.turn_plan.start()
                    while self.robot.at.axis.get_pos() < 4500:
                        progress("turning left")
                        
                    self.turn_plan.stop()
#
#                else:  # > 4500 
#                    self.turn_plan.direction = 1  # turn right
#                    self.turn_plan.start()
#                    while self.robot.at.axis.get_pos() > 4500:
#                        progress("turning right")
#                        yield forDuration(0.1)
#                       
#                    self.turn_plan.stop()
#

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
            elif evt.type == KEYUP and evt.key in [K_w, K_s] : # stop moving
                self.move_plan.stop()

            elif evt.type == KEYUP and evt.key in [K_a, K_d, K_LEFT, K_RIGHT] : # stop rotating
                self.rotate_plan.stop()
                progress("rotating keyup event!")

            elif evt.type == KEYUP and evt.key in [K_q, K_e] : # stop turning
                self.turn_plan.stop()


        else: # This is auto mode block
            # Auto Mode
            if not self.auto_plan.isRunning():
                self.auto_plan.start()
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
    self.queue = []
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

      self.latest = dic

      if len(self.app.queue) > 1024:
          self.app.queue.pop()
      self.app.queue.insert(0, dic) # store in queue

      progress("Message received at: " + str(ts))
      #for k,v in dic:
      #    progress("   %s : %s" % (k,repr(v)))

      yield self.forDuration(0.3)

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
