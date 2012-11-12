from joy import *
from socket import socket, AF_INET, SOCK_STREAM, error as SocketError
from json import loads as json_loads

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
    self.sensor = SensorPlan(self,("67.194.200.121",8080))
    self.sensor.start()
    
  def onEvent( self, evt ):
    # Punt to superclass
    # this is here to remind you to override it
    return super( WaypointSensorApp, self ).onEvent(evt)
  
  def onStop( self ):
    self.sensor.stop()
    return super( WaypointSensorApp, self ).onStop()
      
if __name__=="__main__":
  print """
  Running the waypoint sensor demo
  
  Connects to waypoint application and reads sensor.
  
  The waypoint sensor send JSON maps with keys:
  'f', 'b' : front and back sensor values
  'w' : list of lists. Each sub-list is of length 2. List of waypoint
    coordinates, including the next waypoint. Each time the next 
    waypoint changes, it means the previous waypoint was reached.
  """
  app=WaypointSensorApp()
  app.run()

