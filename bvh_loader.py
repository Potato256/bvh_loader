import os
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation
from scipy.spatial.transform import Rotation as R
import time, sys
from math import sin, cos, pi

IDENTITY = np.array([[1,0,0],[0,1,0],[0,0,1]], dtype=np.float64)


global fig
global ax
fig = plt.figure()
ax  = p3.Axes3D(fig)


class Point():
  def __init__(self):
    self.pos = np.array([0,0,0], dtype=np.float64)
    self.offset = np.array([0,0,0], dtype=np.float64)
    self.rotate = IDENTITY
    self.name = ''
    self.channels_num = 0
    self.channels = []
    self.channels_content = [0 for i in range(0,12)]
    self.children = [] 
    self.type = '' 
    self.isLeaf = False
    self.time3 = time.time()
    # This line connects with father
    self.line =ax.plot([0],[0],[0])[0]

  def build(self, str):
    '''This function process a string read from a bvh file to get 
          skeleton infos. It works recursively.'''
    
    if str[:9] == 'HIERARCHY':
      tmp = str.partition('ROOT')
      self.type = 'ROOT'
    elif str[:5] == 'JOINT':
      tmp = str.partition('JOINT')
      self.type = 'JOINT'
    elif str[:8] == 'End Site':
      tmp = str.partition('End Site')
      self.isLeaf = True
      self.type = 'End Site'

    tmp = tmp[2].partition('{')
    if not self.isLeaf:
      self.name = tmp[0].lstrip(' \t\n').rstrip(' \t\n')
    else:
      self.name = 'End'
      
    tmp = tmp[2].partition('OFFSET')
    if not self.isLeaf:
      tmp = tmp[2].partition('CHANNELS')
    else:
      tmp = tmp[2].partition('}')

    # This part get the offset
    offset_str = tmp[0].lstrip(' \t\n').rstrip(' \t\n')
    offset_tmp = offset_str.partition(' ')
    self.offset[0] = float(offset_tmp[0])
    offset_tmp = offset_tmp[2].lstrip().partition(' ')
    self.offset[1] = float(offset_tmp[0])     # python str=>float will neglect
    self.offset[2] = float(offset_tmp[2])     # space and newline on both sides.
    # This part get the channels
    if(self.isLeaf):
      return
    tmp = tmp[2].lstrip(' \t\n').partition(' ')
    self.channels_num = int(tmp[0])
    str = tmp[2]
    for i in range(self.channels_num):
      cnt =0
      while(True):
        cnt += 1
        if str[cnt] == ' ' or str[cnt] == '\n':    
          self.channels.append(str[:cnt])
          str = str[cnt:].lstrip(' \n')
          break
    str = str.lstrip(' \n\t')
    # This part deal with the children
    while(str[0]!='}'):
      child = Point()
      child.build(str)
      self.children.append(child)
      leftBracket = 0
      cnt = 0
      while(leftBracket == 0):
        if str[cnt] == '{':
          leftBracket += 1
        cnt += 1
      while(leftBracket != 0):
        if str[cnt] == '{':
          leftBracket += 1
        elif str[cnt] == '}':
          leftBracket -= 1
        cnt += 1
      str = str[cnt:].lstrip(' \t\n')


  def draw(self, last):

    #ax.scatter(self.pos[0], self.pos[2], self.pos[1], c='red', marker ='o')
    if self.type != 'ROOT':
      self.line.set_data(np.array([[last.pos[0], self.pos[0]],
                                   [last.pos[2], self.pos[2]]],dtype=np.float64))
      self.line.set_3d_properties(np.array([last.pos[1], self.pos[1]], 
                                          dtype=np.float64))
    for p in self.children:
      p.draw(self)


  def update_pos(self,last):
    # rotate in zxy order
    if self.type=='ROOT':
      adjust = 3
    else:
      adjust = 0

    if self.type != 'End Site' :
      tmp = self.channels_content
      angX, angY, angZ = pi*self.channels_content[adjust+0]/180,\
                         pi*self.channels_content[adjust+1]/180,\
                         pi*self.channels_content[adjust+2]/180
      r_x = np.array([[1           ,0            , 0],
                      [0, cos(angX), -sin(angX)],
                      [0, sin(angX), cos(angX)]], dtype=np.float64)
      
      r_y = np.array([[cos(angY),0 , sin(angY)],
                      [0           ,1            , 0],
                      [-sin(angY),0, cos(angY)]], dtype=np.float64)
      r_z = np.array([[cos(angZ), -sin(angZ), 0],
                      [sin(angZ), cos(angZ), 0],
                      [0           ,0            , 1]], dtype=np.float64)
      # r = np.dot(r_x, r_z)
      # r = np.dot(r_y, r)
      r = r_x @ r_y @ r_z
      self.rotate = np.dot(last.rotate, r)

    if  self.type =='ROOT':
      self.pos = np.array([self.channels_content[0], 
        self.channels_content[1], self.channels_content[2]], dtype=np.float64)
    else:
      self.pos = last.pos + np.dot(last.rotate, self.offset)
    
    for p in self.children:
      p.update_pos(self)

  def read_frame(self, frameList):
    i = 0
    while frameList and i < self.channels_num:
      if frameList[0] != '':
        self.channels_content[i] = float(frameList[0])
        i += 1
      del frameList[0]

    for child in self.children:
      child.read_frame(frameList)

def update(frameNum, root, myFile):
  print()
  print(frameNum)

  time1 = time.time()
  print(f'draw: {time1-root.time3}')

  for i in range(0, 4):
    frame = myFile.readline()       # one frame in one line
  frame = myFile.readline()       # one frame in one line
  
  frameList = frame.split(' ')
  if frameList:
    root.read_frame(frameList)

  time2 = time.time()
  print(f'read: {time2-time1}')

  root.update_pos(Point())

  root.time3 = time.time()
  print(f'calculate: {root.time3-time2}')

  root.draw(Point())

print(os.getcwd())  
bvhFile = 'data/bollywood.bvh'
str = ''
root = Point()

with open(bvhFile, 'r') as myFile:
  for line in myFile:
    if('MOTION') in line:
      break
    str += line

  # Setting the axes properties
  ax.set_xlim3d([-100, 100])
  ax.set_xlabel('X')

  ax.set_ylim3d([-100, 100])
  ax.set_ylabel('Z')

  ax.set_zlim3d([-100, 100])
  ax.set_zlabel('Y')
  
  ax.set_title(bvhFile)
  
  root.build(str)  

  frame_num = myFile.readline().partition(':')
  frame_num = int(frame_num[2])
  frame_time = myFile.readline().partition(':')
  frame_time = float(frame_time[2])
  
  frame_num = int(frame_num/5) - 1 
  line_ani = animation.FuncAnimation(fig, update, frame_num, 
                                     fargs=(root, myFile),
                                     interval=5*frame_time*1000, repeat=False,
                                     repeat_delay=1000, blit=False)
  line_ani.save(bvhFile.partition('/')[2].partition('.')[0]+'.gif'
        , writer='imagemagick', fps=0.2/frame_time)

  plt.show()
  
