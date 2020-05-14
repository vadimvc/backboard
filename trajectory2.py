import math
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np
import matplotlib.patches as patches
from mpl_toolkits.mplot3d import Axes3D
import removevals
import removeafterline

def ms_to_fs(meterpersecond):
    return meterpersecond*3.28084

def m_to_f(meters):
    return meters*3.28084

def calcangle_firstQ(y,x):
    slope = (y[-2] - y[-1]) / (x[-2] - x[-1])
    angle=np.arctan(slope)
    angle=math.degrees(angle)
    angle+=90
    return angle

def newline(p1, p2):
    ax = plt.gca()
    xmin, xmax = ax.get_xbound()

    if(p2[0] == p1[0]):
        xmin = xmax = p1[0]
        ymin, ymax = ax.get_ybound()
    else:
        ymax = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmax-p1[0])
        ymin = p1[1]+(p2[1]-p1[1])/(p2[0]-p1[0])*(xmin-p1[0])

    l = mlines.Line2D([xmin,xmax], [ymin,ymax])
    ax.add_line(l)
    return l

def mirrorarray(array):
    arraynew=[]

    for x in range(len(array)):
        if x == 0:
            arraynew.append(array[0])
            magicnumber=array[x]
        
        else:
            val=arraynew[x-1]-(array[x]-magicnumber)
            arraynew.append(val)
            magicnumber=array[x]

    return arraynew

#defining gravity
g=9.81 #m/s^2
g=ms_to_fs(g)

#ball radius in feet
ballrad=0.391667

#just helps as a default run case
#x=np.linspace(0,100,50000)

# defining the line to bounce against
linep1=[13,0]
linep2=[15,50]

def ballthrow(velocity, angle, height, x, linep1, linep2):
    #velocity is in meters/second
    #angle is in degrees
    #height is in meters
    #x is the linearly spaced array

    ##turning everything into feet/second and feet
    velocity=ms_to_fs(velocity)
    
    height=ms_to_fs(height)

    #changing angles to radians
    angle=math.radians(angle)

    #equation for trajectory of a ball
    y=height+x*math.tan(angle)-g*x**2/(2*velocity**2*math.cos(angle)**2)

    #removes all of the values where the ball drops through the floor
    #(array_y,array_x)=removevals.removezeros(y,x,ballrad)

    (array_x,array_y)=removeafterline.remove_to_right_of(x, y, linep1, linep2)

    #return variables y and x
    return array_y,array_x

def bounceball(velocity, angle, height):

    #defines the space over which to see the balls trajectory
    x=np.linspace(0,100,50000)
    linep1=[13,0]
    linep2=[15,50] 
    (y_array,x_array)=ballthrow(velocity, angle, height, x, linep1, linep2)

    #calculates bounce angle
    newangle=calcangle_firstQ(y_array,x_array)

    #setting up the second bounce
    (secondbouncey, secondbouncex)=ballthrow(-velocity/5, newangle, max(x_array), x,linep1, linep2)

    secondbouncex=mirrorarray(secondbouncex)
    secondbouncey=mirrorarray(secondbouncey)

    #offsetting the values of the x matrix by the distance the first bounce was
    
    secondbouncex+=np.max(x_array)

    #appending the values of the second bounce to the first throw
    y_array=np.append(y_array,secondbouncey)
    x_array=np.append(x_array,secondbouncex)

    return y_array,x_array


#(y,x)=ballthrow(9,45,1,x,linep1,linep2)


(y,x)=bounceball(9,45,1)


# All the plotting stuff is below
fig = plt.figure()
plt.plot(x,y, 'r')
#makes a line where the ball bounces
p1 = [0,ballrad]
p2 = [50,ballrad]
#newline(p1,p2)

newline(linep1,linep2)

#plotting
plt.title('Ball Trajectory' )  
plt.xlabel('ball as it\'s flying in x direction (feet)')  
plt.ylabel("ball as it\'s flying in y direction (feet)")
plt.show()
