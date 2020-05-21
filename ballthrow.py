import math
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt


import matplotlib.patches as patches
import mpl_toolkits.mplot3d.art3d as art3d

g=9.81 #m/s^2

def CalcInitialVelocity(x, z, targetx, targetz, alpha):
    #x y and z position the ball is being thrown from
    #x is going down the field, y is lateral, and z is height
    # numbers are positive going down the field, z is up and y completes right hand rule
    #alpha is the angle relative to the ground from which you're throwing the ball
    #all values are in meters
    
    #changing to a coord system where the place you're shooting from is 0,0
    dx=abs(targetx-x)
    dz=abs(targetz-z)

    alpha=math.radians(alpha)
    beta=math.radians(alpha)

    #physics ref https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Map%3A_University_Physics_I_-_Mechanics%2C_Sound%2C_Oscillations%2C_and_Waves_(OpenStax)/04%3A_Motion_in_Two_and_Three_Dimensions/4.04%3A_Projectile_Motion
    term1 = g/2
    term2 = dx**2/math.cos(alpha)**2
    term3 = 1/(dx*math.tan(alpha)-dz)

    Velocity_squared=term1*term2*term3
    Velocity = math.sqrt(Velocity_squared)

    #solving for velocity values. note that Vox and Vx are the same
    Velocity_x=Velocity*np.cos(alpha)
    Velocity_y_initial=Velocity*np.sin(alpha)
    Velocity_y_final=Velocity_y_initial**2+2*g*(targetz-z)

    return Velocity_x, Velocity_y_final, Velocity_y_initial
    #velocity terms outputted in m/s

def CalcBeta(x,y,targetx,targety):
    #inputs the target location and outputs the angle relative to the sagital plane of the court
    deltay=targetx-x
    deltax=targety-y
    angle=math.atan2(deltay,deltax)
    beta=90-math.degrees(angle)
    print(beta)
    return beta
    #output is in degrees
    
def CalculatePath(x,y,z,alpha,targetx,targety,targetz):
    #Calculate the initial velocity given the starting points and the end desired location
    (Velocity_x, Velocity_y_final, Velocity_y_initial)=CalcInitialVelocity(x,z,targetx,targetz,alpha)
    
#    Velocity_y_final=Velocity_y_final-g*t
#########TODO CALCULATE THE ACTUAL RIGHT AMOUNT OF TIME THIS TAKES AND THEN HAVE THAT BE THE UPPER BOUND 
    t=np.linspace(0, 3, num=50)

    #Equations of motion

    Z=z+Velocity_y_initial*t-0.5*g*t**2
    Y=np.linspace(y,targety,num=50)
    X=x+Velocity_x*t

    return X,Y,Z

def PlotPath3D(x,y,z,alpha,targetx,targety,targetz):
    (x,y,z)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(x, y, z, 'red')
#######TODO MAKE THIS PATCH WORK CORRECTLY
    # rect = patches.Rectangle((0,0),0.1,.18,linewidth=1,edgecolor='blue',facecolor='blue')
    # ax.add_patch(rect)
    # art3d.pathpatch_2d_to_3d(rect, z=0, zdir="z")

    
    plt.title('Ball Trajectory' )  
    ax.set_xlabel('Ball as it\'s flying in x direction')  
    ax.set_ylabel('Ball as it\'s flying in y direction')
    ax.set_zlabel('Ball as it\'s flying in z direction')
    plt.show()

def PlotPath2D(x,y,z,alpha,targetx,targety,targetz):
    (x,y,z)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
    fig = plt.figure()
    plt.plot(x,z,'r')
    plt.title('Ball Trajectory' )  
    plt.xlabel('Ball as it\'s flying in x direction (feet)')  
    plt.ylabel("Ball as it\'s flying in y direction (feet)")
    plt.show()


#test case
x=-5; y=0; z=2; alpha=60
targetx=0;targety=0; targetz=8

PlotPath2D(x,y,z,alpha,targetx,targety,targetz)


#(x,y,z)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
#print(x)
#CalcBeta(x,y,targetx,targety)
#print(CalcInitialVelocity(x,z,alpha))
