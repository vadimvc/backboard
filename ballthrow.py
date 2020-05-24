import math
import numpy as np
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import mpl_toolkits.mplot3d.art3d as art3d

g=9.81 #m/s^2
no_of_steps=15
BackBoardHeight=.121
BackBoardWidth=.1829

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
    
    endt=(targetx-x)/Velocity_x
    t=np.linspace(0, endt, num=no_of_steps)
    #Equations of motion

    Z=z+Velocity_y_initial*t-0.5*g*t**2
    Y=np.linspace(y,targety,num=no_of_steps)
    X=x+Velocity_x*t

    return X,Y,Z

def PlotPath3D(x,y,z,alpha,targetx,targety,targetz,xrot,yrot):
    (xpath,ypath,zpath)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
    (Line0,Line1,Line2,Line3)=GenerateLines(xrot,yrot,targetz) 
    
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(xpath, ypath, zpath, 'blue', label='ball path')
    ax.plot3D([Line0[0][0][0],Line0[0][1][0]], [Line0[1][0][0],Line0[1][1][0]], [Line0[2][0][0],Line0[2][1][0]], 'red', label='backboard')
    ax.plot3D([Line1[0][0][0],Line1[0][1][0]], [Line1[1][0][0],Line1[1][1][0]], [Line1[2][0][0],Line1[2][1][0]], 'red')
    ax.plot3D([Line2[0][0][0],Line2[0][1][0]], [Line2[1][0][0],Line2[1][1][0]], [Line2[2][0][0],Line2[2][1][0]], 'red')
    ax.plot3D([Line3[0][0][0],Line3[0][1][0]], [Line3[1][0][0],Line3[1][1][0]], [Line3[2][0][0],Line3[2][1][0]], 'red')    
    plt.title('Ball Trajectory' )  
    ax.set_xlabel('Ball as it\'s flying in x direction')  
    ax.set_ylabel('Ball as it\'s flying in y direction')
    ax.set_zlabel('Ball as it\'s flying in z direction')
    plt.legend()
    plt.show()

def PlotPath2D(x,y,z,alpha,targetx,targety,targetz):
    (xpath,ypath,zpath)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
    fig = plt.figure(figsize=(8, 9))
    plt.subplot(211)
    plt.plot(xpath,zpath,'r')
    plt.scatter(xpath, zpath, s=337, edgecolors= "black")
    plt.title('Ball Trajectory' )  
    plt.xlabel('Ball as it\'s flying in x direction (meters)')  
    plt.ylabel("Ball as it\'s flying in y direction (meters)")
    
    
    #spaces the start and end points so I can do a scatter plot showing the ball
    xtemp=np.linspace(x,targetx,num=no_of_steps)
    ytemp=np.linspace(y,targety,num=no_of_steps)
    
    plt.subplot(212)
    plt.plot(xtemp,ytemp,'r')
    plt.scatter(xtemp,ytemp, s=337, edgecolors= "black")
    plt.xlabel('Ball as it\'s flying in x direction (meters)') 
    plt.ylabel("Ball as it\'s flying in z direction (meters)")
    plt.show()

def GenerateLines(xrot,yrot,targetz):
    #intended to generate all the lines which the ball can bounce again
    #alpha and beta represent the 2 angles, alpha is up/down and beta is left/right
    
    HalfHeight=BackBoardHeight/2
    HalfWidth=BackBoardWidth/2
   
    #alpha is the rotation about z axis (sideways)
    #beta is the rotation about y axis (up/down)
    #gamma is the rotation about x axis (useless) - keep this zero
    alpha=xrot
    beta=yrot
    gamma=0


    alpha=math.radians(alpha)
    beta=math.radians(beta)
    gamma=math.radians(gamma)

    #rotation matrix for euler angles
    RotationMatrix=[[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma)-np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma)+np.sin(alpha)*np.sin(gamma)],
                    [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma)+np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma)-np.cos(alpha)*np.sin(gamma)],
                    [-1*np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]]
    
    #xyz for backboard positions pre rotation                
    TR=[[0],
        [HalfWidth],
        [HalfHeight]]

    TL=[[0],
        [-HalfWidth],
        [HalfHeight]]

    BL=[[0],
        [-HalfWidth],
        [-1*HalfHeight]]

    BR=[[0],
        [HalfWidth],
        [-1*HalfHeight]]

    #matrix multiplication for transform
    TR_Rotated=np.dot(RotationMatrix,TR)
    TL_Rotated=np.dot(RotationMatrix,TL)
    BL_Rotated=np.dot(RotationMatrix,BL)
    BR_Rotated=np.dot(RotationMatrix,BR)

    #adding offset off the ground
    TR_Rotated[2]=targetz+TR_Rotated[2]
    TL_Rotated[2]=targetz+TL_Rotated[2]
    BL_Rotated[2]=targetz+BL_Rotated[2]
    BR_Rotated[2]=targetz+BR_Rotated[2]

    #defining lines
    Line0=[[TR_Rotated[0],TL_Rotated[0]],[TR_Rotated[1],TL_Rotated[1]],[TR_Rotated[2],TL_Rotated[2]]]
    Line1=[[TL_Rotated[0],BL_Rotated[0]],[TL_Rotated[1],BL_Rotated[1]],[TL_Rotated[2],BL_Rotated[2]]]
    Line2=[[BL_Rotated[0],BR_Rotated[0]],[BL_Rotated[1],BR_Rotated[1]],[BL_Rotated[2],BR_Rotated[2]]]
    Line3=[[BR_Rotated[0],TR_Rotated[0]],[BR_Rotated[1],TR_Rotated[1]],[BR_Rotated[2],TR_Rotated[2]]]

    #return for lines. They are in conveenitnt to plot order ([x1,x2],[y1,y2],[z1,z2])
    return Line0,Line1,Line2,Line3

def PlotGeneratedLines(xrot,yrot,targetz):
    (Line0,Line1,Line2,Line3)=GenerateLines(xrot,yrot,targetz)
 
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D([Line0[0][0][0],Line0[0][1][0]], [Line0[1][0][0],Line0[1][1][0]], [Line0[2][0][0],Line0[2][1][0]], 'red')
    ax.plot3D([Line1[0][0][0],Line1[0][1][0]], [Line1[1][0][0],Line1[1][1][0]], [Line1[2][0][0],Line1[2][1][0]], 'red')
    ax.plot3D([Line2[0][0][0],Line2[0][1][0]], [Line2[1][0][0],Line2[1][1][0]], [Line2[2][0][0],Line2[2][1][0]], 'red')
    ax.plot3D([Line3[0][0][0],Line3[0][1][0]], [Line3[1][0][0],Line3[1][1][0]], [Line3[2][0][0],Line3[2][1][0]], 'red')
    plt.title('BasketballNet' )  
    ax.set_xlabel('X')  
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.show()

def CalcAngles(x,y,z,alpha,targetx,targety,targetz):
    (X,Y,Z)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)

    print(X[-1])
    print(X[-2])

#test case
x=-3; y=-2; z=0; alpha=60
targetx=0;targety=0; targetz=3
xrot=0; yrot=0

#CalcAngles(x,y,z,alpha,targetx,targety,targetz)

#PlotGeneratedLines(xrot,yrot,targetz)

#PlotPath2D(x,y,z,alpha,targetx,targety,targetz)

PlotPath3D(x,y,z,alpha,targetx,targety,targetz,xrot,yrot)



#(x,y,z)=CalculatePath(x,y,z,alpha,targetx,targety,targetz)
#print(x)
#CalcBeta(x,y,targetx,targety)
#print(CalcInitialVelocity(x,z,alpha))