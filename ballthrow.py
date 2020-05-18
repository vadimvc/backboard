import math
g=9.81 #m/s^2



def CalcInitialVelocity(x, z, alpha):
    #x y and z position the ball is being thrown from
    #x is going down the field, y is lateral, and z is height
    # numbers are negative going down the field, z is up and y completes right hand rule
    #alpha is the angle relative to the ground from which you're throwing the ball
    #all values are in meters
    
    #in every case here I am assuming that we are starting 2 meters off the ground since I will not
    #be iterating the height
    z=z-2
    
    alpha=math.radians(alpha)
    beta=math.radians(alpha)

    #physics ref https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Map%3A_University_Physics_I_-_Mechanics%2C_Sound%2C_Oscillations%2C_and_Waves_(OpenStax)/04%3A_Motion_in_Two_and_Three_Dimensions/4.04%3A_Projectile_Motion
    term1 = g/2
    term2 = x**2/math.cos(alpha)**2
    term3 = 1/(x*math.tan(alpha)-z)

    Velocity_squared=term1*term2*term3
    Velocity = math.sqrt(Velocity_squared)

    return Velocity
    #velocity outputted in m/s

def CalcBeta(x,y,targetx,targety):
    #inputs the target location and outputs the angle relative to the sagital plane of the court
    deltay=targetx-x
    deltax=targety-y
    angle=math.atan2(deltay,deltax)
    beta=90-math.degrees(angle)
    print(beta)
    return beta
    #output is in degrees
    
def CalculatePath(x,y,z,initial_velocity,alpha,beta):
    ###
    initial_velocity=CalcInitialVelocity(x,z,alpha)



#test case
x=-10; y=10; z=10; alpha=60
targetx=0;targety=0



#CalcBeta(x,y,targetx,targety)
#print(CalcInitialVelocity(x,z,alpha))
