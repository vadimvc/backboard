# import math
# import matplotlib.pyplot as plt
# import matplotlib.lines as mlines
import numpy as np
# import matplotlib.patches as patches
# from mpl_toolkits.mplot3d import Axes3D


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



def remove_to_right_of(x_array, y_array, linep1, linep2):
    x_array_new=[]
    y_array_new=[]

    for x in range(len(x_array)):
       # print(target_array[x])
      #  print(following_array[x])
        value=(linep2[0]-linep1[0])*(y_array[x]-linep1[1])-(x_array[x]-linep1[0])*(linep2[1]-linep1[1])

        if value>0:
       #     print('The value is to on the left side of the line')
            x_array_new.append(x_array[x])
            y_array_new.append(y_array[x])
        
    return x_array_new, y_array_new

# array1=np.linspace(0,75,76)
# array2=array1*array1*array1

# linep1=[50,0]
# linep2=[50,50]

# (x, y)=remove_to_right_of(array1,array2,linep1,linep2)

# # All the plotting stuff is below
# fig = plt.figure()
# plt.plot(x,y, 'r')
# #makes a line where the ball bounces
# newline(linep1,linep2)


# #plotting
# plt.title('Ball Trajectory' )  
# plt.xlabel('ball as it\'s flying in x direction (feet)')  
# plt.ylabel("ball as it\'s flying in y direction (feet)")
# plt.show()


# print(x_array)
# print(y_array)