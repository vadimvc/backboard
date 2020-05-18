#steps to solving this thing

# 1) chose a location somewhere in x y space

# 2) pick a location on the board you want to hit

# 3) figure out an iteration of velocities and angles which will lead you to hitting that location
#    maybe a good strategy is to have a fairly limited number of velocities you iterate through and
#    you figure out the angle based on that

# 4) for every loation you chose on the board now change the angle of the backboard (in both planes) an drun case 3 again
#
# 5) throw out all of the solutions which don't make it into the net and then average all of the other angles together.
#    *I'm not sure avererage is actually the right solution but let's start there
#
# 6) run this same case for every x y location on the hoop in an x by n grid
#
# 7) Compile a list of the best angles for every position and now fit a surface to all of those points and their respective angles
#
# 8) Rerun this entire script with the ball colliding with this new curve.
#
# 9) Keep doing this until the curve shape changes less than 0.05mm?

