import numpy as np

W=640
H=480


def transformcoordinates(x,y):

    P1 = (0,0)
    P2 = (W,H)
    p1 = (-1,1)
    p2 = (1,-1)

    M = np.matrix([[P1[0], P1[1], 1, 0], [-P1[1] , P1[0], 0, 1], [P2[0], P2[1], 1, 0], [-P2[1], P2[0], 0, 1]])
    Minv = M.getI()
    tC = np.matrix([[p1[0]], [p1[1]], [p2[0]], [p2[1]]])
    C= Minv*tC
    xp = C[0]*x+C[1]*y+C[2]
    yp = C[1]*x-C[0]*y+C[3]
    return (xp,yp)


res = transformcoordinates(0,0)
print "{}".format(res)
res = transformcoordinates(W/2,H/2)
print "{}".format(res)
res = transformcoordinates(W,H)
print "{}".format(res)
res = transformcoordinates(0,H)
print "{}".format(res)
res = transformcoordinates(W,0)
print "{}".format(res)