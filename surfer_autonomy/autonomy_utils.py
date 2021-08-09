import math

def wrapToPi(ang):
    if(ang > math.pi):
        ang = ang - 2*math.pi

    if(ang < -math.pi):
        ang = ang + 2*math.pi

    return(ang)

def norm2d(vect):
    norm = math.sqrt(vect[0]*vect[0] + vect[1]*vect[1])
    return norm

def norm3d(vect):
    norm = math.sqrt(vect[0]*vect[0] + vect[1]*vect[1] + vect[2]*vect[2])
    return norm
