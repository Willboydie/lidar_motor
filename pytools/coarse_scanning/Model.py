import math

X_ANGLE_OFFSET = 3.54
Y_ANGLE_OFFSET = 3.39

CART_BIN_RADIUS = 0.1

def mapToWorldAngles(x_angle, y_angle):
    world_x = x_angle - X_ANGLE_OFFSET
    world_y = y_angle - Y_ANGLE_OFFSET
    return world_x, world_y

def computeCartPoint(x_angle, y_angle, distance):
    world_x, world_y = mapToWorldAngles(x_angle, y_angle)
    return toCartBasic(world_x, world_y, distance)

def toCartBasic(theta, phi, r):
    x = r * math.cos(theta) * math.cos(phi) 
    y = r * math.sin(theta) * math.cos(phi)
    z = r * math.sin(phi)
    return x, y, z