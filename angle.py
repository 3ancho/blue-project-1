import math

def angle( cur_point, next_point )
    # out put direction and theta

    theta = math.atan(float(next_point[1] - cur_point[1]) / (next_point[0] - cur_point[0]) )

    direction = self.app.direction

    if theta < math.pi:
        direction = 1
    else:
        theta = theta - math.pi
        direction = -1

    return direction, theta



