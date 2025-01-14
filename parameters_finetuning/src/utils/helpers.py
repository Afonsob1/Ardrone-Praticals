def normalize_angle(angle):
    while angle > 3.14159:
        angle -= 2 * 3.14159
    while angle < -3.14159:
        angle += 2 * 3.14159
    return angle

def calculate_distance(point1, point2):
    return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

def degrees_to_radians(degrees):
    return degrees * (3.14159 / 180.0)

def radians_to_degrees(radians):
    return radians * (180.0 / 3.14159)