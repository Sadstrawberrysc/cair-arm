import math
def loss_to_deg(loss):   
    theta_rad = math.acos(max(-1.0, min(1.0, 1 - loss)))
    return theta_rad * 180 / math.pi

deg = loss_to_deg(0.4) 
print(deg)