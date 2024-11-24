def is_water_color(rgb):
    """
    @Checks if the RGB value corresponds to water.
    """
    if (40 < rgb[0] < 70) and (40 < rgb[1] < 80) and (125 < rgb[2] < 170):  # Typical water color range
        return True
    return False

def sweep_for_water():
    sweeping_motor.set_power(20)
    while sweeping_motor.get_encoder() < 90:
        rgb = get_normalized_value()
        if is_water_color(rgb):
            angle = sweeping_motor.get_encoder()
            sweeping_motor.set_power(0)
            sweeping_motor.set_position_relative(-angle)  # Reset position
            return angle
        
    sweeping_motor.set_power(-20)
    while sweeping_motor.get_encoder() > -90:
        rgb = get_normalized_value()
        if is_water_color(rgb):
            angle = sweeping_motor.get_encoder()
            sweeping_motor.set_power(0)
            sweeping_motor.set_position_relative(-angle)  # Reset position
            return angle

    sweeping_motor.set_power(0)
    return None


def avoid_water():
    water_angle = sweep_for_water()
    if water_angle is not None:
        print(f"Water detected at angle: {water_angle}")
        # Decide the turn direction based on the angle of water detcted 
        if water_angle > 0:
            Motion.turn(40, 90, "left")  # Turn left 
        else:
            Motion.turn(40, 90, "right")  # Turn right 
       
        Motion.move(40, 20, "forward")
         
        if is_water():
            avoid_water() 
    else:
        print("No water detected. Continuing..")
        Motion.move(40, 10, "forward")


def check_and_navigate():
    """
    Combine water detection and navigation to mmake sure the robot avoids water zones
    ""
    while True:
        avoid_water()
        Motion.move(40, 10, "forward")  # Continue moving forward if no water is found 

#continue for this week 

if __name__ == "__main__":
    initialize_components()
    check_and_navigate()

