from utils import sound
from utils.brick import TouchSensor, wait_ready_sensors, reset_brick

# Constants
SOUND1 = sound.Sound(duration=0.1, pitch="C6", volume=100)
SOUND2 = sound.Sound(duration=0.1, pitch="D6", volume=100)
SOUND3 = sound.Sound(duration=0.1, pitch="E6", volume=100)
SOUND4 = sound.Sound(duration=0.1, pitch="F6", volume=100)
DATA_FILE = "../data/touch_sensors.csv"

print("Program start.\nWaiting for sensors to turn on...")

# Sensors
TOUCH_SENSORS = [TouchSensor(1), TouchSensor(2), TouchSensor(3), TouchSensor(4)]

wait_ready_sensors(True)
print("Done initializing.")


def play_sound(sound):
    sound.play()
    sound.wait_done()


def get_states():
    return ''.join('1' if sensor.is_pressed() else '0' for sensor in TOUCH_SENSORS)


def flute_mechanism():
    try:
        output_file = open(DATA_FILE, "w")
        while True:
            states = get_states()
            output_file.write(f"{states}\n")
            
            if states == "1111":  # Emergency stop
                break
            
            if states == "1000":  # Note 1
                play_sound(SOUND1)
                
            if states == "0100":  # Note 2
                play_sound(SOUND2)
                
            if states == "0010":  # Note 3
                play_sound(SOUND3)
                
            if states == "0001":  # Note 4
                play_sound(SOUND4)
            
    except KeyboardInterrupt:
        pass
    
    finally:
        print("Done")
        output_file.close()
        reset_brick()
        exit()


if __name__ == "__main__":
    flute_mechanism()

