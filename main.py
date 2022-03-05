import os
import time
import sys
import RPi.GPIO as GPIO

GPIO_TEMP_PIN = 4
GPIO_PWM_PINS = [17, 18]
PWMS = []

MIN_TEMP = 35
MAX_TEMP = 55

FAN_LOW = 20
FAN_HIGH = 100

FAN_OFF = 0
FAN_MAX = 100


dc = 100  # duty cycle (0-100) for PWM pin

# pin setup
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme


def get_cpu_temperature():
    res = os.popen(
        'cat /sys/devices/virtual/thermal/thermal_zone0/temp').readline()
    temp = (float(res)/1000)
    return temp


def get_temperature():
    import Adafruit_DHT
    sensor = Adafruit_DHT.AM2302
    humidity, temperature = Adafruit_DHT.read_retry(sensor, GPIO_TEMP_PIN)


def get_ambiental_temp():
    ## TODO: implement
    cpu_temp = get_cpu_temperature()
    humidity, temp = get_temperature()
    return temp


def setup_pins():
    for pin in GPIO_PWM_PINS:
        GPIO.setup(pin, GPIO.OUT)
        pwm = GPIO.PWM(pin, 50)  # 50 Hz
        PWMS.append(pwm)

    GPIO.setup(GPIO_TEMP_PIN, GPIO.IN)

def start_fans():
    for pwm in PWMS:
        pwm.start(dc)


def cleanup():
    for pwm in PWMS:
        set_fan_speed(0, pwm)
        pwm.stop()  # stop PWM
    GPIO.cleanup()  # cleanup all GPIO
    print("Exiting")
    sys.stdout.flush()


def manual():
    v = input("Set the speed (0-100)")
    try:
        v = int(v)
    except:
        print(f"Invalid input: {v}")
        return False
    for pwm in PWMS:
        set_fan_speed(v, pwm)
    return True


def set_fan_speed(speed, gpio):
    speed = max(0, min(speed, 100))  # clamp speed to 0-100
    gpio.ChangeDutyCycle(speed)


def auto():
    temp = get_ambiental_temp()

    # Turn off the fan if temperature is below MIN_TEMP
    if temp < MIN_TEMP:
        speed = FAN_OFF

    # Set fan speed to MAXIMUM if the temperature is above MAX_TEMP
    elif temp > MAX_TEMP:
        speed = FAN_MAX

    # Caculate dynamic fan speed
    else:
        step = (FAN_HIGH - FAN_LOW)/(MAX_TEMP - MIN_TEMP)
        delta = temp - MIN_TEMP
        speed = FAN_LOW + (round(delta) * step)

    for pwm in PWMS:
        set_fan_speed(speed, pwm)

    print(
        f"\rCurrent temperature: {temp:.2f} *C  |  current speed: {speed:.2f}", end="")


def main():
    setup_pins()
    start_fans()

    try:
        while True:

            # manual()
            auto()

            time.sleep(1)

    except Exception as e:
        print("Got exception: ", str(e))
    finally:
        cleanup


if __name__ == "__main__":
    main()
