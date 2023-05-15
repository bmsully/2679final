#!usr/bin/env python3
import RPi.GPIO as GPIO #! Cannot install on MacOS, will need to on RaspberryPi
import time
import threading

## Consider using the following rather than time.sleep
# event = threading.Event()
# event.wait(1)
## event.wait() is non-blocking whereas time.sleep() is, this is not good as it wastes time between computation, unless of course we want to block for movement and computation...

# ledpin = 12				# PWM pin connected to LED
# GPIO.setwarnings(False)			#disable warnings
# GPIO.setmode(GPIO.BOARD)		#set pin numbering system
# GPIO.setup(ledpin,GPIO.OUT)
# pi_pwm = GPIO.PWM(ledpin,1000)		#create PWM instance with frequency
# pi_pwm.start(0)				#start PWM of required Duty Cycle 
# while True:
#     for duty in range(0,101,1):
#         pi_pwm.ChangeDutyCycle(duty) #provide duty cycle in the range 0-100
#         time.sleep(0.01)
#     time.sleep(0.5)
    
#     for duty in range(100,-1,-1):
#         pi_pwm.ChangeDutyCycle(duty)
#         time.sleep(0.01)
#     time.sleep(0.5)
    
#     break

class Pin():
    def __init__(self, channel, IO, **kwargs):
        """
        Generic pin initialization.

        Parameters:
        channel: integer, the pin assignment on the pi according to GPIO.BOARD
        IO: GPIO.IN | GPIO.OUT, the mode of the pin
        **kwargs: keyword arguments for the pins, vary on pin mode
            GPIO.IN: pull_up_down=GPIO.PUD_UP|GPIO.PUD_DOWN
            GPIO.OUT: initial=GPIO.HIGH|GPIO.LOW
        """
        self.channel = channel
        self.IO = IO
        GPIO.setup(channel, IO, **kwargs)

class GPIO(Pin):
    def __init__(self, channel, IO):
        """
        Initializes a GPIO pin. Passed to superclass.

        Parameters:
        channel: the pin assignment on the pi according to GPIO.BOARD
        IO: 
        """
        super().__init__(channel, IO)

    def drivePin(self, value):
        GPIO.output(self.channel, value)

class PWM(Pin):
    def __init__(self, channel, frequency=450):
        """
        Initializes a PWM pin. Passed to superclass.
        """
        super().__init__(channel, GPIO.OUT)
        self.pwm = GPIO.PWM(channel, frequency)
    
    def drivePWM(self, duration, dutycycle=50):
        """
        Drives the PWM pin.
        
        Parameters:
        self: instance of pin
        duration: int|float, time in seconds
        dutycycle=50: 0<float<100.0, dutycycle for pwm
        """
        self.pwm.start(dutycycle)
        time.sleep(duration)
        self.pwm.stop()

class MotorDriver:
    PWMA = 32
    AIN2 = 16
    AIN1 = 15
    STDBY = 13
    BIN1 = 18
    BIN2 = 22
    PWMB = 33
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        # Standby Pin
        self.STDBY = GPIO(13, GPIO.OUT, GPIO.HIGH) #TODO: Double check the Pull-up/pull down network for pi vs. arduino
        # Right Motor Pins
        self.PWMA = PWM(32)
        self.AIN2 = GPIO(16, GPIO.OUT)
        self.AIN1 = GPIO(15, GPIO.OUT)
        # Left Motor Pins
        self.BIN1 = GPIO(18, GPIO.OUT)
        self.BIN2 = GPIO(22, GPIO.OUT)
        self.PWMB = PWM(33)

    def driveRight(self, duration):
        self.AIN1.drivePin(GPIO.HIGH)
        self.AIN2.drivePin(GPIO.LOW)
        self.PWMA.drivePWM(duration)
    
    def driveLeft(self, duration):
        self.BIN1.drivePin(GPIO.HIGH)
        self.BIN2.drivePin(GPIO.LOW)
        self.PWMB.drivePWM(duration)
    
    def driveBoth(self, duration):
        self.AIN1.drivePin(GPIO.HIGH)
        self.AIN2.drivePin(GPIO.LOW)
        self.BIN1.drivePin(GPIO.HIGH)
        self.BIN2.drivePin(GPIO.LOW)
        rightMotor = threading.Thread(target=self.PWMA.drivePWM, args=(duration,))
        leftMotor = threading.Thread(target=self.PWMB.drivePWM, args=(duration,))
        rightMotor.start()
        leftMotor.start()
        rightMotor.join()
        leftMotor.join()

    def stop(self):
        pass
