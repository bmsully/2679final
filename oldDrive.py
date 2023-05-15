import RPi.GPIO as GPIO
import time

def setup():
	GPIO.setwarnings(False)
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(32, GPIO.OUT) # PWMA
	GPIO.setup(16, GPIO.OUT) # AIN2
	GPIO.setup(15, GPIO.OUT) # AIN1
	GPIO.setup(13, GPIO.OUT, initial=GPIO.HIGH) # STBY
	GPIO.setup(18, GPIO.OUT) # BIN1
	GPIO.setup(22, GPIO.OUT) # BIN2
	GPIO.setup(33, GPIO.OUT) # PWMB

if __name__ == "__main__":
	try:
		setup()
		pwma = GPIO.PWM(32, 450)
		pwmb = GPIO.PWM(33, 450)
		pwma.start(50)
		pwmb.start(50)
		GPIO.output(16, 1)
		GPIO.output(15, 0)
		GPIO.output(18, 1)
		GPIO.output(22, 0)
		time.sleep(1)
		pwma.stop()
		pwmb.stop()
	finally:
		GPIO.cleanup()
