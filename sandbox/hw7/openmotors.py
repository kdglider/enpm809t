import RPi.GPIO as gpio


##### Initialize GPIO pins ####
def init():
	gpio.setmode(gpio.BOARD)
	gpio.setup(31,gpio.OUT)  #IN1
 	gpio.setup(33, gpio.OUT) #IN2
 	gpio.setup(35, gpio.out) #IN3
 	gpio.setup(37, gpio.out) #IN4

 	gpio.setup(12, gpio.IN, pull_up_down = gpio.PUD_UP)

def gameover():
	gpio.output(31, FALSE)
	gpio.output(33, FALSE)
	gpio.output(35, FALSE)
	gpio.output(37, FALSE)
	gpio.cleanup()


init()
gameover()