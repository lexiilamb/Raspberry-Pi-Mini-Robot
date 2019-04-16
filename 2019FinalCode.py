#Libraries
import RPi.GPIO as GPIO
import time

#time.sleep(15)

# Set threshold to 20 cm
threshold = 40

# Turn execution time
turnTime = .1
# Forward and backward execution time
linearTime = .1

# Set GPIO Pins
# Left ultra sonic sensor 
TRIGGER1 = 4
ECHO1 = 17
# Middle ultra sonic sensor 
TRIGGER2 = 18
ECHO2 = 27
# Right ultra sonic sensor
TRIGGER3 = 23
ECHO3 = 24
# Motors (continuous rotation servos)
RIGHT_MOTOR = 16
LEFT_MOTOR = 21

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO direction (IN / OUT)
# Left sensor GPIOs
GPIO.setup(TRIGGER1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
# Middle sensor GPIOs
GPIO.setup(TRIGGER2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
# Right sensor GPIOs
GPIO.setup(TRIGGER3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)
# Motor GPIOs
GPIO.setup(RIGHT_MOTOR, GPIO.OUT)
GPIO.setup(LEFT_MOTOR, GPIO.OUT)

# Set PWN on pin at 100Hz
pwm = GPIO.PWM(RIGHT_MOTOR, 100)
pwm1 = GPIO.PWM(LEFT_MOTOR, 100)
# Start with 0 duty cycle
pwm.start(5)
pwm1.start(5)

def updateR(angle):
    duty = float(angle) / 10.0 + 2.5
    pwm.ChangeDutyCycle(duty)

def updateL(angle):
    duty = float(angle) / 10.0 + 2.5
    pwm1.ChangeDutyCycle(duty)

def leftDistance():
    # set Trigger to HIGH
    GPIO.output(TRIGGER1, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER1, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO1) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO1) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def middleDistance():
    # set Trigger to HIGH
    GPIO.output(TRIGGER2, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER2, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO2) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO2) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

def rightDistance():
    # set Trigger to HIGH
    GPIO.output(TRIGGER3, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(TRIGGER3, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(ECHO3) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(ECHO3) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance
 
if __name__ == '__main__':
    try:
        while True:
            # Display all distances 
            print ("\n -------------------------------------------")
            dist = leftDistance()
            print ("Left = %.1f cm" % dist)

            #time.sleep(.25)
            
            dist2 = middleDistance()
            print ("Middle = %.1f cm" % dist2)

            #time.sleep(.25)
            
            dist3 = rightDistance()
            print ("Right = %.1f cm" % dist3)

            #time.sleep(.25)

            # If nothing in front of robot, move forward
            if(dist2 >= threshold):
                # Check side sensors 
                # If right sensor < threshold, turn left
                if(dist3 < 10):
                    # turn left
                    print ("Turning left")
                    updateR(50)
                    updateL(50)
                    time.sleep(turnTime)
                # If left sensor < threshold, turn right
                elif(dist < 10):
                    # turn right
                    print ("Turning right")
                    updateR(130)
                    updateL(130)
                    time.sleep(turnTime)
                else:
                    print ("Moving forward")
                    # Forward
                    updateR(50)
                    updateL(130)
                    time.sleep(linearTime)
            # Back up and turn if all sensors detect object
            elif((dist < threshold) and (dist2 < threshold) and (dist3 < threshold)):
                # backward
                print ("Backing up")
                updateR(130)
                updateL(50)
                time.sleep(linearTime)
                # Pick direction to turn
                # If left sensor is greater than right sensor, turn left
                if(dist > dist3):
                    # turn left
                    print ("Turning left")
                    updateR(50)
                    updateL(50)
                    time.sleep(turnTime)
                # If right sensor is greater than left sensor, turn right
                else:
                    # turn right
                    print ("Turning right")
                    updateR(130)
                    updateL(130)
                    time.sleep(turnTime)             
            # If left sensor is greater than right sensor, turn left
            elif(dist > dist3):
                # turn left
                print ("Turning left")
                updateR(50)
                updateL(50)
                time.sleep(turnTime)
            # If right sensor is greater than left sensor, turn right
            else:
                # turn right
                print ("Turning right")
                updateR(130)
                updateL(130)
                time.sleep(turnTime)
                
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.output(RIGHT_MOTOR, False)
        GPIO.output(LEFT_MOTOR, False)
        pwm.ChangeDutyCycle(0)
        pwm1.ChangeDutyCycle(0) 

        pwm.stop()
        pwm1.stop()
        GPIO.cleanup()
