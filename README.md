# oradar_ws
Autonomous_rover 2.0
## Initial Runt Rover Arduino Tests July 11th
![Runt Rover Motor Configuration](/home/jetson/Downloads/20230711_174701.jpg)

The test arduino code for the runt rover will move each motor seperately to verify that the wiring is correct.
```arduino


void setup() {
  // put your setup code here, to run once:
  // Set motor control pins as output
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);

  
}


void loop() {
  // put your main code here, to run repeatedly:
//  INB controls the direction, INA controls the speed
//motor 1 control
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);            //spin motor 1 forward
  digitalWrite(2, HIGH); //pin 2 is direction on motor controller 1
  digitalWrite(3, HIGH); //pin 3 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);
  digitalWrite(2, LOW); //spin motor 1 back
  digitalWrite(3, HIGH);
  delay(1000);
  digitalWrite(2, LOW); //stop motor 1
  digitalWrite(3, LOW);
  delay(1000);
//motor 2 control
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);            //spin motor 2 forward
  digitalWrite(4, HIGH); //pin 4 is direction on motor controller 1
  digitalWrite(5, HIGH); //pin 5 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);
  digitalWrite(4, LOW); //spin motor 2 back
  digitalWrite(5, HIGH);
  delay(1000);
  digitalWrite(4, LOW); //stop motor 2
  digitalWrite(5, LOW);
  delay(1000);
//motor 3 control
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);            //spin motor 3 forward
  digitalWrite(6, HIGH); //pin 6 is direction on motor controller 1
  digitalWrite(7, HIGH); //pin 7 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(6, LOW); //spin motor 3 back
  digitalWrite(7, HIGH);
  delay(1000);
  digitalWrite(6, LOW); //stop motor 3
  digitalWrite(7, LOW);
  delay(1000);
//motor 4 control
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);            //spin motor 4 forward
  digitalWrite(8, HIGH); //pin 8 is direction on motor controller 1
  digitalWrite(9, HIGH); //pin 9 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);
  digitalWrite(8, LOW); //spin motor 4 back
  digitalWrite(9, HIGH);
  delay(1000);
  digitalWrite(8, LOW); //stop motor 4
  digitalWrite(9, LOW);
  delay(1000);
//motor 5 control
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);            //spin motor 5 forward
  digitalWrite(10, HIGH); //pin 10 is direction on motor controller 1
  digitalWrite(11, HIGH); //pin 11 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);
  digitalWrite(10, LOW); //spin motor 5 back
  digitalWrite(11, HIGH);
  delay(1000);
  digitalWrite(10, LOW); //stop motor 5
  digitalWrite(11, LOW);
  delay(1000);
//motor 6 control
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);            //spin motor 6 forward
  digitalWrite(12, HIGH); //pin 12 is direction on motor controller 1
  digitalWrite(13, HIGH); //pin 13 is the enable pin on the motor controller
  delay(1000);
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);
  digitalWrite(12, LOW); //spin motor 6 back
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(12, LOW); //stop motor 6
  digitalWrite(13, LOW);
  delay(1000);
 }
```
the code with functions for turning the rover is listed below. This is simple tank drive where turning is moving one side forward and the other side backwards. Maybe a turn by only turning one side would be more appropriate

```


void setup() {
  // put your setup code here, to run once:
  // Set motor control pins as output
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
  digitalWrite(12, LOW);

  
}

void leftside_forward(){
  digitalWrite(2, HIGH); //pin 2 is direction on motor controller 1
  digitalWrite(3, HIGH); //pin 3 is the enable pin on the motor controller
  digitalWrite(4, HIGH); //pin 4 is direction on motor controller 2
  digitalWrite(5, HIGH); //pin 5 is the enable pin on the motor controller
  digitalWrite(6, HIGH); //pin 6 is direction on motor controller 3
  digitalWrite(7, HIGH); //pin 7 is the enable pin on the motor controller
}

void leftside_back(){
  digitalWrite(2, LOW); //pin 2 is direction on motor controller 1
  digitalWrite(3, HIGH); //pin 3 is the enable pin on the motor controller
  digitalWrite(4, LOW); //pin 4 is direction on motor controller 2
  digitalWrite(5, HIGH); //pin 5 is the enable pin on the motor controller
  digitalWrite(6, LOW); //pin 6 is direction on motor controller 3
  digitalWrite(7, HIGH); //pin 7 is the enable pin on the motor controller
}

void leftside_stop(){
  digitalWrite(2, LOW); //pin 2 is direction on motor controller 1
  digitalWrite(3, LOW); //pin 3 is the enable pin on the motor controller
  digitalWrite(4, LOW); //pin 4 is direction on motor controller 2
  digitalWrite(5, LOW); //pin 5 is the enable pin on the motor controller
  digitalWrite(6, LOW); //pin 6 is direction on motor controller 3
  digitalWrite(7, LOW); //pin 7 is the enable pin on the motor controller
}

void rightside_forward(){
  digitalWrite(8, HIGH); //pin 2 is direction on motor controller 1
  digitalWrite(9, HIGH); //pin 3 is the enable pin on the motor controller
  digitalWrite(10, HIGH); //pin 4 is direction on motor controller 2
  digitalWrite(11, HIGH); //pin 5 is the enable pin on the motor controller
  digitalWrite(12, HIGH); //pin 6 is direction on motor controller 3
  digitalWrite(13, HIGH); //pin 7 is the enable pin on the motor controller
}

void rightside_back(){
  digitalWrite(8, LOW); //pin 2 is direction on motor controller 1
  digitalWrite(9, HIGH); //pin 3 is the enable pin on the motor controller
  digitalWrite(10, LOW); //pin 4 is direction on motor controller 2
  digitalWrite(11, HIGH); //pin 5 is the enable pin on the motor controller
  digitalWrite(12, LOW); //pin 6 is direction on motor controller 3
  digitalWrite(13, HIGH); //pin 7 is the enable pin on the motor controller
}

void rightside_stop(){
  digitalWrite(8, LOW); //pin 2 is direction on motor controller 1
  digitalWrite(9, LOW); //pin 3 is the enable pin on the motor controller
  digitalWrite(10, LOW); //pin 4 is direction on motor controller 2
  digitalWrite(11, LOW); //pin 5 is the enable pin on the motor controller
  digitalWrite(12, LOW); //pin 6 is direction on motor controller 3
  digitalWrite(13, LOW); //pin 7 is the enable pin on the motor controller
}

void forward(){
  rightside_forward();
  leftside_forward();
}

void back(){
  rightside_back();
  leftside_back();
}

void spinleft(){
  rightside_back();
  leftside_forward();
}

void spinright(){
  rightside_forward();
  leftside_back();
}

void stopspin(){
  leftside_stop();
  rightside_stop();
}

void loop() {
  //rightside movement
  forward();
  delay(1000);
  stopspin();
  delay(1000);
  back();
  delay(1000);
  stopspin();
  delay(1000);
  spinleft();
  delay(1000);
  stopspin();
  delay(1000);
  spinright();
  delay(1000);
  stopspin();
  delay(1000);
 }
```
# lidar_lisener.py
this python file is the main file used to process lidar data. It currently calculates the distance to the nearest object at each angle index and provides a score for each direction based on the distance and angle towards target.
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

# Set up plot
fig, ax = plt.subplots()

# Flag to enable or disable plotting
PLOT_ENABLED_distance = False
PLOT_ENABLED_SCORE = True
ROBOT_WIDTH =  .4
LIDAR_FOV = 360
LIDAR_RESOLUTION =  .8 # I found this by knowing 360 degrees/approx 450 data points per 1 rotation
LIDAR_MAX_RANGE = 12 #meters  # Maximum reliable range of the LIDAR

# Calculate the number of Lidar indices that represent the robot's width
ROBOT_WIDTH_INDICES = int((ROBOT_WIDTH / (2 * np.pi * LIDAR_MAX_RANGE)) * LIDAR_FOV / LIDAR_RESOLUTION)

def plot_data(data):
    global ax
    plt.cla()  # Clear the old plot
    ax.bar(range(len(data.ranges)), data.ranges)  # Create a new plot
    plt.draw()  # Update the plot
    plt.pause(0.001)  # Needed to update the plot
    
def plot_scores(scores):
    global ax
    plt.cla()  # Clear the old plot
    angles = np.linspace(0, LIDAR_FOV, len(scores))  # Calculate the corresponding angle for each score
    ax.plot(angles, scores)  # Create a new plot
    ax.set_xlabel('Angle (degrees)')  # Set x-axis label
    ax.set_ylabel('Score')  # Set y-axis label
    plt.draw()  # Update the plot
    plt.pause(0.001)  # Needed to update the plot


def callback(data):
    global PLOT_ENABLED
    # Get the current time
    current_time = rospy.get_time()
    # Check if one second has passed since the last update
    if PLOT_ENABLED_distance and current_time - callback.last_update_time >= 1.0:
        plot_data(data)
        # Update the last update time
        callback.last_update_time = current_time

    # Calculate free space considering the robot's width
    free_space = [min(data.ranges[i:i+ROBOT_WIDTH_INDICES]) for i in range(len(data.ranges) - ROBOT_WIDTH_INDICES + 1)]

    # Calculate a score for each direction
    scores = []
    for i, distance in enumerate(free_space):
        if distance > ROBOT_WIDTH:
            angle_difference = abs(i - len(free_space) // 2)  # The angle difference to the target direction
            score = distance / (1 + angle_difference)  # The score is higher for larger distances and smaller angle differences
            scores.append(score)
        else:
            scores.append(0)  # If there is not enough space for the robot, the score is 0

    # Find the direction with the highest score
    best_direction = scores.index(max(scores))
    
    if PLOT_ENABLED_SCORE and current_time - callback.last_update_time >= 1.0:
        plot_scores(scores)
        # Update the last update time
        callback.last_update_time = current_time

    # TODO: Command the robot to move in the best direction


def listener():
    rospy.init_node('lidar_plot', anonymous=True)
    # Initialize the last update time to the current time
    callback.last_update_time = rospy.get_time()
    rospy.Subscriber("/MS200/scan", LaserScan, callback, queue_size=1)
    plt.show(block=True)  # Begin matplotlib event loop

if __name__ == '__main__':
    listener()
```
