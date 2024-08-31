#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry

class CarController:
    def __init__(self):
        rospy.init_node('car_controller', anonymous=True)
        self.pub_SteeringAngle = rospy.Publisher('SteeringAngle', Float64, queue_size=10)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Float64, queue_size=10)
        self.pub_brakes = rospy.Publisher('brakes', Float64, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.brake_value =1.6*pow(10, -47.2)
        self.Kp = 0.79
        self.Ki = 0.005
        self.Kd = 5
        self.set_point = 75.0
        self.error = 0.0
        self.integral = 0.0
        self.derivative = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.last_time = rospy.get_time()
        
        self.current_distance = 0.0

    def start_simulation(self):
        """Publishes a command to start the simulation."""
        simulation_pub = rospy.Publisher('/startSimulation', Bool, queue_size=1)
        while simulation_pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        simulation_pub.publish(True)
        rospy.loginfo("Simulation started by node")

    def compute_pid(self):
        """Calculate PID value for given reference feedback."""
        current_time = rospy.get_time()
        dt = current_time - self.last_time if self.last_time != 0 else 0.1
        self.error = self.set_point - self.current_distance
        self.integral += self.error * dt
        self.derivative = (self.error - self.last_error) / dt if dt > 0 else 0

        self.output = self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative
        self.last_error = self.error
        self.last_time = current_time
        return self.output

    def odom_callback(self, msg):
        """Updates the current distance based on odometry data."""
        self.current_distance = msg.pose.pose.position.y

    def run(self):
        """Main method to control the car using PID."""
        self.start_simulation()
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            velocity = self.compute_pid()
            print(velocity,"   ",self.current_distance)
            if velocity > 1.0:
                velocity = 1.0
                self.pub_brakes.publish(0)
            elif velocity < 0.0:
                velocity = 0.0
                self.pub_brakes.publish(self.brake_value)

            self.pub_cmd_vel.publish(velocity)
            print(velocity)
            self.pub_SteeringAngle.publish(0.0)  # Assuming straight line movement

            if self.current_distance >= self.set_point:
                rospy.loginfo("Target distance reached. Stopping.")
                self.pub_cmd_vel.publish(0.0)
                self.pub_brakes.publish(self.brake_value)
                print("done")
                break
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = CarController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node was interrupted")
