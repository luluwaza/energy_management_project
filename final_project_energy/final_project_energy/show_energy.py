import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import threading
import matplotlib.pyplot as plt


class ShowEnergy(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        qos_profile2 = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        super().__init__("ShowEnergy")
        self.battery_percentage = 0
        self.msg = []
        self.subscription = self.create_subscription(BatteryState, "/battery_state", self.get_battery_percentage, 10)
        self.subscription2 = self.create_subscription(LaserScan, "/scan", self.get_laser_scan, qos_profile)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", qos_profile2)

        self.max_battery_voltage = 12.4
        self.min_battery_voltage = 11.0
        self.battery_voltage_difference = self.max_battery_voltage - self.min_battery_voltage

        self.current_speed = 0.01
        self.speed_difference = 0.01
        self.iterations_amount = 21

        self.turning_speed = 0.1
        self.turning_distance = 0.5

        self.battery_usages = {}
        self.battery_percentages = []
        self.elapsed_times = []

        self.amount_of_turns = 0
        self.was_space = False

        plt.plot(self.battery_percentages, marker="o")
        plt.title("Battery Percentage Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Battery Percentage (%)")
        plt.ylim(0, 100)
        plt.grid(True)

        # create a thread to run the test and still having the subcribers work in the background
        self.test_thread = threading.Thread(target=self.run_test)
        self.test_thread.start()

    def run_test(self):
        self.stop_robot()
        self.current_speed = float(
            input("Enter the starting speed (0.01 for starting a complete test on the turtlebot3): ")
        )
        self.speed_difference = float(
            input("Enter the speed difference (0.01 for starting a complete test on the turtlebot3): ")
        )
        self.current_speed -= self.speed_difference
        self.iterations_amount = int(
            input("Enter the amount of iterations (21 for starting a complete test on the turtlebot3): ")
        )
        amount_of_turns_per_speed = int(input("Enter the amount of turns the robot should do per speed: "))
        self.was_space = self.space_on_left()
        self.get_logger().info("Starting test in 5 seconds")
        for i in range(5):
            print(f"{5 - i}" + "." * i, end="\r")
            time.sleep(1)

        self.get_logger().info("=============")
        self.get_logger().info("Test started")
        self.get_logger().info("=============")
        for i in range(self.iterations_amount):
            self.current_speed += self.speed_difference
            self.old_battery_percentage = self.battery_percentage
            self.get_logger().info(f"Starting test with speed: {self.current_speed:.2f}")
            start_time = time.time()

            while self.amount_of_turns < amount_of_turns_per_speed:
                if self.obstacle_detected():
                    self.avoid_obstacle()
                else:
                    self.navigate()

            end_time = time.time()
            self.elapsed_time = end_time - start_time
            self.elapsed_times += [self.elapsed_time]
            self.stop_robot()
            self.amount_of_turns = 0

            self.display_battery_tests_results()
            self.display_speed_tests_results()
            if i < self.iterations_amount - 1:
                self.get_logger().info(f"Test finished for this speed in {self.elapsed_time}s")
                # TODO wait for the user to put a hand in front of the lidar instead of waiting 10 seconds
                for i in range(10):
                    print(f"The test will start again in {10 - i} seconds", end="\r")

        most_efficient_speed = min(self.battery_usages, key=self.battery_usages.get)
        self.get_logger().info(f"The most efficient speed is: {most_efficient_speed:.2f}")
        self.get_logger().info("Test finished, press ctrl + c to exit the program")

    def get_battery_percentage(self, msg):
        self.battery_percentage = (msg.voltage - self.min_battery_voltage) / self.battery_voltage_difference * 100
        self.battery_percentages.append(self.battery_percentage)
        self.show_graph()

    def show_graph(self):
        plt.plot(self.battery_percentages)
        plt.pause(0.01)

    def get_laser_scan(self, msg):
        self.msg = msg.ranges

    def space_on_left(self):
        min_degree = int(75 / 360 * len(self.msg))
        max_degree = int(105 / 360 * len(self.msg))
        return (
            min(self.msg[min_degree:max_degree]) > self.turning_distance
            and min(self.msg[(min_degree * 2) : (max_degree * 2)]) > 0.25
        )

    def turn_left(self):
        self.amount_of_turns += 1
        self.move(0.0, self.turning_speed, 15.708)  # pi / 4 = 90 degrees (* 5 because 0.1 speed and not 1.0)

    def move(self, linear_speed, angular_speed, duration):
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        self.publisher.publish(msg)
        time.sleep(duration)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)

    def obstacle_detected(self):
        degrees = int(30 / 360 * len(self.msg))
        forward_lidar = self.msg[:degrees] + self.msg[-degrees:]
        return min(forward_lidar) < 0.22

    def avoid_obstacle(self):
        self.get_logger().info("Avoiding obstacle")

        degrees = int(30 / 360 * len(self.msg))
        forward_left_lidar = self.msg[:degrees]
        forward_right_lidar = self.msg[-degrees:]

        msg = Twist()
        if min(forward_left_lidar) < min(forward_right_lidar):
            msg.linear.x = 0.0
            msg.angular.z = -self.turning_speed
        else:
            msg.linear.x = 0.0
            msg.angular.z = self.turning_speed

        while self.obstacle_detected():
            self.publisher.publish(msg)

    def navigate(self):
        if self.space_on_left() and not self.was_space:
            self.was_space = True
            self.get_logger().info("Turning left")
            self.turn_left()
        elif self.space_on_left():
            msg = Twist()
            msg.angular.z = 0.0
            msg.linear.x = self.current_speed
            self.publisher.publish(msg)
        else:
            if self.msg[90] < self.turning_distance:
                self.was_space = False
            msg = Twist()
            msg.angular.z = 0.0
            msg.linear.x = self.current_speed
            self.publisher.publish(msg)

    def stop_robot(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = 0.0
        self.publisher.publish(msg)

    def display_battery_tests_results(self):
        self.get_logger().info("===========================================")
        self.get_logger().info(f"Old battery percentage for this speed: {self.old_battery_percentage}")
        self.get_logger().info(f"New battery percentage for this speed: {self.battery_percentage}")
        self.battery_usages[self.current_speed] = self.old_battery_percentage - self.battery_percentage

        self.get_logger().info("Battery usage per speed:")
        for key, value in self.battery_usages.items():
            self.get_logger().info(f"{key:.2f}: {value}%")
        self.get_logger().info("===========================================")

    def display_speed_tests_results(self):
        self.get_logger().info(f"Elapsed time for this speed: {self.elapsed_time}s")
        self.get_logger().info("Elapsed time per speed:")
        for i in range(len(self.elapsed_times)):
            speed = self.current_speed - self.speed_difference * (self.iterations_amount - i - 1)
            elapsed_time = self.elapsed_times[i]
            self.get_logger().info(f"{speed:.2f}: {elapsed_time:.2f}s")
        self.get_logger().info("===========================================")


def main(args=None):
    rclpy.init(args=args)
    node = ShowEnergy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
