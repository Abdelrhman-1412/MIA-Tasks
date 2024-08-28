#!/usr/bin/env python3
import rospy
import time
import math
import subprocess
import sys
from turtlesim.msg import Pose
from std_msgs.msg import String, Bool
from std_srvs.srv import Empty, EmptyResponse
from turtlesim.srv import Kill

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')

        self.turtles = {}  # Dictionary to store turtle states (health, attacks, pose)
        self.start_game_pub = rospy.Publisher('/game_started', Bool, queue_size=1)
        self.start_game_pub.publish(False)
        self.attack_sub = rospy.Subscriber('/turtle_attack', String, self.attack_callback)
        self.startgame_service = rospy.Service('/start_game', Empty, self.startgame_callback)
        self.game_started = False
        self.rate = rospy.Rate(10)  # Loop at 10 Hz

        self.start_turtlesim_node()  # Start turtlesim node
        self.subscribe_turtles()
        rospy.loginfo("Game Engine initialized.")

    def start_turtlesim_node(self):
        rospy.loginfo("Starting turtlesim_node...")
        # List of terminal emulators to try
        terminals = [
            ['gnome-terminal', '--', 'rosrun', 'turtlesim', 'turtlesim_node'],
            ['xterm', '-e', 'rosrun', 'turtlesim', 'turtlesim_node'],
            ['konsole', '-e', 'rosrun', 'turtlesim', 'turtlesim_node'],
        ]
        max_retries = 3
        for attempt in range(max_retries):
            rospy.loginfo(f"Attempt {attempt + 1} of {max_retries}...")
            terminal = terminals[attempt]
            try:
                subprocess.Popen(terminal)
                time.sleep(2)  # Wait for turtlesim to start
                rospy.loginfo(f"turtlesim_node started successfully using {terminal[0]}.")
                self.remove_turtle('turtle1')
                return  # Exit after successful start
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                rospy.logwarn(f"Failed to start turtlesim_node with {terminal[0]}. Error: {e}")      
            rospy.logwarn("Retrying with the next terminal emulator...")

        rospy.logerr("Failed to start turtlesim_node after all attempts. Exiting game.")
        rospy.loginfo("Ending game...")
        rospy.logerr("Consider installing a terminal emulator like 'gnome-terminal', 'xterm', or 'konsole' to run ROS nodes.")
        rospy.loginfo("You can install one of these with the following commands:")
        rospy.loginfo("sudo apt-get install gnome-terminal")
        rospy.loginfo("sudo apt-get install xterm")
        rospy.loginfo("sudo apt-get install konsole")
        rospy.signal_shutdown("Game ended due to failure to start turtlesim_node.")
        sys.exit("--> Game Engine Closed <--")
            
    def subscribe_turtles(self):
        topics = rospy.get_published_topics()
        for topic, _ in topics:
            if '/pose' in topic:
                turtle_name = topic.split('/')[1]
                if turtle_name not in self.turtles:
                    # Initialize turtle state
                    self.turtles[turtle_name] = {'health': 100, 'attacks': 10, 'pose': None}
                    rospy.Subscriber(topic, Pose, self.update_turtle_pose, callback_args=turtle_name)
                    rospy.loginfo(f"Connected to {turtle_name}")

    def update_turtle_pose(self, data, turtle_name):
        self.turtles[turtle_name]['pose'] = data

    def attack_callback(self, msg: String):
        if not self.game_started:
            return
        
        attacker = msg.data
        rospy.loginfo(f"{attacker} performed an attack!")
        self.calculate_damage(attacker)

    def calculate_damage(self, attacker):
        for target, state in self.turtles.items(): 
            if target != attacker and self.turtles[attacker]['pose'] and state['pose']:
                if self.attack_within_distance(self.turtles[attacker]['pose'], state['pose']):
                    self.process_attack(attacker, target)
        
    def attack_within_distance(self, pose1, pose2):
        distance = math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
        return distance <= 2.0

    def process_attack(self, attacker, target):
        if self.turtles[attacker]['attacks'] > 0:
            self.turtles[target]['health'] -= 50
            self.turtles[attacker]['attacks'] -= 1
            rospy.loginfo(f"{attacker} attacked {target}!")
            if self.turtles[target]['health'] <= 0:
                rospy.loginfo(f"{target} has been defeated!")
                self.turtles.pop(target)
                self.remove_turtle(target)
                rospy.loginfo(f"{target} has been removed from the game.")

    def remove_turtle(self, turt):
        try:
            # Create a service proxy to the /kill service
            kill_turtle = rospy.ServiceProxy('/kill', Kill)
            # Wait for the service to be available
            rospy.wait_for_service('/kill')
            # Call the service with the name of the turtle to be killed
            kill_turtle(turt)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    
    def check_game_over(self):
        all_attacks_used = all(t['attacks'] == 0 for t in self.turtles.values())

        if all_attacks_used or len(self.turtles) == 1:
            max_health = max(self.turtles[t]['health'] for t in self.turtles)
            winners = [t for t in self.turtles if self.turtles[t]['health'] == max_health]

            if len(winners) == 1:
                winner_message = f"{winners[0]} wins!"
            else:
                winner_message = "It's a draw!"

            rospy.loginfo(winner_message)
            rospy.signal_shutdown("Game Over")

    def startgame_callback(self, request):
        self.game_started = True
        self.start_game_pub.publish(True)
        rospy.loginfo("Game is Starting...")
        rospy.loginfo("3"); rospy.sleep(1); rospy.loginfo("2"); rospy.sleep(1); rospy.loginfo("1"); rospy.sleep(1)
        rospy.loginfo("GO!")
        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            if self.game_started:
                self.check_game_over()

if __name__ == '__main__':
        engine = GameEngine()
        engine.run()

