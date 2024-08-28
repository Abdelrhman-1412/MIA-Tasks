#!/usr/bin/env python3
import rospy
import time
import subprocess
from turtlesim.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse
from turtlesim.srv import Kill


# Note: still need to: 1- handle collision attacks | 2- update status of each player accordingly | 3- handle winner 

class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')
        
        self.player_statuses = {} # [turtleID]: {'health': x, 'attacks_left': x}
        self.status_pub = rospy.Publisher('/game_status', Bool, queue_size=1)
        self.status_pub.publish(True) 
        
        self.turtle_subs = {}
        self.attack_sub = rospy.Subscriber('/turtle_attack', String, self.attack_callback)

        self.rate = rospy.Rate(10)
        rospy.loginfo("Game Engine has been created")

        self.start_turtlesim_node() # start turtlesim node

        self.startgame_service = rospy.Service('/start_game', Empty, self.startgame_callback)
        self.game_started = False
        self.game_lobby()
        self.run()
        
    def start_turtlesim_node(self):
        rospy.loginfo("Attempting to start turtlesim_node...")
        while not rospy.is_shutdown():
            try:
                # subprocess.Popen(['gnome-terminal', '--', 'rosrun', 'turtlesim', 'turtlesim_node'])
                subprocess.Popen(['xterm', '-e', 'rosrun turtlesim turtlesim_node'])
                time.sleep(2)  # Give it some time to start and initialize
            except subprocess.CalledProcessError as e:
                rospy.logwarn(f"Failed to start turtlesim_node. Retrying... Error: {e}")
                time.sleep(2)  # Wait before trying again
                continue

            rospy.loginfo("turtlesim_node started successfully.")
            rospy.wait_for_service('/kill')
            kill_turtle_service = rospy.ServiceProxy('/kill', Kill)
            kill_turtle_service("turtle1")
            break

    def game_lobby(self):
        rospy.loginfo("Waiting for players to join...")
        rospy.loginfo(f"Currently joined players: {[f'turtle{id}' for id in self.player_statuses.keys()]}")

        while not self.game_started:
            turtles = self.get_active_turtles() # Initialize status for all currently active turtles
            for turtle_name in turtles:
                player_id = int(turtle_name.replace('turtle', ''))
                if player_id not in self.player_statuses:
                    self.initialize_player_status(player_id)
                    self.update_turtle_subscriptions()  
                    rospy.loginfo(f"Currently joined players: {[f'turtle{id}' for id in self.player_statuses.keys()]}")
                    rospy.logwarn("Call /start_game service to start game.")

            rospy.sleep(1)  # 1 second delay then check again to prevent overload

    def startgame_callback(self, request):
        self.game_started = True
        self.status_pub.publish(False)
        rospy.loginfo("Game is Starting... No more players can join.")
        rospy.loginfo("3"); rospy.sleep(1); rospy.loginfo("2"); rospy.sleep(1); rospy.loginfo("1"); rospy.sleep(1)
        rospy.loginfo("GO!")
        self.set_initial_positions()
        return EmptyResponse()

    def get_active_turtles(self):
        # Retrieve all topics
        topics = rospy.get_published_topics() # list of [[topic, topic type], ...]
        turtle_topics = [str(topic) for topic, _ in topics if 'pose' in topic] # Filter topics to find turtle pose topics
        turtle_names = [topic.split('/')[1] for topic in turtle_topics] # Extract turtle names from topic names
        # rospy.loginfo(f"Active turtles: {turtle_names}") # remove later just displaying players for testing
        return turtle_names
    
    def update_turtle_subscriptions(self):
        turtles = self.get_active_turtles()
        for turtle_name in turtles:
            if turtle_name not in self.turtle_subs:
                self.turtle_subs[turtle_name] = rospy.Subscriber(f'/{turtle_name}/pose', Pose, self.pose_callback)

    def initialize_player_status(self, player_id):
        # Initialize status for a new player (turtle)
        if player_id not in self.player_statuses:
            self.player_statuses[player_id] = {
                'health': 100,  # Default initial health
                'attacks_left': 10  # Default initial attacks left
            }
            rospy.loginfo(f"turtle{player_id} joined the game!")
            self.publish_game_status()

    def publish_game_status(self, update_id = None, update_health = None, update_attacks = None):
             
        for player_id, status in self.player_statuses.items():

            # Check and cast types
            player_id = int(player_id)
            health = int(status['health'])
            attacks_left = int(status['attacks_left'])
            
            #check if update needed
            if update_id == player_id:
                if update_health != None:
                    health = update_health
                if update_attacks != None:
                    attacks_left = update_attacks

            self.player_statuses[player_id] = {
                'health': health,
                'attacks_left': attacks_left
            }

        status_message = "Game Status:\n"
        for player_id, status in self.player_statuses.items():
            player_name = f"turtle{player_id}"
            health = status['health']
            attacks_left = status['attacks_left']
            status_message += f"{player_name}: Health = {health}, Attacks Left = {attacks_left}\n"

        rospy.loginfo(status_message)

    def set_initial_positions(self):
        pass # to do

    # <-- CALLBACK FUNCTIONS -->
    def pose_callback(self, msg: Pose):
        if not self.game_started:
            pass
        else:
            return # to do

    def attack_callback(self, msg: String):
        if not self.game_started:
            pass
        else:
            
            turtle_name = msg.data
            rospy.loginfo(f"{turtle_name} performed an attack!") 
            # still need to implement attack logic
            # --> to do <--
            
            # for testing purpose
            _, player_id = turtle_name.split("turtle")
            player_id = int(player_id)
            curr_attacks = int(self.player_statuses[player_id]['attacks_left'])
            self.publish_game_status(update_id=player_id, update_attacks=curr_attacks-1) 
            

    def run(self):
        while not rospy.is_shutdown():
            if not self.game_started:
                rospy.logwarn("Game has not started yet.")
                self.rate.sleep()

if __name__ == '__main__':
    try:
        GameEngine()
    except rospy.ROSInterruptException:
        pass