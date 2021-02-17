#!/usr/bin/env python

# ROS Node - Action Server - IoT ROS Bridge

'''
**ROS Node - Action Server - IoT ROS Bridge**

Python Modules required for ROS client,Action Server,Threading
::

    import rospy
    import actionlib
    import threading

Message Class that is used by ROS Actions internally
::

    from pkg_ros_iot_bridge.msg import msgRosIotAction

Message Class that is used for Goal Messages
::

    from pkg_ros_iot_bridge.msg import msgRosIotGoal

Message Class that is used for Result Messages
::

    from pkg_ros_iot_bridge.msg import msgRosIotResult

Message Class that is used for Feedback Messages
::

    from pkg_ros_iot_bridge.msg import msgRosIotFeedback

Message Class for MQTT Subscription Messages
::

    from pkg_ros_iot_bridge.msg import msgMqttSub

Custom Python Module to perfrom MQTT Tasks
::

    from pyiot import iot

Python Module to make HTTP requests and to parse data from string
::

    import requests
    from ast import literal_eval'''

import rospy
import actionlib
import threading

# Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotAction

# Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotGoal

# Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult

# Message Class that is used for Feedback Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback

# Message Class for MQTT Subscription Messages
from pkg_ros_iot_bridge.msg import msgMqttSub

# Custom Python Module to perfrom MQTT Tasks
from pyiot import iot
import requests
from ast import literal_eval



class RosIotBridgeActionServer:
    '''
    This Class links ROS IoT and Action Server.

    Constructor

    Initialize the Action Server
    ::

        self._as = actionlib.ActionServer(
            "/action_ros_iot",
            msgRosIotAction,
            self.on_goal,
            self.on_cancel,
            auto_start=False,
        )

    * self.on_goal - It is the fuction pointer which points to a function which will be called
                        when the Action Server receives a Goal.

    * self.on_cancel - It is the fuction pointer which points to a function which will be called
                        when the Action Server receives a Cancel Request.

    Read and Store IoT Configuration data from Parameter Server
    ::

        param_config_iot = rospy.get_param("config_iot")
        self._config_mqtt_server_url = param_config_iot["mqtt"]["server_url"]
        self._config_mqtt_server_port = param_config_iot["mqtt"]["server_port"]
        self._config_mqtt_sub_topic = param_config_iot["mqtt"]["topic_sub"]
        self._config_mqtt_pub_topic = param_config_iot["mqtt"]["topic_pub"]
        self._config_mqtt_qos = param_config_iot["mqtt"]["qos"]
        self._config_mqtt_sub_cb_ros_topic = param_config_iot["mqtt"][
            "sub_cb_ros_topic"
        ]

    Initialize ROS Topic Publication
    Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
    ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
    ::   

        self._handle_ros_pub = rospy.Publisher(
            self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10
        )


    Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
    self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
    ::

        ret = iot.mqtt_subscribe_thread_start(
            self.mqtt_sub_callback,
            self._config_mqtt_server_url,
            self._config_mqtt_server_port,
            self._config_mqtt_sub_topic,
            self._config_mqtt_qos,
        )

    '''

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer(
            "/action_ros_iot",
            msgRosIotAction,
            self.on_goal,
            self.on_cancel,
            auto_start=False,
        )
        

        
        '''   * self.on_goal - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the fuction pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request. '''
        

        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param("config_iot")
        self._config_mqtt_server_url = param_config_iot["mqtt"]["server_url"]
        self._config_mqtt_server_port = param_config_iot["mqtt"]["server_port"]
        self._config_mqtt_sub_topic = param_config_iot["mqtt"]["topic_sub"]
        self._config_mqtt_pub_topic = param_config_iot["mqtt"]["topic_pub"]
        self._config_mqtt_qos = param_config_iot["mqtt"]["qos"]
        self._config_mqtt_sub_cb_ros_topic = param_config_iot["mqtt"][
            "sub_cb_ros_topic"
        ]
        print(param_config_iot)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(
            self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10
        )

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(
            self.mqtt_sub_callback,
            self._config_mqtt_server_url,
            self._config_mqtt_server_port,
            self._config_mqtt_sub_topic,
            self._config_mqtt_qos,
        )
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        '''
        This is a callback function for MQTT Subscriptions           

        Storing the Recieved Message in variable Payload
        ::

            payload = message.payload.decode("utf-8")

        Publish the received message to ``msg_mqtt_sub`` Message class
        ::

            msg_mqtt_sub = msgMqttSub()
            msg_mqtt_sub.timestamp = rospy.Time.now()
            msg_mqtt_sub.topic = message.topic
            msg_mqtt_sub.message = payload

            self._handle_ros_pub.publish(msg_mqtt_sub)

        Call IncomingOrders function to Update Incoming Order Spreadsheet
        ::

            self.IncomingOrders(payload)

        '''
    
        payload = message.payload.decode("utf-8")

        print("[MQTT SUB CB] Topic: ", message.topic)

        msg_mqtt_sub = msgMqttSub()
        msg_mqtt_sub.timestamp = rospy.Time.now()
        msg_mqtt_sub.topic = message.topic
        msg_mqtt_sub.message = payload

        self._handle_ros_pub.publish(msg_mqtt_sub)

        self.IncomingOrders(payload)

    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):

        '''
        This function will be called when Action Server receives a Goal

        Validate incoming goal parameters
        ::

            if goal.message:
                goal_handle.set_accepted()

        Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
        ''self.process_goal'' - is the function pointer which points to a function that will process incoming Goals
        ::

            thread = threading.Thread(name="worker", target=self.process_goal, args=(goal_handle,))
            thread.start()
            
        Executed if goal is invalid
        ::
        
            else:
                goal_handle.set_rejected()
                return
        '''
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)

        if goal.message:

            goal_handle.set_accepted()

            # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
            # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
            thread = threading.Thread(
                name="worker", target=self.process_goal, args=(goal_handle,)
            )
            thread.start()

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):
        '''
        This function is called is a separate thread to process Goal.

        Storing the result message from the message class `` msgRosIotResult`` in variable result
        ::

            result = msgRosIotResult()

        Extract the goal message from ``goal_handle`` and assign to the variable ``msg``
        ::

            goal_id = goal_handle.get_goal_id()
        

            goal = goal_handle.get_goal()
            msg = goal.message.decode("utf-8")
            msg = literal_eval(msg)

        Call the ``update_sheet`` function to update the respective sheets
        ::

            ret = self.update_sheet(msg)

        Using the ``ret`` (Return message) update the flag success in ``result``

            if ret == "1":
                rospy.loginfo("UpdateSheet Thread Started")
                result.flag_success = True
            else:
                rospy.logerr("Failed to start UpdateSheet Thread")
                result.flag_success = False

            rospy.loginfo("Send goal result to client")
            print("ret : ", ret)
            if result.flag_success == True:
                rospy.loginfo("Succeeded")
                goal_handle.set_succeeded(result)
            else:
                rospy.loginfo("Goal Failed. Aborting.")
                goal_handle.set_aborted(result)
      


        '''
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()
        msg = goal.message.decode("utf-8")
        msg = literal_eval(msg)

        rospy.logwarn("Recieved  Goal ID: " + str(goal_id.id))

        ret = self.update_sheet(msg)

        if ret == "1":
            rospy.loginfo("UpdateSheet Thread Started")
            result.flag_success = True
        else:
            rospy.logerr("Failed to start UpdateSheet Thread")
            result.flag_success = False

        rospy.loginfo("Send goal result to client")
        print("ret : ", ret)
        if result.flag_success == True:
            rospy.loginfo("Succeeded")
            goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        '''
        This function will be called when Goal Cancel request is send to the Action Server
        '''
        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Received cancel request for Goal ID:" + str(goal_id.id))

    def update_sheet(self, msg):
        '''
        This function Updates the Message received from other ROS nodes through Action Server to the Respective Sheet ID of IMS

        :Param msg: A dictionary containing parameters
        :Type msg: Dictionary
        :return: if success "1"

        Webapp URL 
        ::

            URL1 = "https://script.google.com/macros/s/AKfycbyvZvFADAkusgdLFqvQAuKMT5GweDrY06mp0AtUi70Mamr2ESM/exec"

        Response message
        ::

            response1 = requests.get(URL1, params=msg)

        Return
        ::

            return response1.content
        '''
        URL1 = "https://script.google.com/macros/s/AKfycbyvZvFADAkusgdLFqvQAuKMT5GweDrY06mp0AtUi70Mamr2ESM/exec"
        response1 = requests.get(URL1, params=msg)

        return response1.content

    def IncomingOrders(self, payload):
        '''
        This function Updates the Incoming order spreadsheet with the given parameters

        Converting string to dictionary
        ::
            payload = literal_eval(payload)

        Assigning priority & cost based on ``item``
        ::

            item = payload["item"]

            if item == "Medicine":
                priority = "HP"
                cost = "1000"

            elif item == "Food":
                priority = "MP"
                cost = "500"

            elif item == "Clothes":
                priority = "LP"
                cost = "100"

        Updating Inventory sheet with ``parameters``
        ::
            parameters = {
                "id": "IncomingOrders",
                "Team Id": "VB#1516",
                "Unique Id": "aYzqLq",
                "Order Id": payload["order_id"],
                "Order Date and Time": payload["order_time"],
                "Order Id": payload["order_id"],
                "Item": item,
                "Priority": priority,
                "Cost": cost,
                "Order Quantity": payload["qty"],
                "City": payload["city"],
                "Longitude": payload["lon"],
                "Latitude": payload["lat"],
            }

            self.update_sheet(parameters)
  

        '''
        payload = literal_eval(payload)
        item = payload["item"]

        if item == "Medicine":
            priority = "HP"
            cost = "1000"

        elif item == "Food":
            priority = "MP"
            cost = "500"

        elif item == "Clothes":
            priority = "LP"
            cost = "100"

        # Updating Inventory sheet
        parameters = {
            "id": "IncomingOrders",
            "Team Id": "VB#1516",
            "Unique Id": "aYzqLq",
            "Order Id": payload["order_id"],
            "Order Date and Time": payload["order_time"],
            "Order Id": payload["order_id"],
            "Item": item,
            "Priority": priority,
            "Cost": cost,
            "Order Quantity": payload["qty"],
            "City": payload["city"],
            "Longitude": payload["lon"],
            "Latitude": payload["lat"],
        }

        self.update_sheet(parameters)


def main():
    '''
    Initialize the node
    ::

        rospy.init_node("node_ros_iot_bridge_action_server")

    Creating class object
    ::
        
        ros_iot_bridge = RosIotBridgeActionServer()

    Run the node until interrupt ``ctrl + c``
    ::

        rospy.spin()

    '''
    rospy.init_node("node_ros_iot_bridge_action_server")

    ros_iot_bridge = RosIotBridgeActionServer()

    rospy.spin()


if __name__ == "__main__":
    main()
