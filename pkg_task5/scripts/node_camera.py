#!/usr/bin/env python
import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from pyzbar.pyzbar import decode
from node_iot_ros_bridge_action_client import RosIotBridgeActionClient
import datetime


class Camera1(RosIotBridgeActionClient):
    def __init__(self):
        self._action_client = RosIotBridgeActionClient()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/eyrc/vb/camera_1/image_raw", Image, self.callback
        )

        self._process_fineshed = False

    def get_qr_data(self, arg_image, pkg):

        # Points for respective box to focus image for particular box in shelf
        packages = {
            "packagen00": [(124, 311), (238, 415)],
            "packagen01": [(308, 305), (412, 420)],
            "packagen02": [(470, 303), (603, 420)],
            "packagen10": [(117, 483), (243, 595)],
            "packagen11": [(307, 483), (413, 593)],
            "packagen12": [(474, 484), (602, 594)],
            "packagen20": [(116, 631), (244, 741)],
            "packagen21": [(304, 631), (413, 738)],
            "packagen22": [(475, 633), (598, 738)],
        }

        if pkg in packages:
            [(x1, y1), (x2, y2)] = packages["{}".format(pkg)]
            # Focus particular area where particular box is present
            img = arg_image[y1:y2, x1:x2]

            qr_result = decode(img)
            if len(qr_result) > 0:
                color = qr_result[0].data
                # Assign only if the stored color value of that package is 'None'
                if rospy.get_param("packages/{}".format(pkg)) == "None":
                    color_code = (
                        lambda color: "R"
                        if color == "red"
                        else ("Y" if color == "yellow" else "G")
                    )
                    item_type = (
                        lambda color: "Medicine"
                        if color == "red"
                        else ("Food" if color == "yellow" else "Clothes")
                    )
                    priority = (
                        lambda color: "HP"
                        if color == "red"
                        else ("MP" if color == "yellow" else "LP")
                    )
                    storage_no = "R{} C{}".format(pkg[8], pkg[9])
                    # cost = lambda color: "{}".format(rd.randint(1000,1500)) if color == "red" else ("{}".format(rd.randint(500,1000)) if color == "yellow" else "{}".format(rd.randint(100,500)))
                    cost = (
                        lambda color: "1000"
                        if color == "red"
                        else ("500" if color == "yellow" else "100")
                    )
                    pkg_details = {
                        "color": color,
                        "item": item_type(color),
                        "priority": priority(color),
                        "cost": cost(color),
                        "dispatch_status": "NO",
                        "shipped_status": "NO",
                    }
                    rospy.set_param("packages/{}".format(pkg), pkg_details)
                    date_object = datetime.date.today()
                    date = "{}".format(date_object)
                    year = date.split("-")[0][2:]
                    month = date.split("-")[1]
                    sku = "{}{}{}{}{}".format(
                        color_code(color), pkg[8], pkg[9], month, year
                    )
                    parameters = str(
                        {
                            "id": "Inventory",
                            "Team Id": "VB#1516",
                            "Unique Id": "aYzqlq",
                            "SKU": sku,
                            "Item": item_type(color),
                            "Priority": priority(color),
                            "Storage Number": storage_no,
                            "Cost": cost(color),
                            "Quantity": "1",
                        }
                    )
                    print(parameters)
                    self._action_client.send_goal(parameters)
                    rospy.sleep(3)

    def callback(self, data):
        try:
            image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        # Defining package names
        pkg_name = [
            "packagen00",
            "packagen01",
            "packagen02",
            "packagen10",
            "packagen11",
            "packagen12",
            "packagen20",
            "packagen21",
            "packagen22",
        ]

        for i in range(len(pkg_name)):
            self.get_qr_data(image, pkg_name[i])

        # self._process_fineshed = True


def main():
    rospy.sleep(5)
    rospy.init_node("node_camera", anonymous=True)

    ic = Camera1()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == "__main__":
    main()
