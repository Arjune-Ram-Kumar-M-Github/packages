'''

# config_iot_ros.yaml 
# IoT Configuration
::

    config_iot:
    mqtt:
        server_url: "broker.mqttdashboard.com"        # http://www.hivemq.com/demos/websocket-client/
        server_port: 1883
        topic_sub: "/eyrc/vb/aYzqLq/orders"          # <unique_id> = aYzqLq
        topic_pub: "/eyrc/aYzqLq/ros_to_iot"          # <unique_id> = aYzqLq
        qos: 0

        sub_cb_ros_topic: "/ros_iot_bridge/mqtt/sub"   # ROS nodes can listen to this topic to receive data from MQTT


    google_apps:
        spread_sheet_id: "AKfycbyVEPZqnjZrv6_726rCFEySQ8eyURCNlqCQcz672Vxcs0ZogdAZ"                         # Spreadsheet Id/Webapp Id
        submission_spread_sheet_id: "AKfycbw5xylppoda-8HPjt2Tzq4ShU_Xef-Ik-hEtBPcPk0gdGw8095j4RZ7"          # Spreadsheet Id for submission

'''