#!/usr/bin/python3

import sys
import yaml
import logging 
import rclpy
from rclpy.node import Node 
from tracking_msgs.srv import Led
from tracking_msgs.msg import Info, DeviceInfo, LocationInfo
from tracking_database.sqlite_db import SqliteDatabase

class TagStatus(Node):
    def __init__(self):
        super().__init__("tag_status")
        LOG_FORMAT = "%(funcName)s line[%(lineno)s]: %(message)s"
        logging.basicConfig(format = LOG_FORMAT, level = logging.INFO)
        self.logger = logging.getLogger()
        
        self.tracking_led_client = self.create_client(Led, "one_led")
        while not self.tracking_led_client.wait_for_service(timeout_sec = 1.0):
            self.logger.info("Waiting for write connect led service ...")
        self.tracking_led_client_futures = []

        self.timer = self.create_timer(10, self.status_cb)
        self.config = self.load_yaml(sys.argv[1])
        self.whitelist = self.config["whitelist"]
        self.hostdevices = self.config["hostdevices"]
        self.db = SqliteDatabase()

        self.logger.info("--- Initialising tag location check ---")
        self.load_zone()

    def load_yaml(self, file_path: str):
        with open(file_path, 'r') as stream:
            try:
                yaml_config = yaml.safe_load(stream)
                assert yaml_config is not None
                return yaml_config
            except yaml.YAMLError as e:
                self.logger.error('Failed to load config file. Check its path.')
                raise yaml.YAMLError(e)
            except KeyError as e:
                self.logger.error('Config file is invalid. Missing field:')
                self.logger.error(e)
                raise AssertionError(e)

    def load_zone(self):
        self.logger.info("Grabbing whatever is in database\n")
        self.cmp_zone = {}
        info_dump = self.db.get_all_info()
        for entry in info_dump:
            mac_id = entry[0][0]
            zone = entry[0][1]
            node_origin = entry[0][2]
            time = entry[0][3]
            self.cmp_zone[mac_id] = zone

    def check_missing(self, mac_id, last_seen, timeout):
        current_time = self.get_clock().now().to_msg().sec
        if current_time - last_seen >= timeout:
            self.logger.info("Device {} cannot be found".format(mac_id))
    
    def check_zone(self, mac_id, latest_zone, last_seen):
        try:
            old_zone = self.cmp_zone[mac_id]
        except Exception as err:
            old_zone = None
        # right after registration, no need to update led
        if old_zone == None:
            self.cmp_zone[mac_id] = latest_zone
            return False
        # do not want to change triage led
        if latest_zone == "Triage":
            self.cmp_zone[mac_id] = latest_zone
            return False
        # no change in zone location
        if old_zone == latest_zone:
            return False
        else:
            self.cmp_zone[mac_id] = latest_zone
            self.logger.info("New zone update for mac id = {}".format(mac_id))
            return True
    
    def status_cb(self):
        led_update = {}
        self.logger.info("")
        self.logger.info("--- Checking tag status ---")
        info_dump = self.db.get_all_info()
        for entry in info_dump:
            mac_id = entry[0]
            zone = entry[1]
            node = entry[2]
            last_seen = entry[3]
            self.check_missing(mac_id, last_seen, 30)
            update_flag = self.check_zone(mac_id, zone, last_seen)
            if update_flag == True:
                if node not in led_update:
                    led_update[node] = []
                    led_update[node].append(mac_id)
                else:
                    led_update[node].append(mac_id)
        empty = bool(led_update)
        if empty is False:
            self.logger.info("Nothing to update")
        else:
            self.connect_request(led_update)        
            self.logger.info(" *** Tracking led request submitted ***")
        self.logger.info("--- Finished status check ---")

    def connect_request(self, data):
        infos = []
        for node in data:
            info_msg = Info()
            location_msg = LocationInfo()
            location_msg.node_origin = node
            
            empty = bool(data.get(node))
            if empty == False:
                devices = []
                device_msg = DeviceInfo()
                device_msg.unique_id = "" 
                devices.append(device_msg)
                device_msg = DeviceInfo()
            else:
                devices = []
                device_msg = DeviceInfo()
                for mac_id in data[node]:
                    device_msg.unique_id = mac_id
                    devices.append(device_msg)
                    device_msg = DeviceInfo()

            info_msg.devices = devices
            info_msg.location = location_msg
            infos.append(info_msg)
            info_msg = Info()

        write_req = Led.Request()
        write_req.info = infos 
        future = self.tracking_led_client.call_async(write_req)
        self.tracking_led_client_futures.append(future)

    def write_spin(self):
        incomplete_futures = []
        for f in self.tracking_led_client_futures:
            rclpy.spin_until_future_complete(self, f, timeout_sec = 1.0)
            if f.done():
                response = f.result()
                self.logger.info("Write result = {}".format(response.write_result))
            else:
                incomplete_futures.append(f)
        self.tracking_led_client_futures = incomplete_futures

def main(args = None):
    rclpy.init(args=args)
    x = TagStatus()
    while rclpy.ok():
        rclpy.spin_once(x)
        x.write_spin()
    x.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
