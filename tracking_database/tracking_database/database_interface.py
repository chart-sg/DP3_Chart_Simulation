import logging
import rclpy
from rclpy.node import Node 
from tracking_msgs.msg import DeviceInfo, LocationInfo, Info 
from tracking_msgs.srv import WriteData, ReadData 
from .sqlite_db import SqliteDatabase
import sys
import yaml

class DatabaseInterface(Node):
    def __init__(self):
        super().__init__("database_node")
        LOG_FORMAT = "%(funcName)s %(lineno)s: %(message)s"
        logging.basicConfig(level = logging.INFO, format = LOG_FORMAT)
        self.logger = logging.getLogger()

        self.read_srv = self.create_service(ReadData, "read_from_db", self.read_db_cb)
        self.write_srv = self.create_service(WriteData, "write_to_db", self.write_db_cb)
        self.register_srv = self.create_service(WriteData, "register_db", self.register_tag_cb)

        config = self.load_yaml(sys.argv[1])
        self.hostdevices = config['hostdevices']
        self.whitelist = config['whitelist']
        self.zones = config['zone_map']
        self.db = SqliteDatabase()

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

    def register_tag_cb(self, request, response):
        device_msg = DeviceInfo()
        location_msg = LocationInfo()
        device_msg.unique_id = request.device.unique_id
        location_msg.zone = request.location.zone
        try: 
            self.db.register_whitelist(device_msg, location_msg, self.get_clock().now().to_msg().sec)
            response.write_result = response.WRITE_RESULT_SUCCESS
        except Exception as err:
            self.logger.info(err)
            response.write_result = response.WRITE_RESULT_FAILED

        return response

    def read_db_cb(self, request, response):
        mac_id = request.device.unique_id
        try: 
            self.check_data(mac_id)
        except Exception as err:
            if str(err) == "Invalid device":
                response.read_result = response.READ_RESULT_INVALID_DEVICE
                return response
        try: 
            old_zone = self.db.get_info_by_id(mac_id, "zone")
            response.location.zone = old_zone
            response.device.unique_id = mac_id
            response.read_result = response.READ_RESULT_SUCCESS
        except Exception as err:
            self.logger.info(err)
            response.read_result = response.READ_RESULT_FAILED
        
        return response

    def write_db_cb(self, request, response):
        self.logger.info("")
        self.logger.info("Writing Location Data to DB")
        device_msg = request.device
        location_msg = request.location
        time_stamp = self.get_clock().now().to_msg().sec
        try:
            self.check_data(device_msg.unique_id, location_msg.node_origin, location_msg.zone)
        except Exception as err:
            if str(err) == "Invalid device":
                self.logger.info("Invalid mac id")
                response.write_result = response.WRITE_RESULT_INVALID_DEVICE
            elif str(err) == "Invalid node name":
                self.logger.info("Invalid node")
                response.write_result = response.WRITE_RESULT_INVALID_NODE
            elif str(err) == "Invalid zone name":
                self.logger.info("Invalid zone")
                response.write_result = response.WRITE_RESULT_INVALID_ZONE
            return response
        try: 
            self.db.update(device_msg, location_msg, time_stamp)
            response.write_result = response.WRITE_RESULT_SUCCESS
        except Exception as err:
            self.logger.info(err)
            response.write_result = response.WRITE_RESULT_FAILED
        return response

    def check_data(self, mac_id, node = "None", zone = "None"):
        if mac_id not in self.whitelist:
            raise Exception("Invalid device")
        if node not in self.hostdevices:
            raise Exception("Invalid node name")
        if zone not in self.zones:
            raise Exception("Invalid zone name")
        else:
            self.logger.info("Data format correct")

def main(args=None):
    rclpy.init()
    x = DatabaseInterface()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
