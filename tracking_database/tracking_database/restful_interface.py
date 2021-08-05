#!/usr/bin/python3

import os
import requests 
import json
import logging 
import rclpy
from rclpy.node import Node 
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

class RestfulAPI(Node):

    def __init__(self):
        super().__init__("update_p_status")
        LOG_FORMAT = "%(funcName)s line[%(lineno)s]: %(message)s"
        logging.basicConfig(format = LOG_FORMAT, level = logging.INFO)
        self.logger = logging.getLogger()
        self.file_path = os.path.dirname(os.path.abspath(__file__))+'/'+'output.json'
        front = self.file_path.split('tracking_database')
        self.file_path = front[0] + 'tracking_database/share/tracking_database/resource/output.json'
        self.callback_group = ReentrantCallbackGroup()
        self.logger.info("Obtaining token")
        self.authentication()

        self.timer = self.create_timer(1, self.check_for_updates, callback_group=self.callback_group)

    def check_for_updates(self):
        self.number_of_cases, self.casualties = self.get_casualty_summary()
        if self.number_of_cases == None:
            self.logger.info("No new updates.")
            return 
        else:
            with open(self.file_path, 'w') as f:
                json.dump(self.casualties, f)
            self.logger.info("New update on the Casualties: {}".format(self.casualties))        

    def authentication(self): 
        header = {"x-api-key": "GCMUDiuY5a7WvyUNt9n3QztToSHzK7Uj", "content-type": "application/json"}
        body = {"email": "admin@mcis.server.com", "password": "P@ssw0rd"}
        r = requests.post('http://awseb-awseb-xg2au8qagzyv-1774340837.ap-southeast-1.elb.amazonaws.com:3001/v1/login/basic', headers = header, json = body)
        response = r.json()
        token = response['data']['tokens']['accessToken']
        self.token = 'Bearer ' + token 
        self.logger.info(self.token)

    def update_casulty_info(self, ble_id):
        link = 'http://awseb-awseb-xg2au8qagzyv-1774340837.ap-southeast-1.elb.amazonaws.com:3001/v1/casualty/commander/updateProfile?btid='
        url = link + ble_id
        self.logger.info(url)
        header = {"x-api-key": "GCMUDiuY5a7WvyUNt9n3QztToSHzK7Uj", "Content-Type": "application/json", "Authorization": self.token} 
        data = {
            "triggered": True 
        }
        r = requests.patch(url, headers = header, json = data)
        response = r.json()
        self.logger.info(response)

    def get_casualty_summary(self):
        url = 'http://awseb-awseb-xg2au8qagzyv-1774340837.ap-southeast-1.elb.amazonaws.com:3001/v1/casualty/cases/summary?conveypointName=CGH'
        header = {"x-api-key": "GCMUDiuY5a7WvyUNt9n3QztToSHzK7Uj", "Content-Type": "application/json", "Authorization": self.token} 

        r = requests.get(url, headers = header)
        response = r.json()

        self.logger.info("GET NEW UPDATES")
        try:
            # for obj in response["data"]:
            #     print(obj)
            #     for item in obj:
            #         print(item)
            #         if item == "numberOfCases":
            #             print(item) # Number of cases
            #             for x in obj[item]:
            #                 print(x)
            #                 number_of_cases = x  # {'Sum':8}
            #         elif item == "casualties":
            #             print(item) # Number of cases
            #             for x in obj[item]:
            #                 casualties = x  # {'Sum':8}                        
                    # print(obj[item])
            number_of_cases = response['data'][0]["numberOfCases"][0]
            casualties = response['data'][0]["casualties"]
            self.logger.info("Number of cases {}".format(number_of_cases)) 
            self.logger.info("Casualties {}".format(casualties))
            return (number_of_cases, casualties)
        except Exception as err:
            self.logger.info("Unable to pull casualty information from AWS web")
            return None

def main(args=None):
    rclpy.init(args=args)
    x = RestfulAPI()
    executor = MultiThreadedExecutor()
    rclpy.spin(x, executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()