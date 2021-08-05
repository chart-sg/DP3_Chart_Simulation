#!/usr/bin/python3

import sys
import json
import yaml
import logging 
from tracking_database.restful_interface import RestfulAPI

class CheckPatientLocation():
    def __init__(self):
        super().__init__()
        LOG_FORMAT = "%(funcName)s line[%(lineno)s]: %(message)s"
        logging.basicConfig(format = LOG_FORMAT, level = logging.INFO)
        self.logger = logging.getLogger()
        
        self.ra = RestfulAPI()

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

def main(args = None):
    cpl = CheckPatientLocation()
    number_of_cases, casualties = cpl.get_casualty_summary() 
    print ("Callign in from rtls_check")
    print (number_of_cases)
    print (casualties)

if __name__ == "__main__":
    main()
