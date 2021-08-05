import logging
import sqlite3
from sqlite3 import Error
import sys
import yaml

class SqliteDatabase():
    def __init__(self):
        LOG_FORMAT = "%(funcName)s %(lineno)s: %(message)s"
        logging.basicConfig(level = logging.INFO, format=LOG_FORMAT)
        self.logger = logging.getLogger()

        self.conn = sqlite3.connect('dp3_tracking_database.db')
        self.c = self.conn.cursor()

        self.c.execute('''CREATE TABLE IF NOT EXISTS RssiTracker (
                    mac_id text,
                    zone text,
                    node_origin text,
                    last_seen integer 
                    )''')
        
    def register_whitelist(self, device_info, location_info, time):
        mac_id = device_info.unique_id
        zone = location_info.zone
        node_origin = ""
        last_seen = time
        with self.conn:
            self.c.execute("SELECT * FROM RssiTracker WHERE mac_id = :id", {'id': mac_id})
            data=self.c.fetchall()
            if len(data)==0:
                self.logger.info("Registering new device")
                self.c.execute("INSERT INTO RssiTracker VALUES (:mac_id, :zone, :node_origin, :last_seen)", 
                            {'mac_id': mac_id, 'zone': zone, 'node_origin': node_origin, 'last_seen':time})
                self.logger.info("Successful registration")
            else:
                self.logger.info("Device already registered")

    def get_info_by_id(self, target_id, field):
        with self.conn:
            self.c.execute("SELECT * FROM RssiTracker WHERE mac_id = :id", {'id': target_id})
            info = self.c.fetchall()
            if info == []:
                self.logger.info("No data available for requested id")
                self.logger.info("ID not yet registered or location not updated")
                return "not_registered" 
            else:
                if field == "all":
                    return info
                elif field == "zone":
                    return info[0][1]
                elif field == "node_origin":
                    return info[0][2]
                elif field == "last_seen":
                    return info[0][3]
    
    def get_all_info(self):
        with self.conn:
            self.c.execute("SELECT * FROM RssiTracker")
            info = self.c.fetchall()
            return info

    def update(self, device_info, location_info, time):
        mac_id = device_info.unique_id
        zone = location_info.zone
        node_origin = location_info.node_origin
        last_seen = time
        with self.conn:
            self.c.execute("""UPDATE RssiTracker SET zone = :zone, node_origin = :node_origin, last_seen = :last_seen where mac_id = :mac_id""",
                    {'zone': zone, 'node_origin': node_origin, 'last_seen': last_seen, 'mac_id': mac_id})
