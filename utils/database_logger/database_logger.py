# -----------------------------------------------------------
# A class for logging any given data to an influxDB database.
# Logs are written to the database in batches.
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------

import time
import logging
from threading import Lock
import socket
import os
from influxdb_client import InfluxDBClient
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.domain.bucket import Bucket
import influxdb_client.client.write_api

# Default values. These can be overwritten by the config file.
TOKEN = os.environ.get('DATABASE_TOKEN', "VDVOtnE6ximneH-TVfx6IN0xQQTwmPwdSmx-eJ3ZYrbKAS_E_3k16-9zMCM0kp5T5xcEriHrHLfPeUl3h5YE1w==") 
ORG = "switch"
BUCKET_NAME = "supplyrack_testbench"    

# Set up python logging for testing
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] [%(levelname)s] [%(threadName)s] [%(name)s] %(message)s",
)

# Set the logger name for this file.
log = logging.getLogger("database_logger")

class DatabaseLogger:
    def __init__(self, config):
        self.database_name = config['database_name']
        self.database_host = config['database_host']
        self.database_port = config['database_port']
        self.database_url = f"http://{self.database_host}:{self.database_port}"
        self.database_token = TOKEN
        if 'database_token' in config.keys():
            self.database_token = config['database_token']
        self.org_name = ORG
        if 'org_name' in config.keys():
            self.org_name = config['org_name']
        self.org_id = None    # The influxdb org id to use this is determined from the org name.

        self.hostname = socket.gethostname() # Hostname of CPU running this datalogger class.

        self.db_client = None
        
        self.start_time = None

        # Influxdb config
        self.write_api = None  # The influxdb client db write api to use.

        self.bucket = BUCKET_NAME    # The influxdb bucket_name to write to.
        if 'bucket_name' in config.keys():
            self.bucket = config['bucket_name']
        self.measurement_name = self.database_name  # Equivalent to a table name in SQL. All data is stored in this table.

        self.database_batch_size = config['database_batch_size']  # Number of Can messages to batch before sending to the database.
        self.influxdb_batch_data = []  # A list of batched data to send to the database.
        self.batch_data_lock = Lock()     # A lock to prevent multiple threads accessing the influxdb_batch_data dictionary at the same time.
        self.batch_data_lock_aquire_timeout_ms = 1000 # The timeout in milliseconds to acquire the lock.
        
    def start(self):
        self.connect_to_influxdb()
        self.start_time = time.monotonic()
            
    def is_influxdb_running(self, host='localhost', port=8086):
        """Check if InfluxDB is running on the specified host and port."""
        try:
            client = InfluxDBClient(url=f"http://{host}:{port}")
            ping_response = client.health()
            if ping_response.status == 'pass':
                return True
        except Exception:
            pass
        return False
        
    def connect_to_influxdb(self):
        """
        Connect to the influxdb database.
        This will create the influxdb bucket wherre data will be logged to if it doesn't already exist.
        """
        # Start the influxdb if it is not already running
        try:
            # Connect to the database
            self.db_client = InfluxDBClient(url=self.database_url, token=self.database_token, org=self.org_name) #, debug=True)
            log.info(f"Connected to influxdb database: {self.database_url} with token: {self.database_token}")
            # Create the bucket if it doesn't exist
            # Get the buckets
            influxdb_buckets_api = self.db_client.buckets_api()
            my_bucket = influxdb_buckets_api.find_bucket_by_name(self.bucket)
            if my_bucket is None:
                # Find the org_id given the org name. We need this to create a bucket since the influxdb client doesn't support creating a bucket with an org name.
                orgs_api = self.db_client.organizations_api()
                orgs = orgs_api.find_organizations()
                # Find and Print org_id for the given org_name
                for org in orgs:
                    if org.name == self.org_name:
                        log.info(f"org_id for {self.org_name}: {org.id}")
                        self.org_id = org.id
                        break
                # Create the bucket
                new_bucket = Bucket(
                    name=self.bucket,
                    retention_rules=[],
                    org_id=self.org_id
                    )
                influxdb_buckets_api.create_bucket(new_bucket)
            # Set the default write options for the write API
            self.write_api = self.db_client.write_api(write_options=SYNCHRONOUS)
            # Switch to the specified bucket
            self.write_api.bucket = self.bucket
            self.write_api.org = self.org_name
            self.bucket = self.bucket
            if self.write_api is None:
                log.error(f"Failed to set influxdb write api. The write api is None")
            if self.bucket is None:
                log.error(f"Failed to set influxdb bucket. The bucket is None")

        except Exception as e:
            log.error(f"Failed to connect to influxdb: {e}")

        
    def _store_data_batch(self):
        """
        Store the batched data to the database.
        This actually sends the batched data to the database and clears the batched data list.
        Not thread safe!
        """
        if self.influxdb_batch_data and self.write_api:
            if self.write_api is not None and self.bucket is not None:
                res = self.write_api.write(bucket=self.bucket, record=self.influxdb_batch_data)
                log.debug(f"Stored {len(self.influxdb_batch_data)} success: {res} data points to the database.")
            else:
                log.error(f"Failed to write to influxdb. The write api is {self.write_api} and the bucket is {self.bucket}")
            self.influxdb_batch_data = []
    
    def parse_and_store_data(self, device_name, data_dict, interface_name, timestamp: float, device_hostname: str = None, force_store_batch: bool = False):
        """
        Parse and store the data to the database.
        Thread safe.

        Args:
            device_name (str): The name of the device that the data was received from.
            data_dict (dict): A dictionary of data to store in the database. 
                              E.g
                              {
                                "variable1_name": variable1_value,
                                "variable2_name": variable2_value,
                               ...ect
                               }

            interface_name (str): The name of the interface that the data was received from.
            timestamp (float): The timestamp of the data in seconds since epoch.
            device_hostname (str, optional): The hostname of the device that the data was received from. Defaults to None.
            force_store_batch (bool, optional): Force the batched data to be stored to the database otherwise it is only stored when a batch is full. Defaults to False.
        """
        # Convert the timestamp in seconds since epoch to nanoseconds epoch.
        timestamp = int(timestamp * 1e9)
        data_point = {
            "measurement": self.measurement_name,
            "tags": {
                "device_name": device_name,
                "interface_name": interface_name, 
            },
            "fields": {},
            "time": timestamp 
        }
        if device_hostname is not None:
            data_point["tags"]["device_hostname"] = device_hostname

        for signal_name in data_dict.keys():
            signal_value = data_dict[signal_name]
            data_point["fields"][signal_name] = signal_value

         ## Data Lock ##
        # Lock the data dictionary to prevent other threads from accessing it.
        res = self.batch_data_lock.acquire(timeout=self.batch_data_lock_aquire_timeout_ms/1000.0)  # Try up to a maximum timeout to get the lock.
        if res:
            self.influxdb_batch_data.append(data_point)

            if (len(self.influxdb_batch_data) >= self.database_batch_size) or force_store_batch:
                self._store_data_batch()
            self.batch_data_lock.release()
            return True
        else:
            log.error(f"Failed to acquire batch_data_lock lock within timeout of {self.batch_data_lock_aquire_timeout_ms}ms.")
            return False

def main(config):
    log.info("Starting database logger")
    database_logger = DatabaseLogger(config)
    database_logger.start()
    log.info("Finished database logger")