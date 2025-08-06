
import requests
from datetime import datetime, timedelta

# Replace with your actual InfluxDB details
org = "switch"
bucket = "supplyrack_testbench"
token = "hRZSM61yMgmgLhpIU8zEZ7dsuV3zNTEjDDbD6o7KHxWOw93iTxlG7LopLpyjKYdzhyHJ0mVXud1ueNz-HT4T7w=="
start_time = "2023-01-01T00:00:00Z"  # Start of the range to delete
stop_time = datetime.utcnow().isoformat() + "Z"  # Current time as the end of the range

# The InfluxDB API URL for delete
url = f"http://localhost:8086/api/v2/delete?org={org}&bucket={bucket}"

# Headers including the authorization token
headers = {
    "Authorization": f"Token {token}",
    "Content-Type": "application/json"
}

# Data to be deleted, specified by the predicate
#  CHECK THIS WILL DELETE THE DATA PERMATENTLY
# INFLUXDB DOES NOT SUPPORT DELETING DATA BASED ON THE DATAFIELD NAME
# THIS IS AN ONGOING ISSUE SINCE 2016. LIKELY NEVER TO BE FIXED (ITS 2023). 
data = {
    "start": start_time,
    "stop": stop_time,
    "predicate": '_measurement="supplyrack-testbench-2" AND device_hostname="calum" AND device_name="Testbench Controller 2" AND interface_name="localhost"'
}

# Send the POST request to delete the data
response = requests.post(url, headers=headers, json=data)

# Check the response
if response.status_code == 204:
    print("Delete request successful.")
else:
    print("Delete request failed.")
    print("Status Code:", response.status_code)
    print("Response:", response.text)
