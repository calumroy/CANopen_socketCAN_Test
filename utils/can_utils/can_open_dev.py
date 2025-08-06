# -----------------------------------------------------------
# A base device controller class for communicating with a device
# using the CANopen protocol. This class is intended to be
# provide some basic utilities for talking to a real
# device over CANopen using the python can library as the interface to the CAN bus.
#
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------

import utils.can_utils.can_utils as can_utils
import canopen
import cantools
import can
import time
import os
import argparse
import yaml
from threading import Lock
from utils.database_logger.database_logger import DatabaseLogger # Required if we want to log CAN sdo comms to the database.

from logging import getLogger

log = getLogger('canopen_dev')

DATA_TYPE_TO_LENGTH_BYTES = {
    1: 1, # BOOLEAN
    2: 1, # INTEGER8
    3: 2, # INTEGER16
    4: 4, # INTEGER32
    5: 1, # UNSIGNED8
    6: 2, # UNSIGNED16
    7: 4, # UNSIGNED32
    8: 4, # REAL32
    9: 4, # VISIBLE_STRING
    10: 4, # OCTET_STRING
    11: 4, # UNICODE_STRING
    12: 4, # TIME_OF_DAY
    13: 4, # TIME_DIFFERENCE
    14: 4, # DOMAIN
}

# The number of times to try and connect to the CAN interface.
NUM_CAN_INTERFACE_CONNECT_ATTEMPTS = 1
class CANOpenDevice:
    """
    A class to read and write to a CANopen device over a CANopen network.
    """
    def __init__(self, config: dict):
        """
        Constructor for the CANopenDevice class.
        Args:
            config: A dictionary of configuration parameters for this CAN device.
        """
        # Name of this sensor device controller
        self.name = config['name']

        self.show_controls = None  # An optional list of control input names to show in the get_control_inputs() function. If None then all controls are shown.
        if 'display' in config:
            if 'show_controls' in config['display']: 
                self.show_controls = config['display']['show_controls']

        # Can interface
        self.can_interface = config['can_interface']
        self.can_interface_type = config['can_interface_type']
        self.can_bitrate = config['can_bitrate']
        self.can_bus = None
        self.can_max_read_timeout_secs = config['can_max_read_timeout_ms'] / 1000.0
        self.operation_mode = 'normal'
        if 'operation_mode' in config:
            self.operation_mode = config['operation_mode']
        self.can_read_only = False # Similar to silent operation_mode but the interface still acknowledges messages.
        if self.operation_mode == 'silent':
            self.can_read_only = True

        # Canopen config
        self.device_id = config['device_id']
        self.eds_file = os.path.join(config['eds_file'])
        # The dbc file is used to read CAN messages and signals and give them a name before putting them into the local read dictionary.
        # The dbc file should match the configuration of the RPDOs and TPDOs definied in the .EDS file otherwise the read data will have the wrong names and/or values.
        self.dbc_file_name = config['dbc_file_name']
        # Save expected object dictionary entry values.
        self.object_dict_config = None
        if 'object_dict_config' in config:
            self.object_dict_config = config['object_dict_config']

        # Load the DBC file.
        self.can_dbc = self.get_can_dbc(self.dbc_file_name)

        # Canopen network, need to set a reference to the can_bus insterface in the network.
        self.network = None
        # canopen node contains an NMT Network Management service (one per canopen device) which manages only this device (do not let this broadcast to all devices).
        self.node = None
        self.listeners = None
        self.notifier = None

        # Stop the CANopen device
        self.stop_can = False

        # Setup the can message filtering so only defined can messages are received.
        # Careful not to filter out can messages that the canopen library needs to function.
        self.can_filter = [] # E.g To filter out all messages not definied in the dbc file with this can_utils.setup_can_filters(self.can_dbc)

        # A flag to indicate if the canopen device has been configured as per the config file (.eds file).
        self.can_device_configured = False

        # Data store for thread safe storage of CAN data.
        # This is used to return the latest data in a thread safe manner to calling threads.
        self.data_read_dict = self._setup_read_data_dict(self.can_dbc)  # Only reads CAN messages definied in the .dbc file.
        self.data_read_lock = Lock()     # A lock to prevent multiple threads accessing the data dictionary at the same time.
        self.read_lock_aquire_timeout_ms = 1000 # The timeout in milliseconds to acquire the lock.

        # Data store for thread safe storage of outgoing CAN data.
        # This is used to store CAN signal values set by other threads to then be periodically sent out over CAN.
        # This is a dict of a dict. The inner dict contains groups of can signal names and values belonging to one CAN message.
        # dict( can_message_name : dict( can_signal_name : can_signal_value )
        self.data_write_dict = {}  # Empty as initially no CAN messages are being sent.
        self.data_write_lock = Lock()     # A lock to prevent multiple threads accessing the data dictionary at the same time.
        self.write_lock_aquire_timeout_ms = 1000 # The timeout in milliseconds to acquire the lock.


        # Keep track of the number of times we failed to read a CAN message.
        self.disable_can_err_count_check = config['disable_can_err_count_check'] if 'disable_can_err_count_check' in config else False
        self.error_lock = Lock()     # A lock to prevent multiple threads accessing the error count at the same time.
        self.can_error_count = 0
        self.can_max_errors = config['can_max_errors'] if 'can_max_errors' in config else 10
        self.can_comms_healthy = True

        # Database Logger
        # Define a database to log all sdo CAN messages received from the device.
        self.database_logger = None
        if 'database_logger' in config:
            self.database_logger = DatabaseLogger(config['database_logger'])
            self.database_logger.start()

        # Get the CAN DBC file defining.
    def get_can_dbc(self, dbc_file_path_name):
        path = os.path.join(dbc_file_path_name)
        log.info(f"DBC file path is  {path}")
        return cantools.db.load_file(path)
    
    def _setup_read_data_dict(self, can_dbc: cantools.db.Database):
        """
            Setup a dictionary of SignificantSignal objects for each signal in the CAN database. These objects just store the latest data and some meta data on the signal.
            Args:
                can_dbc: The CAN database object.
            Returns:
                A dictionary of SignificantSignal objects for each signal in the CAN database.
        """
        data_dict = {}
        for message in can_dbc.messages:
            for signal in message.signals:
                is_float = True
                is_str = False
                # Any signal that has choices defined  in the database will return a string for the decoded value.
                if signal.choices is not None:
                    if len(signal.choices) > 0:
                        is_float = False
                        is_str = True
                # Store each value as the "message_name.signal_name" so we can have multiple signals with the same name but in different messages.
                mess_sign_name = message.name + "." + signal.name
                data_dict[mess_sign_name] = can_utils.SignificantSignal(mess_sign_name, is_float, is_str, keep_mean_stats=False)
        return data_dict
    
    def connect_to_can_bus(self):
        """
            Setup the canopen CAN bus.
            Connect to the can bus interface and setup and configure the canopen nodes.
            Start reading CAN messages.
            
            This funciotn creates a can interface and then passes this interface to the canopen library to use for CANopen functionailty such as sdo cAN comms. 
            A callback is setup in addition to the canopen library callbacks to read the can messages off the bus and process them.
            The reason for setting up our own can callback (can notifier) is so we can process and decode messages as definied in the dbc file, where as the 
            canopen librbary decodes messages as defined in the .eds file. 
            This is not ideal as we want to use the dbc file to decode the messages and signals and given them more descriptive names then what an eds file uses.
            The canopen library will still decode the messages and signals as definied in the .eds file but we will also decode the messages and signals as definied in the .dbc file.
        """

        self.can_bus = None
        self._setup_canopen()

        # Setup the CAN bus
        self.can_bus = None

        # Connect to the Can bus interface.
        try_num = 0
        while self.can_bus is None:
            try:
                self.can_bus = can_utils.connect_to_can_interface(self.can_interface, self.can_interface_type, self.can_bitrate, self.can_filter, self.operation_mode)
            except OSError:
                log.error("Cannot find the CAN interface %s. Is the CAN interface connected?", self.can_interface)
                self.can_bus = None
                self.increment_can_error_count()
                time.sleep(1)  # Wait a second before trying again.
                try_num += 1
                if try_num >= NUM_CAN_INTERFACE_CONNECT_ATTEMPTS:
                    log.error("Failed to connect to the CAN interface %s after %s attempts.", self.can_interface, try_num)
                    self.can_bus = None
                    break

        if self.can_bus is not None:
            # Associate the bus with the network
            self.network.bus = self.can_bus
            
            # Add your list of can.Listener with the network's
            # The purpose of this is so the canopen library can still operate via recevieing callbacks but we can also read the can messages off the bus in our own defined callback function.
            self.listeners = [self._handle_received_can_msg] + self.network.listeners
            # Start the can notifier which will notify all the listener functions when a can message is received.
            self.notifier = can.Notifier(self.can_bus, self.listeners, self.can_max_read_timeout_secs)


    def _setup_canopen(self):
        # Create CANopen network
        self.network = canopen.Network()

        # Load EDS configuration, add node to network
        self.node = canopen.RemoteNode(self.device_id, self.eds_file)
        self.network.add_node(self.node)

    def stop_read_comms(self):
        """
        Stop reading from the CAN bus(s).
        Thread safe.
        """
        self.stop_can = True
        if self.notifier is not None:
            self.notifier.stop()    # Stop the can notifier to prevent any more callback functions being called when a cna msg is received.
        if self.can_bus is not None:
            self.can_bus.shutdown() # Shutdown the can bus interface.
            self.can_bus = None

    def _handle_received_can_msg(self, msg: can.Message):
        """
        Handle a received can message by decoding it and adding it to a thread safe data store. 
        Note: The canopen library has its own CAN message callback function which is called when a can message is received in 
              parrallel to this function, so it can perform CANopen specific funcitonality. 
              This callback is for reading the CAN messages off the bus and decoding them as definied in the dbc file (which is not part of canopen funcitons).  
        
        Args:
            msg (can.Message): The received can message.

        """

        ##############################
        ### Receive a CAN message.
        ##############################
 
        if msg is not None:
            log.debug("Can msg recv Id: %s for data %s", hex(msg.arbitration_id), msg.data)
            decoded_msg = False
            try:
                message = self.can_dbc.decode_message(msg.arbitration_id, msg.data, decode_containers=True)
                decoded_msg = True
            except KeyError as e:
                #log.error("Can msg recv Id: %s not found in dbc file. Data: %s", hex(msg.arbitration_id), msg.data)
                pass
            except Exception as e:
                log.error("Can msg recv Id: %s failed to decode. Data: %s", hex(msg.arbitration_id), msg.data)
            if decoded_msg:
                self.clear_can_error_count() # Clear error count as we have received a message.
                self.can_comms_healthy = True

                message_name = self.can_dbc.get_message_by_frame_id(msg.arbitration_id).name
                # Add the latest message to the can data_read_lock thread safe storage. This is used by the higher level controllers to access the latest data.
                ## Data Lock ##
                # Lock the data dictionary to prevent other threads from accessing it.
                res = self.data_read_lock.acquire(timeout=self.read_lock_aquire_timeout_ms/1000.0)  # Try up to a maximum timeout to get the lock.
                if res:
                    for signal in message.keys():
                        self.data_read_dict[message_name + "." + signal].update_value(message[signal], msg.timestamp)
                    self.data_read_lock.release()
                else:
                    # We did not get the lock in the timeout period.
                    log.error(F"Could not update data dictionary with latest values thread lock took longer then {self.read_lock_aquire_timeout_ms} ms.")

    def get_can_sig_value(self, can_msg_sig_name: str):
        """
        Get the latest CAN message signal value from the data_read_dict.
        Thread safe.

        Args:
            can_msg_sig_name (str): The name of the CAN message and signal name. e.g can_message_name.signal_name
        Returns:
            (dict): A dictionary containing the latest values of the CAN signals in the CAN message.
        """
        sig_dict_val = None
        # Check if the message is in the data_read_dict.
        if can_msg_sig_name not in self.data_read_dict:
            log.error(f"Message {can_msg_sig_name} not found in data_read_dict.")
            return sig_dict_val

        # Get the latest values.
        ## Data Lock ##
        # Lock the data dictionary to prevent other threads from accessing it.
        res = self.data_read_lock.acquire(timeout=self.read_lock_aquire_timeout_ms/1000.0)
        if res:
            sig_dict_val = self.data_read_dict[can_msg_sig_name].get_last_value()
            self.data_read_lock.release()
        else:
            # We did not get the lock in the timeout period.
            log.error(F"Could not get latest value for {can_msg_sig_name} from the read data dictionary thread lock took longer then {self.read_lock_aquire_timeout_ms} ms.")

        return sig_dict_val

    def add_out_can_msg_data(self, can_message_name: str, sig_name_values_dict: dict):
        """
        Set the value of a CAN messages signal values to be sent out over CAN.
        This updates the data_write_dict but does not send the CAN message.
        The CAN messages can be any can messages definied in the dbc file. 
        Thread safe.

        Args:
            message_name (str): The name of the CAN message containing the CAN signal.
            sig_name_values_dict (dict): A dictionary of CAN signal names and values to be sent out over CAN.
        """

        res = self.data_write_lock.acquire(timeout=self.write_lock_aquire_timeout_ms/1000.0)  # Try up to a maximum timeout to get the lock.
        if res:
            self.data_write_dict[can_message_name] = sig_name_values_dict
            self.data_write_lock.release()
        else:
            log.error(f"Failed to acquire write lock for CAN message {can_message_name} with values {sig_name_values_dict}")

    def remove_out_can_msg(self, can_message_name: str):
        """
        Remove the CAN message from the data_write_dict so it is no longer sent out over CAN.
        Thread safe.

        Args:
            message_name (str): The name of the CAN message containing the CAN signal.
        """

        res = self.data_write_lock.acquire(timeout=self.write_lock_aquire_timeout_ms/1000.0)
        if res:
            if can_message_name in self.data_write_dict:
                del self.data_write_dict[can_message_name]
            self.data_write_lock.release()
        else:
            log.error(f"Failed to acquire write lock for removing CAN message {can_message_name}")
            
    def remove_all_out_can_msgs(self):
        """
        Remove all CAN messages from the data_write_dict so they are no longer sent out over CAN.
        Thread safe.
        """

        res = self.data_write_lock.acquire(timeout=self.write_lock_aquire_timeout_ms/1000.0)
        if res:
            self.data_write_dict.clear()
            self.data_write_lock.release()
        else:
            log.error("Failed to acquire write lock for removing all CAN messages")

    def increment_can_error_count(self):
        """
        Increment the number of CAN errors.
        Thread safe.
        """
        if not self.disable_can_err_count_check:
            with self.error_lock:
                self.can_error_count += 1

    def clear_can_error_count(self):
        """
        Clear the number of CAN errors.
        Thread safe.
        """
        with self.error_lock:
            self.can_error_count = 0

    def check_comms_healthy(self):
        # Check if CAN comms are still considered to be healthy
        err_count = 0
        with self.error_lock:
            err_count = self.can_error_count
        if err_count >= self.can_max_errors:
            log.debug(f"CAN comms error count exceeded {self.can_max_errors} errors.")
            self.can_comms_healthy = False
        else:
            self.can_comms_healthy = True
        return self.can_comms_healthy

    def set_canopen_state(self, state):
        """
        Set the state of the CANopen network.
                    - 'INITIALISING'
                    - 'PRE-OPERATIONAL'
                    - 'STOPPED'
                    - 'OPERATIONAL'
                    - 'SLEEP'
                    - 'STANDBY'
                    - 'RESET'
                    - 'RESET COMMUNICATION'
        """
        if not self.can_read_only:
            if state in ['INITIALISING', 'PRE-OPERATIONAL', 'STOPPED', 'OPERATIONAL', 'SLEEP', 'STANDBY', 'RESET', 'RESET COMMUNICATION']:
                try:
                    self.node.nmt.state = state
                except canopen.SdoCommunicationError:
                    log.error(f"SdoCommunicationError Cannot set the state of the CANopen network to {state}")
                    self.increment_can_error_count()
                    return False
                except can.CanOperationError as e:
                    log.error(f"CanOperationError {e} Cannot set the state of the CANopen network to {state}")
                    self.increment_can_error_count()
                    return False                            
                return True
            else:
                log.error("Cannot set the state of the CANopen network to %s", state)
                return False
        else:
            # Do not set the state if we are in can_read_only mode as this sends can messages.
            return False

    def get_canopen_state(self):
        """
        Get the state of the CANopen network.
            - 'INITIALISING',
            - 'STOPPED',
            - 'OPERATIONAL',
            - 'SLEEP',
            - 'STANDBY',
            - 'PRE-OPERATIONAL'
        """
        return self.node.nmt.state
    
    def _read_sdo_value(self, obj_name: str, sub_name: str = None, log_to_db: bool = False):
        """
        Read a value from the object dictionary on the remote device over CANopen using sdo comms.
        Return the decoded value and the raw un-decoded data.

        Args:
            obj_name (str|int): The name of the object dictionary entry or the integer index address value (address in the .eds file).
            sub_name (str|int): The name of the subindex or the integer index subindex value (sub address in the .eds file).
            log_to_db (bool): True if the sdo variable should be logged to the database. A valid database logger must be defined in the
                              config file and a connection to the database must be established already for this to occur.

        Returns:
            current_val, raw_data (int, bytes): The current value of the object dictionary entry and the raw data, read from the device.
        """
        raw_data = None
        current_val = None
        if not self.can_read_only:
            try:
                if sub_name is not None:
                    raw_data = self.node.sdo[obj_name][sub_name].data
                    current_val = self.node.sdo[obj_name][sub_name].raw
                else:
                    raw_data = self.node.sdo[obj_name].data
                    current_val = self.node.sdo[obj_name].raw
            except KeyError as e:
                log.error(f"Cannot find object dictionary entry {obj_name} {sub_name}")
                raw_data = None
                current_val = None
            except canopen.sdo.exceptions.SdoAbortedError as e:
                log.error(f"SdoAbortedError Cannot read object dictionary entry {obj_name} {sub_name}")
                raw_data = None
                current_val = None
            except canopen.sdo.exceptions.SdoCommunicationError as e:
                log.error(f"SdoCommunicationError Cannot read object dictionary entry {obj_name} {sub_name}")
                raw_data = None
                current_val = None
                self.increment_can_error_count()
            except can.CanOperationError as e:
                log.error(f"CanOperationError {e} Cannot read object dictionary entry {obj_name} {sub_name}")
                raw_data = None
                current_val = None
                self.increment_can_error_count()

            if current_val is not None:
                self.clear_can_error_count() # Clear error count as we have received an SDO message from the device and therefore have some CAN comms to it.
                # log to the database the results of the sdo variable read.
                if log_to_db:
                    if self.database_logger is not None:
                        # SDO variable will be stored as device_name.object_name.subindex_name
                        sdo_var_name = self.name + "." + obj_name
                        if sub_name is not None:
                            sdo_var_name = self.name + "." + obj_name + "." + sub_name
                        msg_timestamp = time.time()  # Unix timestamp in seconds.
                        # Log the sdo variable to the database use the device name and don't use a hostname.
                        self.database_logger.parse_and_store_data(self.name, {sdo_var_name: current_val}, self.can_interface, msg_timestamp, device_hostname=None)
                        log.debug(f"Stored SDO var {obj_name} {sub_name} to database with value {current_val}")

        return current_val, raw_data
    
    def _write_sdo_value(self, new_value: int, len_raw_data: int, obj_name: str, sub_name: str = None):
        """
        Write a value to the object dictionary on the remote device over CANopen using sdo comms.

        Args:
            new_value (int): The new value to write to the object dictionary entry.
            len_raw_data (int): The length of the raw data in bytes.
            obj_name (str|int): The name of the object dictionary entry or the integer index address value (address in the .eds file).
            sub_name (str|int): The name of the subindex or the integer index subindex value (sub address in the .eds file). Use None if there is no subindex.
        
        Returns:
            changed (bool): True if the value was successfully written to the remote CANopen device.
        """
        changed = False
        if not self.can_read_only: # Do not send sdo can mesages if in read only mode.
            snd_data = new_value.to_bytes(len_raw_data, byteorder='little')
            try:
                if sub_name is not None:
                    self.node.sdo[obj_name][sub_name].set_data(snd_data)
                else:
                    self.node.sdo[obj_name].set_data(snd_data)
                changed = True
                log.info(f"Set object dictionary entry {obj_name} {sub_name} to {new_value:#x}")
            except canopen.sdo.exceptions.SdoAbortedError as e:
                log.error(f"SdoAbortedError {e} Cannot write object dictionary entry {obj_name} {sub_name} to 0x{new_value:02x}")
                changed = False
            
        return changed

    def _set_pdo_mapping_param(self, obj: dict):
        """
        To change any object dictionary entry that is a PDO Mapping Parameter we need to follow a particular sequence of writes.
            1. Disable the PDO (set highest bit in the COBID)
            2. Write zero to the number of mapping parameters.
            3. Write the new mapping parameter values.
            4. Write the new non zero number of mapping parameters.
            5. Enable the PDO by writing the original COBID back to the device.
        
        Args:
            obj (dict): A dictionary with the following entries:
                - "name" (str): The name of the object dictionary entry Mapping Parameter.
                - "subindicies" (list Optional):  An optional list of dictionaries with the following entries:
                        - "name" (str): The name of the subindex.
                        - "value" (int): The expected value of the subindex.
        Returns:
            configured, changed (bool, bool): Configured = True if the device Mapping was set correctly. Changed = True if the mapping was changed.
        """   
        
        map_configured = True
        map_changed = False
        obj_name = obj['name']
        if "Mapping Parameter" in obj_name:
            if "subindicies" in obj:
                for subindex in obj["subindicies"]:
                    # If the current value does not match the expected value then set it to the expected value.
                    sub_name = subindex['subname']
                    exp_val = subindex['value']
                    current_val, raw_data = self._read_sdo_value(obj_name, sub_name)
                    if current_val is not None and raw_data is not None:
                        # If the value is not correct set the value. This means go through steps 1 to 5 above.
                        if current_val != exp_val:
                            ######################
                            ## Step 1.
                            #####################
                            # Get the current COBID by finding the matching pdo.
                            ob_addr = self.node.sdo[obj_name].od.index
                            ob_cobid = None
                            cobid_len = None
                            pdo_number = None
                            cobid_raw_data = None

                            # Find the location of the matching cob_id of this pdo to disable it.
                            # From the obj_name check if this is a rpdo or tpdo and get the pdo number
                            is_rpdo = ("rpdo" in obj_name.casefold()) or ("receive" in obj_name.casefold())
                            is_tpdo = ("tpdo" in obj_name.casefold()) or ("transmit" in obj_name.casefold())
                            assert(is_rpdo != is_tpdo)  # It cannot be a rpdo and tpdo at the same time.
                            # Get the number from the obj_name
                            if any(char.isdigit() for char in obj_name):
                                pdo_number = int(''.join(filter(str.isdigit, obj_name)))
                            check_pdos = None
                            if is_rpdo:
                                check_pdos = self.node.rpdo
                            elif is_tpdo:
                                check_pdos = self.node.tpdo
                            # Check if in the eds file 0 is used to define pdo1.
                            lowest_pdo_number_name = check_pdos[1].com_record[1].name
                            lowest_pdo_number = int(''.join(filter(str.isdigit, lowest_pdo_number_name)))
                            if lowest_pdo_number == 0:
                                pdo_number += 1
                            
                            if check_pdos is not None and pdo_number is not None:
                                ob_cobid = check_pdos[pdo_number].cob_id
                                dis_ob_cobid = ob_cobid
                                # Disable the PDO by setting the highest bit in the COBID.
                                dis_ob_cobid |= 0x80000000 
                                # Write the new COBID to the device.
                                sdo_full_name_cob_id = check_pdos[pdo_number].com_record[1].name
                                # Split the full name into the object name and subindex name split at the . in the str
                                sdo_full_name_cob_id_split = sdo_full_name_cob_id.split(".")
                                # Get the length of the COB-ID object dictionary entry by just reading the raw value from the device (we don;t trust the EDS file for the correect datatype and length as it may have errors).
                                cobid_cur_val, cobid_raw_data = self._read_sdo_value(sdo_full_name_cob_id_split[0], sdo_full_name_cob_id_split[1])
                                cobid_len = len(cobid_raw_data)  # Required to write the correct number of bytes back to the device with the new cob id value.
                                map_changed |= self._write_sdo_value(dis_ob_cobid, cobid_len, sdo_full_name_cob_id_split[0], sdo_full_name_cob_id_split[1])
                                map_configured = False if not map_changed else map_configured
                            if cobid_raw_data is not None and cobid_len is not None and pdo_number is not None and check_pdos is not None:
                                ######################
                                ## Step 2.
                                #####################
                                # Write zero to the number of mapping parameters.
                                ob_num_map_param = 0
                                # The number of entries is always at sub index 0.
                                num_map_val, num_map_raw_data = self._read_sdo_value(obj_name, 0)
                                if num_map_raw_data is not None:
                                    map_changed |= self._write_sdo_value(ob_num_map_param, len(num_map_raw_data), obj_name, 0)
                                    map_configured = False if not map_changed else map_configured
                                ######################
                                ## Step 3.
                                #####################
                                # Write the new mapping parameter values. Use the len of the raw_data read back from the device (not the eds file) to write the correct number of bytes.
                                map_changed |= self._write_sdo_value(exp_val, len(raw_data), obj_name, sub_name) 
                                map_configured = False if not map_changed else map_configured
                                ######################
                                ## Step 4.
                                #####################
                                # Write the new non zero number of mapping parameters.
                                # read the number of mapping parameters.
                                check_pdos[pdo_number].read()
                                new_len_map = len(check_pdos[pdo_number].map)
                                for sub_in in obj["subindicies"]:
                                    if sub_in["subname"] == "Number of entries":
                                        new_len_map = int(sub_in["value"])
                                        break
                                # The number of entries is always at sub index 0.
                                if num_map_raw_data is not None:
                                    map_changed |= self._write_sdo_value(new_len_map, len(num_map_raw_data), obj_name, 0)
                                    map_configured = False if not map_changed else map_configured
                                ######################
                                ## Step 5.
                                #####################
                                # Enable the PDO by writing the original COBID back to the device.
                                # Write the new COBID to the device.
                                map_changed |= self._write_sdo_value(cobid_cur_val, cobid_len, sdo_full_name_cob_id_split[0], sdo_full_name_cob_id_split[1])
                                map_configured = False if not map_changed else map_configured
                    else:
                        map_configured = False
        return map_configured, map_changed

    def configure_object_dict_config(self, config_dict):
        """
            Go through the config["object_dict_config"] section and check that the values are correct.
            If they are not correct then try to set the correct values.
            If the values are correct then return True else return False.

            Args:
                config_dict (dict): The config dictionary. Expected entries are "object_dict_config" and "device_id".
                                    The "object_dict_config" entry is a list of dictionaries with the following entries:
                                        - "name" (str): The name of the object dictionary entry.
                                        - "value" (int Optional): An optional entry if subindicies doesn't exit. The expected value of the object dictionary entry.
                                        - "subindicies" (list Optional):  An optional list of dictionaries with the following entries:
                                                - "name" (str): The name of the subindex.
                                                - "value" (int): The expected value of the subindex.
            Returns:
                configured (bool): True if the object dictionary is configured correctly else False.
        """
        
        configured = True
        changed = False

        # Only try to read and configure the canopen device if there are object dictionary entries to configure.
        if "object_dict_config" in config_dict:
            # Load the eds file configuration from the node.
            # This reads the current configuration of the node from the node device and updates the local object dictionary with these values.
            try:
                if not self.can_read_only: # Do not send can messages if in read only mode.
                    self.node.load_configuration()
            except canopen.sdo.exceptions.SdoCommunicationError as e:
                log.error(f"SdoCommunicationError {e} Cannot load_configuration from canopen device {self.device_id}")
                self.increment_can_error_count()
            except can.CanOperationError as e:
                log.error(f"CanOperationError {e} Cannot load_configuration from canopen device {self.device_id}")
                self.increment_can_error_count()
        
        
            for obj in config_dict["object_dict_config"]:
                if "name" in obj:
                    obj_name = obj['name']
                    # Check if this is a PDO Mapping Parameter. These require a particular sequeunce of writes to change.
                    # If the name contians "Mapping Parameter"
                    if "Mapping Parameter" in obj_name:
                        mp_configured, mp_changed = self._set_pdo_mapping_param(obj)
                        configured &= mp_configured 
                        changed |= mp_changed
                    else:
                        # Any other object dictionary entry should be set directly.
                        if "subindicies" in obj:
                            for subindex in obj["subindicies"]:
                                if obj_name in self.node.object_dictionary:
                                    sub_name = subindex['subname']
                                    exp_val = subindex['value']
                                    current_val, raw_data = self._read_sdo_value(obj_name, sub_name)
                                    if current_val is not None and raw_data is not None:
                                        # If the value is not correct set the value.
                                        if current_val != exp_val:
                                            changed |= self._write_sdo_value(exp_val, len(raw_data), obj_name, sub_name)
                                            if not changed:
                                                configured = False
                                    else:
                                        configured = False
                                else:
                                    log.error(f"Cannot find object dictionary entry {obj_name}")
                                    configured = False
                        else:
                            # This object has no subindicies so just check the value.
                            if obj_name in self.node.object_dictionary:
                                exp_val = subindex['value']
                                current_val, raw_data = self._read_sdo_value(obj_name)
                                if current_val is not None and raw_data is not None:
                                    if current_val != exp_val:
                                        # The value is not correct so set the value.
                                        changed |= self._write_sdo_value(exp_val, len(raw_data), obj_name)
                                        if not changed:
                                            configured = False
                                else:
                                    configured = False
                else:
                    log.debug(f"Object dictionary entry has no name {obj}")
                    pass
        
        if changed:
            # Save the configuration parameters so a power cycle doesn't wipe them.
            try:
                if not self.can_read_only: # Do not send sdo can mesages if in read only mode.
                    self.node.store()
            except canopen.sdo.exceptions.SdoCommunicationError as e:
                log.error(f"Cannot store configuration to canopen device {self.device_id}")
                configured = False
                self.increment_can_error_count()

        log.info(f"Object dictionary configured as expected: {configured}, it was changed to this expected configuration: {changed}")
        return configured        
    
    def read_all_object_dict_entries_from_device(self, save_read_value=False):
        """
        Use SDO read commands to read all the object dictionary entries from the remote device.
        
        Fix any data type size errors.
        This means the object dictionary entries are read and the sized returned is compared to the 
        size defined in the .eds file object dictionary. If the size is different, then the data type is changed
        to a large size to accomadate the returned data.

        The default value of the object dictionary entries is updated to the value returned from the device.
        
        Args:
            save_read_value (bool, optional): Save the read values from the device to the default value of the object dictionary entry. Defaults to False.
                                              This is useful if you want to save the current configuration of the device.
        """

        data_type_changed = []
        failed_to_read = []

        if not self.can_read_only:
            for OB_entry in self.node.object_dictionary.values():
                ode_read_value = None # The object dictionary entry value read from the device.

                # Print the OB_entry.name and OB_entry.index as a hex value
                log.info('0x%X: %s' % (OB_entry.index, OB_entry.name))
                # If it has subobjects (check if the subindicies attribute exists), print them.

                # Check if the values() function exists. This will exist for entries that have sub indicies or are an array.
                if "values" in dir(self.node.sdo[OB_entry.name]):
                    for subobj in OB_entry.values():
                        # Print the OB_entry.index as a hex value and print the subobj.subindex and subobj.name
                        log.info('    0x%X: %d: %s' % (OB_entry.index, subobj.subindex, subobj.name))

                        # Fix bug in canopen. For ARRAY types the first index always has datatype = 5 = unsignedint8
                        # Some devices can respond with a different datatype for the first index ie. unsigned16 which will cause an canopen error.
                        # This is a hack to fix this issue.
                        # First get the raw data retured from the device. 
                        try:    
                            raw_data = self.node.sdo[OB_entry.name][subobj.name].data
                        except canopen.sdo.exceptions.SdoAbortedError:
                            # If the device returns an error, then skip this subobject.
                            continue
                        # Get the lenght of the data as definied in the object dictionary.
                        data_type = self.node.sdo[OB_entry.name][subobj.name].od.data_type
                        # Convert the data_type int to a max length for that data type.
                        max_length = DATA_TYPE_TO_LENGTH_BYTES[data_type]
                        # If the length of the raw data is more then the max length, then the data type is wrong.
                        # Set the data type to a a size bigger then the raw data.
                        raw_data_len = len(raw_data)
                        if raw_data_len > max_length:
                            # Set the data type to a a size equal to the raw data returned.
                            if raw_data_len == 4:
                                # Set to data_type = 4 = INTEGER32
                                self.node.sdo[OB_entry.name][subobj.name].od.data_type = 4
                            elif raw_data_len == 2:
                                # Set to data_type = 3 = INTEGER16
                                self.node.sdo[OB_entry.name][subobj.name].od.data_type = 2
                            # Add the data name to the list of names that had the data type changed.
                            data_type_changed.append(f"0x%X: {OB_entry.name}.{subobj.name}" % OB_entry.index)
                        # Now try read using the canopen library with the correct data type.
                        try:
                            ode_read_value = self.node.sdo[OB_entry.name][subobj.name].raw
                            if save_read_value:
                                # Save the value read from the device to the object dictionary entry in the value.
                                self.node.object_dictionary[OB_entry.name][subobj.name].value = ode_read_value
                        except Exception as e:
                            log.info(e)
                            failed_to_read.append(f"0x%X: {OB_entry.name}.{subobj.name}" % OB_entry.index)
                else:
                    try:
                        ode_read_value = self.node.sdo[OB_entry.name].raw
                        if save_read_value:
                            # Save the value read from the device to the object dictionary entry in the value.
                            self.node.object_dictionary[OB_entry.name].value = ode_read_value
                    except Exception as e:
                        log.info(e)
                        failed_to_read.append(f"0x%X: {OB_entry.name}" % OB_entry.index)

        if len(data_type_changed) >0:
            # Print the data_type_changed list 
            log.error("These object dict entries from the canopen .eds file do not match the data size returned from the device. The data size of the loaded .eds was adjusted to read them, data_type_changed:")
            log.error(data_type_changed)
        if len(failed_to_read) >0:
            # Print the failed_to_read list
            log.error("These object dict entries from the canopen .eds file could not be read from the device, failed_to_read:")
            log.error(failed_to_read)

    def send_raw_can_msg(self, can_message_id, can_message_data: bytes, flush_tx=False, is_extended_id=False):
        """
        Send an outgoing CAN message over the CAN bus.
        This takes a raw CAN message id and data (bytes) to send.
        Returns an error if the can msg is not sent.
        Thread safe.

        Args:
            can_message_id (int| str): The CAN message id to send as an int or alternatively the can msg name fromt he dbc file as a string.
            can_message_data (bytes): The CAN message data to send.
        Returns:
            (bool): True if the message was sent, False if the message was not sent.
        """

        if not self.can_read_only: 
            if type(can_message_id) is str:
                can_message_id = self.can_dbc.get_message_by_name(can_message_id).frame_id
            if type(can_message_id) is int:
                try:
                    ##############################
                    ### Send a CAN message.
                    ##############################
                    can_msg = can.Message(arbitration_id=can_message_id, data=can_message_data, is_extended_id=is_extended_id)
                    if self.can_bus is not None:
                        if flush_tx:
                            try:
                                if hasattr(self.can_bus, 'flush_tx_buffer'):
                                    self.can_bus.flush_tx_buffer()
                            except NotImplementedError:
                                log.error("Could not flush tx buffer.")
                                pass
                        self.can_bus.send(can_msg)
                        log.debug(f"Sent raw CAN message {can_message_id} with data {can_message_data}")
                        return True
                    else:
                        return False
                except can.CanOperationError as e:
                    log.error(f"Could not send raw CAN message {can_message_id} with data {can_message_data}")
                    log.error(f"Can send error, {e}")
                    # Clear CAN socket tx buffer to prevent overflow.
                    if self.can_bus is not None:
                        try:
                            if hasattr(self.can_bus, 'flush_tx_buffer'):
                                self.can_bus.flush_tx_buffer()
                        except NotImplementedError:
                            log.error("Could not flush tx buffer.")
                            pass
                    return False
            else:
                log.error(f"Could not send raw CAN message {can_message_id} with data {can_message_data}")
                return False
            
    def send_can_msg(self, can_message_name: str, flush_tx=False):
        """
        Send an outgoing CAN message over the CAN bus.
        This looks in the data_write_dict for the latest values to send.
        Returns an error if the CAN message is not found in the data_write_dict or in the DBC file.
        Thread safe.

        Args:
            message_name (str): The name of the CAN message to send.
            flush_tx (bool, optional): Flush the CAN socket tx buffer before sending the new messages to prevent overflow. Defaults to False.
        Returns:
            (bool): True if the message was sent, False if the message was not sent.
        """

        # Check if the message is in the data_write_dict.
        if can_message_name not in self.data_write_dict:
            log.error(f"Message {can_message_name} not found in data_write_dict.")
            return False

        # Get the latest values to send.
        ## Data Lock ##
        # Lock the data dictionary to prevent other threads from accessing it.
        sig_dict_val = None
        res = self.data_write_lock.acquire(timeout=self.read_lock_aquire_timeout_ms/1000.0)
        if res:
            sig_dict_val = self.data_write_dict[can_message_name]
            self.data_write_lock.release()
        else:
            # We did not get the lock in the timeout period.
            log.error(F"Could not get latest values from data dictionary thread lock took longer then {self.read_lock_aquire_timeout_ms} ms.")
            return False

        if not self.can_read_only:
            if sig_dict_val is not None:
                # Get the latest values to send.
                try:
                    can_raw_data = self.can_dbc.encode_message(can_message_name, sig_dict_val,strict=False,padding=False)
                except KeyError as e:
                    log.error(f"Could not encode message {can_message_name} with values {sig_dict_val}, {e}")
                    return False
                can_m = self.can_dbc.get_message_by_name(can_message_name)
                can_msg = can.Message(arbitration_id=can_m.frame_id, data=can_raw_data, is_extended_id=can_m.is_extended_frame)
                try:
                    ##############################
                    ### Send a CAN message.
                    ##############################
                    if self.can_bus is not None:
                        if flush_tx:
                            try:
                                if hasattr(self.can_bus, 'flush_tx_buffer'):
                                    self.can_bus.flush_tx_buffer()
                            except NotImplementedError:
                                log.error("Could not flush tx buffer.")
                                pass
                        self.can_bus.send(can_msg)
                        log.debug(f"Sent CAN message {can_message_name} with values {sig_dict_val}")
                        return True
                    else:
                        return False
                except can.CanOperationError as e:
                    log.error(f"Could not send CAN message {can_message_name} with values {sig_dict_val}")
                    log.error(f"Can send error, {e}")
                    self.increment_can_error_count()
                    # Clear CAN socket tx buffer to prevent overflow.
                    if self.can_bus is not None:
                        try:
                            if hasattr(self.can_bus, 'flush_tx_buffer'):
                                self.can_bus.flush_tx_buffer()
                        except NotImplementedError:
                            log.error("Could not flush tx buffer.")
                            pass
            else:
                log.error(f"Could not get latest values for message {can_message_name}")
                return False
        return False
    
    def send_all_out_can_msgs(self, flush_tx=False):
        """
        Send all outgoing CAN messages over the CAN bus.
        This looks in the data_write_dict for the latest values to send.
        Returns an error if the CAN message is not found in the data_write_dict or in the DBC file.
        Thread safe.

        Args:
            message_name (str): The name of the CAN message to send.
            flush_tx (bool, optional): Flush the CAN socket tx buffer before sending the new messages to prevent overflow. Defaults to False.
        Returns:
            (bool): True if the message was sent, False if the message was not sent.
        """
        if self.can_bus is not None:
            if flush_tx:
                self.can_bus.flush_tx_buffer() # Clear CAN socket tx buffer to prevent overflow.
        for message_name in self.data_write_dict.keys():
            self.send_can_msg(message_name)
        

    def export_object_dictionary_to_file(self, file_name):
        """Export the object dictionary to a file.

        Args:
            file_path (str): The path to the file to export to.
        """
        canopen.export_od(self.node.object_dictionary,file_name)

    def close(self):
        # Disconnect the CANopen network
        self.network.disconnect()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default='', help=f'An optional path to a config file if none is provided a default path is used.')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.FullLoader)
            config = config["axio_dev"]
        axio_dev = CANOpenDevice(config)
        axio_dev.start()

        time.sleep(100)

        log.info("Stopped btms CAN comms")
        time.sleep(5)
        log.info("Exiting axio_dev")

    except (KeyboardInterrupt, SystemExit):
        log.error("Exiting")