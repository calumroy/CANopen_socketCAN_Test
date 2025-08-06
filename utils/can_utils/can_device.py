# -----------------------------------------------------------
# A base device controller class for communicating with a device over CAN.
# This class provides some basic utilities for talking to a real
# device over CAN using the python can library.
#
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------

import utils.can_utils.can_utils as can_utils
import cantools
import threading
from threading import Lock
import os
import time
import can

from logging import getLogger

log = getLogger('can_dev')

# The number of times to try and connect to the CAN interface.
NUM_CAN_INTERFACE_CONNECT_ATTEMPTS = 1

class CANDevice(object):
    """
    A class to read and write to a device over a CAN network.
    """
    def __init__(self, config: dict):
        """
        Constructor for the CANDevice class.
        Args:
            config: A dictionary of configuration parameters for this CAN device.
        """
        # Name of this sensor device controller
        self.name = config['name']

        self.show_controls = None  # An optional list of control input names to show in the get_control_inputs() function. If None then all controls are shown.
        if 'display' in config:
            if 'show_controls' in config['display']: 
                    self.show_controls = config['display']['show_controls']

        # Can device id for this sensor. This is read from the sensor.
        self.device_id = 0

        # Can interface config
        self.dbc_file_name = config['dbc_file_name']
        self.can_interface = config['can_interface']
        self.can_interface_type = config['can_interface_type']
        self.can_bitrate = config['can_bitrate']
        self.can_max_read_timeout_secs = config['can_max_read_timeout_ms'] / 1000.0
        self.operation_mode = 'normal'
        if 'operation_mode' in config:
            self.operation_mode = config['operation_mode']
        self.can_read_only = False # Similar to silent operation_mode but the interface still acknowledges messages.
        if self.operation_mode == 'silent':
            self.can_read_only = True

        self.stop_can = False
        self.can_bus = None

        # Load the DBC file parameter
        self.can_dbc = self.get_can_dbc(self.dbc_file_name)

        # Setup the can message filtering so only defined can messages are received.
        self.can_filter = can_utils.setup_can_filters(self.can_dbc)

        # A flag to indicate if the can device has been configured as per the config file.
        self.can_device_configured = False

        # Data store for thread safe storage of CAN data.
        # This is used to return the latest data in a thread safe manner to calling threads.
        self.data_read_dict = self._setup_read_data_dict(self.can_dbc) # Creates a dict of SignificantSignal objects for each CAN signal in the DBC file.
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
        self.can_error_count = 0
        self.can_max_read_errors = config['can_max_read_errors']
        self.can_comms_healthy = True  # Start off assuming the CAN comms are healthy. If not then this will be set to false.
        self.read_can_comms_enabled = True # A flag to indicate if the read CAN comms thread is enabled.
        self.transmit_buffer_full = False # A flag to indicate if the transmit buffer is full.
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

    def add_out_can_msg_data(self, can_message_name: str, sig_name_values_dict: dict):
        """
        Set the value of a CAN messages signal values to be sent out over CAN.
        This updates the data_write_dict but does not send the CAN message.
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

    def get_read_comms_thread(self):
        """
        Return a thread that continually reads the CAN bus and stores data in the data_read_dict.
        Thread safe.
        """

        # Connect to the CAN bus.
        self._connect_to_can_bus()

        # Create read thread.
        read_can_thread = threading.Thread(target=lambda: self._start_read_can_comms(), name=self.name, daemon=True)

        return read_can_thread

    def stop_read_comms(self):
        """
        Stop reading from the CAN bus(s).
        Thread safe.
        """
        self.stop_can = True
        if self.can_bus is not None:
            self.can_bus.shutdown() # Shutdown the can bus interface.
            self.can_bus = None
    
    def _connect_to_can_bus(self):
        # Setup the CAN bus
        self.can_bus = None

        # Connect to the Can bus interface.
        # Try up to a maximum number of times to connect to the CAN bus.
        try_num = 0
        while self.can_bus is None:
            try:
                self.can_bus = can_utils.connect_to_can_interface(self.can_interface, self.can_interface_type, self.can_bitrate, self.can_filter, self.operation_mode)
            except OSError:
                log.error("Cannot find the CAN interface %s. Is the CAN interface connected?", self.can_interface)
                self.can_bus = None
                time.sleep(1)  # Wait a second before trying again.
                try_num += 1
                if try_num >= NUM_CAN_INTERFACE_CONNECT_ATTEMPTS:
                    log.error("Failed to connect to the CAN interface %s after %s attempts.", self.can_interface, try_num)
                    self.can_bus = None
                    break

    def _start_read_can_comms(self):
        """
        Start reading CAN data off the CAN bus.

        """

        # Read CAN bus messages.
        log.info(f"Reading CAN device messages from {self.can_interface} interface")

        # CAN bus reading loop
        msg = None
        while not self.stop_can:
            if self.can_bus is not None:
                ##############################
                ### Receive a CAN message.
                ##############################
                msg = None
                try:
                    # Get a can message but only wait for a maximum time. This is to allow the thread to exit if the stop_can flag is set and to check if we have received a CAN message in the excepted time.
                    msg = self.can_bus.recv(self.can_max_read_timeout_secs)
                except can.CanOperationError as e:
                    log.warning(f"Can recv error, {e}")
                    self.can_error_count += 1
                
                else:
                    if msg is not None:
                        log.debug(f"Can msg recv Id: {hex(msg.arbitration_id)} for data {msg.data}")
                        decoded_msg = False
                        try:
                            message = self.can_dbc.decode_message(msg.arbitration_id, msg.data, decode_containers=True)
                            decoded_msg = True
                        except KeyError as e:
                            log.debug(f"Can msg recv Id: {hex(msg.arbitration_id)} not found in dbc file. Data: {msg.data}")
                        except Exception as e:
                            log.error(f"Can msg recv Id: {hex(msg.arbitration_id)} could not be decoded. Data: {msg.data}")
                            continue
                        if decoded_msg:
                            self.can_error_count = 0 # Clear error count as we have received a message.
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
                    else:
                        # No CAN message received in excepted time. 
                        # Increment error counter if we where expecting to receive CAN messages.
                        if self.read_can_comms_enabled:
                            self.can_error_count += 1
                # If we where expecting to receive CAN messages if CAN comms are still considered to be healthy.
                if self.read_can_comms_enabled:
                    if self.can_error_count >= self.can_max_read_errors:
                        if self.can_error_count % self.can_max_read_errors == 0:  # Only log every so often.
                            #log.error(f"CAN comms error count {self.can_error_count} exceeded {self.can_max_read_errors} errors.")
                            self.can_comms_healthy = False
            else:
                # Try connect to the CAN bus.
                self._connect_to_can_bus()
                # Sleep for the self.can_max_read_timeout_secs as the can interface is not connected.
                time.sleep(self.can_max_read_timeout_secs)
                log.debug("CAN interface %s not connected as self.can_bus is None.", self.can_interface)
        
        log.info("Stopping CAN read thread.")

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
    
    def get_can_sig(self, can_msg_sig_name: str) -> can_utils.SignificantSignal:
        """
        Get the latest CAN message signal object (can_utils.SignificantSignal) from the data_read_dict.
        This object contains the latest value and timestamp of the signal as well other signal information.

        Args:
            can_msg_sig_name (str): The name of the CAN message and signal name. e.g can_message_name.signal_name
        Returns:
            can_utils.SignificantSignal: A SignificantSignal object containing the latest values of the CAN signal
            
        """
        sigsig = None
        # Check if the message is in the data_read_dict.
        if can_msg_sig_name not in self.data_read_dict:
            log.error(f"Message {can_msg_sig_name} not found in data_read_dict.")
            return sigsig

        # Get the latest values.
        ## Data Lock ##
        # Lock the data dictionary to prevent other threads from accessing it.
        res = self.data_read_lock.acquire(timeout=self.read_lock_aquire_timeout_ms/1000.0)
        if res:
            sigsig = self.data_read_dict[can_msg_sig_name]
            self.data_read_lock.release()
        else:
            # We did not get the lock in the timeout period.
            log.error(F"Could not get latest value for {can_msg_sig_name} from the read data dictionary thread lock took longer then {self.read_lock_aquire_timeout_ms} ms.")

        return sigsig
        
    def send_raw_can_msg(self, can_message_id, can_message_data: bytes, flush_tx=False, is_extended_id=False):
        """
        Send an outgoing CAN message over the CAN bus.
        This takes a raw CAN message id and data (bytes) to send.
        Returns an error if the can msg is not sent.
        Thread safe.

        Args:
            can_message_id (int| str): The CAN message id to send as an int or alternatively the can msg name from the dbc file as a string.
            can_message_data (bytes): The CAN message data to send.
        Returns:
            (bool): True if the message was sent, False if the message was not sent.
        """

        if not self.can_read_only:    
            if type(can_message_id) is str:
                can_message_id = self.can_dbc.get_message_by_name(can_message_id).frame_id
            # type should be an int now.
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
            
    def send_can_msg(self, can_message_name: str, flush_tx=False, override_can_id: int =None):
        """
        Send an outgoing CAN message over the CAN bus.
        This looks in the data_write_dict for the latest values to send.
        Returns an error if the CAN message is not found in the data_write_dict or in the DBC file.
        Thread safe.

        Args:
            message_name (str): The name of the CAN message to send.
            flush_tx (bool): A flag to indicate if the CAN socket tx buffer should be cleared before sending the new CAN messages to it.
            override_can_id (int): An optional CAN id to override the CAN id in the DBC file.
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
                can_id = can_m.frame_id
                if override_can_id is not None:
                    can_id = override_can_id
                can_msg = can.Message(arbitration_id=can_id, data=can_raw_data, is_extended_id=can_m.is_extended_frame)
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
                        self.transmit_buffer_full = False
                        return True
                    else:
                        return False
                except can.CanOperationError as e:
                    log.error(f"Could not send CAN message {can_message_name} with values {sig_dict_val}")
                    log.error(f"Can send error, {e}")
                    # Only raise the `transmit_buffer_full` flag when the underlying
                    # CAN driver reports that the transmission buffer (mailbox/FIFO)
                    # is out of space.  We inspect the exception text for the common
                    # keywords returned by python-can and the kernel drivers.
                    if any(key in str(e).lower() for key in ("no buffer space",
                                                               "enobufs",
                                                               "buffer full",
                                                               "tx buffer full")):
                        self.transmit_buffer_full = True
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
        else:
            return False

    def send_all_out_can_msgs(self, flush_tx=False):
        """
        Send all outgoing CAN messages over the CAN bus.
        This looks in the data_write_dict for the latest values to send.
        Returns an error if the CAN message is not found in the data_write_dict or in the DBC file.
        Thread safe.

        Args:
            message_name (str): The name of the CAN message to send.
            flush_tx (bool): A flag to indicate if the CAN socket tx buffer should be cleared before sending the new CAN messages to it.
        Returns:
            (bool): True if the message was sent, False if the message was not sent.
        """
        if self.can_bus is not None:
            if flush_tx:
                try:
                    if hasattr(self.can_bus, 'flush_tx_buffer'):
                        self.can_bus.flush_tx_buffer()
                except NotImplementedError:
                    log.error("Could not flush tx buffer.")
                    pass
        for message_name in self.data_write_dict.keys():
            self.send_can_msg(message_name)

