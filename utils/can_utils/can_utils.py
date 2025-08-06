# -----------------------------------------------------------
# A utilities library to help with connecting to and reading/writing to a CAN bus.
# Based on python cantools library.
#
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------
import cantools
import can
from typing import Dict, List, Union

def encode_can_string_to_bytes(can_string: str) -> int:
    """
    Encode a string into an ASCII byte value.
    Args:
      can_string (string): A string to encode into a list of bytes.
    Returns:
      Int value representing the ASCII values of the chars in the string.
    """
    temp_list = []
    for i in range(8):  # 8 bytes per CAN message. One byte per ASCII char.
        if i < len(can_string):
            t = ord(can_string[i])
            temp_list.append(f'{t:x}')
        else:
            temp_list.append('00')
    temp_str = ''.join(str(x) for x in temp_list)
    int_of_string = int(temp_str, 16)
    return int_of_string

def connect_to_can_interface(interface_name: str, interface_type: str, bitrate: str, can_filters: Dict[str, Union[int, bool]], operation_mode: str = 'normal') -> can.interface.Bus:
    """
    Connect to the CAN interface.
    Args:
        interface_name (string): The network interface e.g can0
        interface_type (string): The CAN interface type e.g socketcan or pcan
        bitrate (string): The CAN interface type e.g socket_can or pcan
        can_filters (dict): A dictionary defining CAN IDs and CAN masks to apply to incoming CAN packets.
                            This filters CAN messages at the hardware level (if supported) so we only process messages we care about.
        operation_mode (string): The CAN interface operation mode e.g. normal, loopback, silent
    Returns:
        can.interface.Bus: The CAN interface object.
    """
    can_bus = can.interface.Bus(interface_name, interface=interface_type, bitrate=bitrate, can_filters=can_filters,operation_mode=operation_mode)
    return can_bus

def setup_can_filters(can_db: cantools.database.Database) -> List[Dict[str, Union[int, bool]]]:
    """
    Setup the CAN filters dictionary based on the DBC file.
    Allow all messages in the DBC file to be received. Any messages not defined in the can_db are filtered out and not read off the CAN bus.
    Args:
        can_db (dict): A dict of cantools.database defining the CAN DBC file, loaded by python cantools library.
    Returns:
        can_filters (dict): A dictionary defining CAN messages to receive off the CAN bus.
    """
    can_filters = []

    for message in can_db.messages:
        if message.is_extended_frame is False:
            can_filters.append({"can_id": message.frame_id, "can_mask": 0x7FF, "extended": False})
        else:
            # We usually won't use extended CAN messages.
            can_filters.append({"can_id": message.frame_id, "can_mask": 0x1FFFFFFF, "extended": True})
    return can_filters

class SignificantSignal:
    """
    A class to store values of interest over an accumulation period.
    This class can be used to downsample data so not all CAN bus traffic
    by keeping min, max and mean values over an accumulation period.
    It can also be used to simply store the last decoded value of a signal.


    """
    def __init__(self, sig_name: str, is_float: bool, is_str: bool = False, start_value: Union[int, float, str] = 0, keep_mean_stats: bool = False):
        self.sig_name = sig_name  # Name of this signal.
        self.keep_mean_stats = keep_mean_stats  # If true, keep the mean and last value as well as the min and max else just keep the last value.
        self.is_initiliased = False
        self.is_float = is_float
        self.is_str = is_str
        self.timestamp = 0 # The timestamp of the last update.
        self.signal_count = 0  # number of times this signal has been updated since last being cleared.
        # We need to store the type of value this signal is because signals should not be able to change type.
        if self.is_float:
            type_start_value = float(start_value)
        elif self.is_str:
            type_start_value = str(start_value)
        else:
            type_start_value = int(start_value)
        self.start_value = type_start_value
        self.min = type_start_value
        self.max = type_start_value
        self.mean = type_start_value
        self.last_value = type_start_value

    def update_value(self, new_value: Union[int, float, str], timestamp: int) -> None:
        """
        Update the stored value of this signal.

        Args:
            new_value (Union[int, float, str]): The new value to store.
            timestamp (int): The timestamp of the new value.
        """
        self.signal_count += 1
        self.timestamp = timestamp
        if self.is_float:
            type_new_value = float(new_value)
        elif self.is_str:
            type_new_value = str(new_value)
        else:
            type_new_value = int(new_value)
        if type_new_value != self.last_value:
            self.last_value = type_new_value
        self.is_initiliased = True

        # Only update the running min, max and mean values if we are keeping stats.
        if self.keep_mean_stats:
            if type(type_new_value) is not str:
                if type_new_value > self.max:
                    self.max = type_new_value
                if type_new_value < self.min:
                    self.min = type_new_value

                # Update the running mean over the accumulation period.
                self.mean = (self.mean * (self.signal_count - 1) + type_new_value) / self.signal_count
            else:
                # String type values are not supported for mean, min and max.
                self.is_initiliased = True
                self.min = type_new_value
                self.max = type_new_value
                self.mean = type_new_value
                self.last_value = type_new_value

    def clear_value(self):
        """
        Clear the stored value of this signal.
        """
        self.signal_count = 0
        self.min = self.start_value
        self.max = self.start_value
        self.mean = self.start_value
        # The last value is kept as the last value seen before the clear.
        self.is_initiliased = False

    def get_last_value(self):
        return self.last_value
    
    def get_last_value_timestamp(self):
        return self.timestamp
    
    def get_count(self):
        return self.signal_count

class CanDataAggregator:
    """
    A class to store a dictionary of the most interesting CAN signal values. The purpose of this class is to
    aggregate data over an accumulation period so only the most important signal values remain.
    """
    def __init__(self, can_dbc: cantools.database.Database):
        # A dictionary storing all the CAN data received in the last ACCUMULATE_PERIOD_SECS.
        # The key is the signal name and the value is the SignificantSignal object holding past values.
        self.can_dbc = can_dbc
        self.can_signals = {}
        # The timestamp when the last data was populated into the local storage.
        self.last_updated_time = 0
        # Setup a dictionary of SignificantSignal objects for each signal in the CAN database.
        for message in self.can_dbc.messages:
            for signal in message.signals:
                is_float = True
                is_str = False
                # Any signal that has choices defined  in the database will return a string for the decoded value.
                if signal.choices is not None:
                    if len(signal.choices) > 0:
                        is_float = False
                        is_str = True
                self.can_signals[signal.name] = SignificantSignal(signal.name, is_float, is_str, keep_mean_stats=True)

    def get_can_sig_valuenals_dict(self) -> Dict[str, Union[int, float, str]]:
        """
            Return a copy of the can_signals dictionary which maps signals names to their last value.
            This is slow do not call this except on intialisation.
        """
        can_signals_dict = {}

        for message in self.can_dbc.messages:
            for signal in message.signals:
                can_signals_dict[signal.name] = self.can_signals[signal.name].last_value
        return can_signals_dict

    def update_value(self, signal_name: str, value: Union[int, float, str], timestamp: int) -> None:
        """
        Update the value of a CAN signal in the local storage.

        Args:
            signal_name (str): The name of the signal to update.
            value (Union[int, float, str]): The new value of the signal.
            timestamp (int): The timestamp of the new value.
        """
        self.can_signals[signal_name].update_value(value, timestamp)

    def clear_agr_data(self):
        """
        Clear all the aggregated data.
        """
        for sig_signal in self.can_signals.keys():
            self.can_signals[sig_signal].clear_value()

    def get_max_output(self) -> Dict[str, Union[int, float, str]]:
        """
        Get the maximum CAN data for each signal over an accumulation period.
        Only returns signals that have been received at least once over the update period.
        Returns:
            A dictionary of the form {signal_name: max_value}
        """
        max_signals_output = {}
        for sig_signal in self.can_signals.keys():
            if self.can_signals[sig_signal].signal_count > 0:
                max_signals_output[sig_signal] = self.can_signals[sig_signal].max
        return max_signals_output

    def get_min_output(self) -> Dict[str, Union[int, float, str]]:
        """
        Get the minimum CAN data for each signal over an accumulation period.
        Returns:
            A dictionary of the form {signal_name: min_value}
        """
        min_signals_output = {}
        for sig_signal in self.can_signals.keys():
            if self.can_signals[sig_signal].signal_count > 0:
                min_signals_output[sig_signal] = self.can_signals[sig_signal].min

        return min_signals_output

    def get_mean_output(self) -> Dict[str, Union[int, float, str]]:
        """
        Get the mean of the CAN data for each signal over an accumulation period.
        Returns:
            A dictionary of the form {signal_name: mean_value}
        """
        mean_signals_output = {}
        for sig_signal in self.can_signals.keys():
            if self.can_signals[sig_signal].signal_count > 0:
                mean_signals_output[sig_signal] = self.can_signals[sig_signal].mean
        return mean_signals_output
