# -----------------------------------------------------------
# A device controller for controlling an Axiomatic (Axiomatic Technologies Corp) 
# device over CAN using the python can library.
#
# (C) 2025 Switch, Perth, Australia
# -----------------------------------------------------------

import argparse
import yaml
import time
import threading
from threading import Lock
import logging
from typing import Union

import utils.control_inputs as cntrl_inputs_utils
import utils.state_diagrams as state_diagrams
from utils.can_utils import can_open_dev
from logging import getLogger
from dataclasses import dataclass, replace

log = getLogger("axiomatic_dev")

AXIOMATICCONTROLINPUTRANGES = {}

@dataclass
class AxiomaticControl(cntrl_inputs_utils.ControlInputs):
    """
    A data class to store the Axiomatic control parameters.
    These are the controllable parameters that higher level controllers can set.
    """
    enable: bool = True       # Enable the Axiomatic device controller (enable CAN comms and enter the active state).
    can_read_only: bool = False # Disable writing CAN messages to the device.
    clearFaults: bool = False # Clear any faults on the axiomatic device.




@dataclass
class AxiomaticControlOutputs():
    """
    A data class to store the Axiomatic control outputs.
    These are the outputs that other device controllers can read from the Axiomatic device controller.
    """
    device_controller_ready: bool = False # Indicates if the device is ready to be used.
    device_fault: bool = False # Indicates if the device is in a fault state.
    input1_value: float = 0.0 # Generic input 1 value
    input2_value: float = 0.0 # Generic input 2 value
    input3_value: float = 0.0 # Generic input 3 value
    input4_value: float = 0.0 # Generic input 4 value
    input5_value: float = 0.0 # Generic input 5 value
    input6_value: float = 0.0 # Generic input 6 value
    input7_value: float = 0.0 # Generic input 7 value
    input8_value: float = 0.0 # Generic input 8 value
    input9_value: float = 0.0 # Generic input 9 value
    input10_value: float = 0.0 # Generic input 10 value
    input11_freq: float = 0.0 # Generic input 11 (frequency input)
    input12_value: float = 0.0 # Generic input 12 value

class AxiomaticDevice(can_open_dev.CANOpenDevice):
    """
    A software device controller to manage a AXIOMATIC (battery thermal management system) device talking to it via CAN.
    """
    def __init__(self, config):
        self.stop = False
        self.start_time = None
        self.config = config
        
        # Store the threads we create for this device.
        self.threads = []

        # Call the base class constructor, CANDevice.
        super().__init__(config)

        # The rate which the state machine and CAN writing runs at
        self.write_period_ms = config['write_period_ms']
        # Controller outputs
        self.axiomaticcontrol_outputs = AxiomaticControlOutputs()
        # Controller command inputs
        self.axiomaticcontrol = AxiomaticControl()
        self.prev_axiomaticcontrol = AxiomaticControl()  # The previous state of the axiomatic control inputs. Allows for checking if the inputs have changed.
        self.axiomaticcontrol_input_ranges = AXIOMATICCONTROLINPUTRANGES # Define the allowed ranges for some of the control inputs from axiomaticcontrol.
        if "control_input_ranges" in config:
            self.axiomaticcontrol_input_ranges = config['control_input_ranges'] # The allowed input ranges for the controller control inputs.
        if "default_control_inputs" in config:
            self.axiomaticcontrol = AxiomaticControl(**config['default_control_inputs'])  # Take the default control inputs from the config file if they are defined.
        self.control_lock = Lock()  # A lock to protect the control data structure if multiple threads are updating it.
        self.outputs_lock = Lock()
        self.control_lock_aquire_timeout_ms = 1000 # The timeout in milliseconds to acquire the lock.
        self.set_control_inputs(self.axiomaticcontrol) # If some controls are set then make sure they propagate to the internal variables.
        self.allow_controls = True # Allow the control inputs to be set. If false then the control inputs will be ignored.




        # Local input and output control variables to prevent race conditions. 
        # Inputs will be updated at the beginning of the step_statemachine function.
        # Outputs will be updated at the end of the step_statemachine function.
        self.local_axiomaticcontrol = AxiomaticControl()
        self.local_axiomaticcontrol_outputs = AxiomaticControlOutputs()

        ########################### Axiomatic State Machine setup ############################
        ### Setup the state machine for the Axiomatic device.
        ### IMPORTANT NOTE: Do not use snake case for state names.
        ### The underscore means the state is a sub state of a hierarchical state machine. Use camelCase instead for state names and transitions.
        axiomatic_states = ["initialising", "standby", "active", "fault"]
        axiomatic_states_trans = [
            {
                "trigger": "initalisingDone",
                "source": "initialising",
                "dest": "standby",
                "conditions": [self.is_initialising_done],
            },
            {
                "trigger": "enable",
                "source": "standby",
                "dest": "active",
                "conditions": [self.is_enabled],
            },
            {
                "trigger": "disable",
                "source": "active",
                "dest": "standby",
                "unless": [self.is_enabled],
            },
            {
                "trigger": "fault",
                "source": "standby",
                "dest": "fault",
                "conditions": [self.is_fault],
            },
            {
                "trigger": "fault",
                "source": "active",
                "dest": "fault",
                "conditions": [self.is_fault],
            },
            {
                "trigger": "clearFaults",
                "source": "fault",
                "dest": "initialising",
                "conditions": [self.is_clear_fault],
            }
        ]
        self.state_machine = state_diagrams.CustomHierarchicalMachine(
            states=axiomatic_states,
            transitions=axiomatic_states_trans,
            initial="initialising",       # Default state is initialising.
            ignore_invalid_triggers=True,  # Ignore invalid triggers so we can call them and they will just do nothing if the correct source state is not active.

        )

        # Disable all logging from transitions except for errors.
        # This is required otherwise we get a warning when calling transitions which aren't valid for the current state.
        # e.g "Can't trigger event 'enable' from state(s) initialising!"
        # In this case we just want the state machine to silently not do anything.
        getLogger("transitions").setLevel(logging.ERROR)
        
        # Setup some on enter callbacks for some of the states. 
        self.state_machine.states["initialising"].on_enter = [
            self.on_enter_initialising
        ]

    ########################### AXIOMATIC State Machine Transition Conditions ##########################
    ### The following functions define the conditions that need to be satisfied for a
    ### transition from one state to another to occur.
    ###############################################################################################
    def is_initialising_done(self):
        # If the last CAN message indicating the AXIOMATIC device is in the PRE-OPERATIONAL mode and the
        #  flag self.can_device_configured is true then we are done initialising.
        initialising_done = (self.node.nmt.state == 'PRE-OPERATIONAL') 
        # The clearFaults inputs should not be set.
        initialising_done = initialising_done and not self.axiomaticcontrol.clearFaults
        # self.can_device_configured is set when the CAN open device is reported to have the correct configuration.
        return initialising_done and self.can_device_configured
    
    def is_enabled(self):
        # This function idicates if the AXIOMATIC device is commanded to be enabled via higher level controllers.
        return self.axiomaticcontrol.enable

    def is_fault(self):
        # If can_open device state is in STOPPED, SLEEP, STANDBY then we assume a fault state.
        # This is because the AXIOMATIC device is not responding to CAN messages.
        fault_present = False
        can_open_state = self.get_canopen_state()
        if can_open_state in ['STOPPED', 'SLEEP', 'STANDBY']:
            fault_present = True

        # Check state of can bus
        can_bus_fault = not self.check_comms_healthy()
        if fault_present or can_bus_fault:
            log.info(f"Axiomatic device fault present: {fault_present}, can bus fault: {can_bus_fault}")
        return fault_present or can_bus_fault

    def is_clear_fault(self):
        # If the clear faults input is high then we are clearing faults.
        cmd_clear_faults_enabled = self.axiomaticcontrol.clearFaults
        return cmd_clear_faults_enabled and self.check_comms_healthy()
    
    def on_enter_initialising(self):
        """
        Callback for when the state machine enters the initialising state.
        """
        # Clear the configured flag so we will retry configuration.
        self.can_device_configured = False  
        # Clear the outgoing CAN messages
        self.remove_all_out_can_msgs()

    ###############################################################################################

    def step_statemachine(self, now):
        """
        Run one iteration of the state machine.
        Send required CAN messages based on the current state

        Returns:
            state (str): The current state the state machine is in.
        """
        # Update local control inputs
        self._update_local_control_inputs()
        # Apply any transitions that are triggered by the inputs (they will only trigger if the conditions are valid and the correct state is active).
        # Add all transitions here that are triggered by inputs.
        self.state_machine.initalisingDone()
        self.state_machine.enable()
        self.state_machine.disable()
        self.state_machine.fault()
        self.state_machine.clearFaults()

        # Get the current state of the state machine.
        state = self.state_machine.state
        # Get input signals status.
        self._get_DI_status()
        self._get_AI_status()
        # State logic goes here.
        # Send CAN messages based on the current state.
        if state == "initialising":
            # Update local outputs for initialising state
            self.local_axiomaticcontrol_outputs.device_controller_ready = False
            self.local_axiomaticcontrol_outputs.device_fault = False
            canopen_state = self.get_canopen_state()
            if canopen_state != 'PRE-OPERATIONAL':
                # Send the set can open PRE-OPERATIONAL state command.
                self.set_canopen_state('PRE-OPERATIONAL')
                log.info(f"Set canopen state to PRE-OPERATIONAL, current state is {canopen_state}")
            
            if canopen_state == 'PRE-OPERATIONAL':
                if not self.can_device_configured:
                    # If the can open state is PRE-OPERATIONAL then we can load the configuration.
                    self.can_device_configured = self.load_axiomatic_config()
                    self.can_device_configured = True
                    log.info(f"Loaded canopen configuration, can_device_configured = {self.can_device_configured}")

        elif state == "standby":
            # Update local outputs for standby state
            self.local_axiomaticcontrol_outputs.device_fault = False
            self.local_axiomaticcontrol_outputs.device_controller_ready = True
            if self.get_canopen_state() != 'PRE-OPERATIONAL':
                self.set_canopen_state('PRE-OPERATIONAL')

        elif state == "active":
            # Update local outputs for active state
            self.local_axiomaticcontrol_outputs.device_fault = False
            self.local_axiomaticcontrol_outputs.device_controller_ready = True
            canopen_state = self.get_canopen_state()
            if canopen_state != 'OPERATIONAL':
                self.set_canopen_state('OPERATIONAL')
                log.info(f"Set canopen state to OPERATIONAL, current state is {canopen_state}")
            else:
                # Read one SDO from the device to check if the device is responding.
                if not self.disable_can_err_count_check:  # Only do this if we care about checking for healthy can comms with the device.
                    self._read_sdo_value("Error Register", log_to_db=True)


                # No output control messages to send anymore

        elif state == "fault":
            self.local_axiomaticcontrol_outputs.device_fault = True
            self.local_axiomaticcontrol_outputs.device_controller_ready = False
            if self.local_axiomaticcontrol.clearFaults:
                self.clear_can_error_count()

        else:
            log.error(f"Invalid state {state}")

        # Save the current control inputs as the previous control inputs.
        self.prev_axiomaticcontrol = replace(self.axiomaticcontrol)
        # Update shared outputs from local outputs
        self._update_control_outputs()
        return state
    


    def _get_DI_status(self):
        """
        Get the status of the DI signals.
        """
        # GPIO-based contactor feedback has been removed
        # Button status checking has been removed
    


    def _get_AI_status(self):
        """
        Get the status of the AI signals.
        """
        # Read all inputs from TPDO1
        self.local_axiomaticcontrol_outputs.input1_value = self.get_can_sig_value("Com001_TPDO1.input1_value")
        self.local_axiomaticcontrol_outputs.input2_value = self.get_can_sig_value("Com001_TPDO1.input2_value")
        self.local_axiomaticcontrol_outputs.input3_value = self.get_can_sig_value("Com001_TPDO1.input3_value")
        self.local_axiomaticcontrol_outputs.input4_value = self.get_can_sig_value("Com001_TPDO1.input4_value")

        # Read all inputs from TPDO2
        self.local_axiomaticcontrol_outputs.input5_value = self.get_can_sig_value("Com001_TPDO2.input5_value")
        self.local_axiomaticcontrol_outputs.input6_value = self.get_can_sig_value("Com001_TPDO2.input6_value")
        self.local_axiomaticcontrol_outputs.input7_value = self.get_can_sig_value("Com001_TPDO2.input7_value")
        self.local_axiomaticcontrol_outputs.input8_value = self.get_can_sig_value("Com001_TPDO2.input8_value")

        # Read all inputs from TPDO3
        self.local_axiomaticcontrol_outputs.input9_value = self.get_can_sig_value("Com001_TPDO3.input9_value")
        self.local_axiomaticcontrol_outputs.input10_value = self.get_can_sig_value("Com001_TPDO3.input10_value")
        self.local_axiomaticcontrol_outputs.input11_freq = self.get_can_sig_value("Com001_TPDO3.input11_freq")
        self.local_axiomaticcontrol_outputs.input12_value = self.get_can_sig_value("Com001_TPDO3.input12_value")

    def load_axiomatic_config(self):
        # Check for differences between the loaded config and the desired configuration of the device.
        # Try to set the canopen device to the desired configuration.
        return self.configure_object_dict_config(self.config)
    
    def get_state(self):
        """
        Get the current state of the AXIOMATIC controller.
        """
        return self.state_machine.state
    
    def get_faults(self):
        """
        Get the current faults of the AXIOMATIC device.
        """
        faults = ''
        with self.outputs_lock:

            if not self.can_comms_healthy:
                faults += 'CAN Comms Fault, '
        return faults
    
    def get_status(self):
        """
        Get the current status of the AXIOMATIC device.
        """
        status = {}
        # Add the state of the axiomatic device controller
        status['state'] = self.state_machine.state    
        status['allow_controls'] = self.allow_controls  # Show wether the control inputs can be set or not.    
        status['canopen_state'] = self.get_canopen_state()
        status['can_comms_healthy'] = self.can_comms_healthy
        status['can_interface'] = self.can_interface  # Show the can interface the device is connected to.
        status['faults'] = self.get_faults()
        
        # Safely read control requests and outputs
        res = self.control_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)
        if res:
            try:
                pass  # No control inputs to read anymore
            finally:
                self.control_lock.release()
        res = self.outputs_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)
        if res:
            try:
                status['device_fault'] = self.axiomaticcontrol_outputs.device_fault
                status['device_controller_ready'] = self.axiomaticcontrol_outputs.device_controller_ready
                
                # Generic inputs
                status['input1_value'] = self.axiomaticcontrol_outputs.input1_value
                status['input2_value'] = self.axiomaticcontrol_outputs.input2_value
                status['input3_value'] = self.axiomaticcontrol_outputs.input3_value
                status['input4_value'] = self.axiomaticcontrol_outputs.input4_value
                status['input5_value'] = self.axiomaticcontrol_outputs.input5_value
                status['input6_value'] = self.axiomaticcontrol_outputs.input6_value
                status['input7_value'] = self.axiomaticcontrol_outputs.input7_value
                status['input8_value'] = self.axiomaticcontrol_outputs.input8_value
                status['input9_value'] = self.axiomaticcontrol_outputs.input9_value
                status['input10_value'] = self.axiomaticcontrol_outputs.input10_value
                status['input11_freq'] = self.axiomaticcontrol_outputs.input11_freq
                status['input12_value'] = self.axiomaticcontrol_outputs.input12_value
            finally:
                self.outputs_lock.release()
        return status
    
    def unlock_allow_controls(self, allow: bool):
        """
        Allow the control inputs to be set. If false then the control inputs will be ignored.
        """
        self.allow_controls = allow

    def set_control_inputs(self, axiomaticcontrol: Union[AxiomaticControl, dict], unlock_controls: bool = False):
        """
        Set the AXIOMATIC control inputs to new values.
        This is how the AXIOMATIC device controller is controlled from the outside world.
        Thread safe
        
        Args:
            axiomaticcontrol (AxiomaticControl or dict): The new AXIOMATIC control input values to set. 
                                                         Can be of type AxiomaticControl or a dictionary that contains all or part of the same attributes as AxiomaticControl.
                                                         NOTE: If you pass a dict containing some of the attributes of AxiomaticControl then only those attributes will be updated and the others will remain unchanged.
             unlock_controls (bool): If True then the control inputs will be set even if allow_controls is False which is locking the controls.
        """
        # Check self.allow_controls has been created (this call may happen before the constructor has finished).
        if hasattr(self, 'allow_controls'):
            if unlock_controls or self.allow_controls:
                res = self.control_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)  # Try up to a maximum timeout to get the lock.
                if res:
                    if isinstance(axiomaticcontrol, dict):
                        self.axiomaticcontrol.set_from_dict(axiomaticcontrol, self.axiomaticcontrol_input_ranges)
                    else:
                        self.axiomaticcontrol.set_all(axiomaticcontrol, self.axiomaticcontrol_input_ranges)
                    # Connect some control inputs to internal variables.
                    self.can_read_only = self.axiomaticcontrol.can_read_only
                    self.control_lock.release()
                else:
                    log.error(f"Failed to set AXIOMATIC control inputs. Could not acquire lock after {self.control_lock_aquire_timeout_ms}ms.")
            else:
                log.warning("Tried to set AXIOMATIC control inputs but allow_controls is False. Ignoring.") 

    def get_control_inputs(self, show_hidden: bool = False) -> dict:
        """
        Get the values of the AXIOMATIC control inputs.
        Thread safe
        
        Returns:
            Dict: The AXIOMATIC control inputs as a dict of values. Empty if the lock could not be acquired.
        """
        axiomaticcontrol_dict = {}
        res = self.control_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)
        if res:
            # Return the control inputs as a dict.
            if self.show_controls is None or show_hidden: 
                axiomaticcontrol_dict = self.axiomaticcontrol.get_as_dict() # Get all controls
            else:
                # Filter out the controls we don't want to show.
                axiomaticcontrol_dict = self.axiomaticcontrol.get_as_dict(self.show_controls)
            self.control_lock.release()
        else:
            log.error(f"Failed to get AXIOMATIC control inputs. Could not acquire lock after {self.control_lock_aquire_timeout_ms}ms.")
        return axiomaticcontrol_dict

    def get_control_outputs(self):
        """
        Get the values of the AXIOMATIC control outputs.
        Thread safe
        
        Returns:
            dict: The AXIOMATIC control outputs as a dict of values. Empty if the lock could not be acquired.
        """
        axiomaticcontrol_outputs_dict = {}
        res = self.outputs_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)
        if res:
            axiomaticcontrol_outputs_dict = self.axiomaticcontrol_outputs.__dict__.copy()
            self.outputs_lock.release()
        else:
            log.error(f"Failed to get AXIOMATIC control outputs. Could not acquire lock after {self.control_lock_aquire_timeout_ms}ms.")
        return axiomaticcontrol_outputs_dict
    
    def _update_local_control_inputs(self):
        """
        Thread safe update of the local control inputs with the current control inputs.
        """
        timeout = self.control_lock_aquire_timeout_ms / 1000.0
        if self.control_lock.acquire(timeout=timeout):
            self.local_axiomaticcontrol = replace(self.axiomaticcontrol)
            self.control_lock.release()
        else:
            log.error(f"Failed to update AXIOMATIC control inputs. Could not acquire lock after {self.control_lock_aquire_timeout_ms}ms.")
            
    def _update_control_outputs(self):
        """
        Thread-safe update of the public `axiomaticcontrol_outputs` instance
        with the contents of `local_axiomaticcontrol_outputs`.
        """
        timeout = self.control_lock_aquire_timeout_ms / 1000.0
        if self.outputs_lock.acquire(timeout=timeout):
            try:
                self.axiomaticcontrol_outputs = replace(self.local_axiomaticcontrol_outputs)
            finally:
                self.outputs_lock.release()
        else:
            log.error(f"Failed to update AXIOMATIC control outputs. Could not acquire lock after {self.control_lock_aquire_timeout_ms}ms.")

    def get_control_input_ranges(self):
        """
        Get the ranges of the AXIOMATIC control inputs.
        """
        return self.axiomaticcontrol_input_ranges

    def start(self):
        """
        Starts all threads for the AXIOMATIC device controller.
        It appends all threads to the self.threads list.
        Some come from the base class and some are defined in this class.

        """
        self.start_time = time.monotonic()
        # Start a CAN open comms. Read CAN bus messages and load canopen config.
        self.connect_to_can_bus()
        # Start the state machine and CAN writing thread. This runs the state machine and writes CAN messages according to the state.
        st_write_can_thread = threading.Thread(target=lambda: self._start_statemachine(), name=self.name, daemon=True)
        self.threads.append(st_write_can_thread)

        # Start the threads
        [t.start() for t in self.threads]

        log.info(f"Started {self.name} device.")

    def _start_statemachine(self):

        while not self.stop:
            loop_start = time.monotonic() - self.start_time  # Now is the time relative to the start time.
            # Run the state machine at the specified control loop rate.

            if self.can_bus is None:  # No interface connected.
                self.connect_to_can_bus()
            state = self.step_statemachine(loop_start)
            log.debug(f"State: {state}")
            # Try sleep for the remaining time of the polling period.
            loop_end = time.monotonic() - self.start_time
            sleep_time = self.write_period_ms/1000.0 - (loop_end - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                log.warning(f'Took longer than polling period by {-1.0*sleep_time}')



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--config', type=str, default='', help='An optional path to a config file if none is provided a default path is used.')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.FullLoader)
            config = config["axio_dev"][0]  # Get the first device config
        axiomatic = AxiomaticDevice(config)
        axiomatic.start()

        time.sleep(100)
        log.info("Stopped axiomatic CAN comms")
        time.sleep(5)
        log.info("Exiting axiomatic_dev")

    except (KeyboardInterrupt, SystemExit):
        log.error("Exiting")