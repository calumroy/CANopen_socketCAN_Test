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
import numpy as np

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
    precharge_contactor_enable: bool = False # Enable the precharge contactor.
    main_contactor_enable: bool = False # Enable the main contactor.
    negative_contactor_enable: bool = False # Enable the negative contactor.
    relay_1_12v_enable: bool = False # Enable the 12v relay for inverter 1.
    relay_2_12v_enable: bool = False # Enable the 12v relay for inverter 2.

    # We ran out of DI channels on the Axiomatic device so we are using Odroid GPIOs to get the contactors feedback.
    odroid_gpio_k01_pos_contactor_closed: bool = False # The main contactor closed state from the Odroid GPIOs.
    odroid_gpio_k01_pos_contactor_open: bool = False # The main contactor open state from the Odroid GPIOs.
    odroid_gpio_k02_neg_contactor_closed: bool = False # The negative contactor closed state from the Odroid GPIOs.
    odroid_gpio_k02_neg_contactor_open: bool = False # The negative contactor open state from the Odroid GPIOs.

@dataclass
class AxiomaticControlOutputs():
    """
    A data class to store the Axiomatic control outputs.
    These are the outputs that other device controllers can read from the Axiomatic device controller.
    """
    device_controller_ready: bool = False # Indicates if the device is ready to be used.
    main_contactor_feedback: bool = False # Indicates if the main contactor is closed.
    negative_contactor_feedback: bool = False # Indicates if the negative contactor is closed.
    device_fault: bool = False # Indicates if the device is in a fault state.
    contactor_feedback_fault: bool = False  # Indicates if a feedback fault is detected.
    coolant_flow_LM: float = 0.0 # Coolant flow in liters per minute.
    system_enable_button: bool = False # Soft stop button, if the signal is disabled then the testbench will not run and stop any running processes.
    e_stop_button: bool = False # E-stop button, if the signal is disabled then the testbench will not run and stop any running processes.
    coolant_inv_1_temperature_out_C: float = 0.0 # Temperature of the coolant in the inverter 1 in degrees Celsius.
    coolant_inv_2_temperature_out_C: float = 0.0 # Temperature of the coolant in the inverter 2 in degrees Celsius.
    coolant_mot_1_temperature_out_C: float = 0.0 # Temperature of the coolant in the motor 1 in degrees Celsius.
    coolant_mot_2_temperature_out_C: float = 0.0 # Temperature of the coolant in the motor 2 in degrees Celsius.
    coolant_temperature_in_C: float = 0.0    # Input coolant temperature (all inputs combined)
    inv1_coolant_pressure_in_psi: float = 0.0 # Coolant pressure in psi.
    inv2_coolant_pressure_in_psi: float = 0.0 # Coolant pressure in psi.
    mot1_coolant_pressure_in_psi: float = 0.0 # Coolant pressure in psi.
    mot2_coolant_pressure_in_psi: float = 0.0 # Coolant pressure in psi.

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
        self.system_en_inverse_logic = config.get('system_enable_inverse_logic', False)

        self.contactor_status_timeout_ms = 1000 # The default timeout in milliseconds to wait for the contactor feedback to match the commanded contactor status.
        if "contactor_status_timeout_ms" in config:
            self.contactor_status_timeout_ms = config['contactor_status_timeout_ms']
        self.last_on_close_contactors_time = [None, None] # The last time the contactor was commanded to close, rising edge. A list of length 3 for each group contactors and the contactors box commands.
        self.open_close_feedback_fault = False # Indicates that the device controller is reading the wrong state of the contactors.
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
            show_state_attributes=True, show_auto_transitions=False, show_conditions=True
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
        fault_present |= self.open_close_feedback_fault
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

                self._check_contactors_command_matches_feedback(self.local_axiomaticcontrol, self.prev_axiomaticcontrol, self.contactor_status_timeout_ms)
                com001_rpdo1_msg = {"k01_pos_contactor_enable": self.local_axiomaticcontrol.main_contactor_enable,
                                    "k02_neg_contactor_enable": self.local_axiomaticcontrol.negative_contactor_enable,
                                    "k03_prechg_contactor_enable": self.local_axiomaticcontrol.precharge_contactor_enable,
                                    "inverter_1_12v_relay_enable": self.local_axiomaticcontrol.relay_1_12v_enable,
                                    "inverter_2_12v_relay_enable": self.local_axiomaticcontrol.relay_2_12v_enable,
                                    "DO_4_enable":0,
                                    "DO_5_enable":0,
                                    "DO_6_enable":0,
                }
                # Add the message to the outgoing CAN messages dict.
                self.add_out_can_msg_data("Com001_RPDO1", com001_rpdo1_msg)
                # # Actually send all the outgoing CAN messages.
                self.send_all_out_can_msgs()

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
    
    def _check_contactors_command_matches_feedback(self, axiomaticcontrol: AxiomaticControl, prev_axiomaticcontrol: AxiomaticControl, timeout_ms):
        """
            Check each contactor command against its feedback. If a mismatch persists
            longer than the specified timeout, disable that contactor command.
        """
        # Update local outputs for contactor feedback
        self.local_axiomaticcontrol_outputs.contactor_feedback_fault = False
        now = time.monotonic()
        # Check main contactor feedback
        if axiomaticcontrol.main_contactor_enable:
            if not prev_axiomaticcontrol.main_contactor_enable:
                self.last_on_close_contactors_time[0] = now

        if self.last_on_close_contactors_time[0] is not None:
            if (now - self.last_on_close_contactors_time[0]) > (timeout_ms / 1000.0):
                if axiomaticcontrol.main_contactor_enable != self.local_axiomaticcontrol_outputs.main_contactor_feedback:
                    log.error("Main contactor feedback mismatch.")
                    self.local_axiomaticcontrol_outputs.contactor_feedback_fault = True
                else:
                    self.last_on_close_contactors_time[0] = None
                        
        # Check negative contactor feedback
        if axiomaticcontrol.negative_contactor_enable:
            if not prev_axiomaticcontrol.negative_contactor_enable:
                self.last_on_close_contactors_time[1] = now
        if self.last_on_close_contactors_time[1] is not None:
            # Slightly shorter timeout for the contactors box
            if (now - self.last_on_close_contactors_time[1]) > (timeout_ms / 1000.0):
                if axiomaticcontrol.negative_contactor_enable != self.local_axiomaticcontrol_outputs.negative_contactor_feedback:
                    log.error("Negative contactor feedback mismatch.")
                    self.local_axiomaticcontrol_outputs.contactor_feedback_fault = True
                else:
                    self.last_on_close_contactors_time[1] = None

    def _get_DI_status(self):
        """
        Get the status of the DI signals.
        """
        main_contactor_closed = self.local_axiomaticcontrol.odroid_gpio_k01_pos_contactor_closed
        main_contactor_open = self.local_axiomaticcontrol.odroid_gpio_k01_pos_contactor_open
        negative_contactor_closed = self.local_axiomaticcontrol.odroid_gpio_k02_neg_contactor_closed 
        negative_contactor_open = self.local_axiomaticcontrol.odroid_gpio_k02_neg_contactor_open
        # Update local control outputs for DI status
        self.local_axiomaticcontrol_outputs.main_contactor_feedback = main_contactor_closed and not main_contactor_open
        self.local_axiomaticcontrol_outputs.negative_contactor_feedback = negative_contactor_closed and not negative_contactor_open
        self.open_close_feedback_fault =  main_contactor_closed and main_contactor_open
        self.open_close_feedback_fault |= negative_contactor_closed and negative_contactor_open
        system_en_button_status = self.get_can_sig_value("Com001_TPDO3.system_enable_button")
        e_stop_button_status = self.get_can_sig_value("Com001_TPDO3.e_stop_button")
        enable_satus = 1 if not self.system_en_inverse_logic else 0 
        self.local_axiomaticcontrol_outputs.system_enable_button = (system_en_button_status == enable_satus) and (e_stop_button_status == enable_satus)
        self.local_axiomaticcontrol_outputs.e_stop_button = e_stop_button_status != enable_satus
    
    def _steinhart_hart_temperature_conversion(self, resistance):
        """
        Convert resistance to temperature using Steinhart-Hart method
        Args:
            resistance (float): The resistance of the thermistor in ohms.
        Returns:
            float: The temperature in degrees Celsius.
        """
        # Values obtained using an online calculator for the Steinhart-Hart Method
        if resistance <= 0.0:
            return 0.0
        A, B, C = 1.26392655e-3, 2.67635298e-4, 1.11295539e-7
        T_kelvin = 1.0/(A + B*np.log(resistance) + C*(np.log(resistance)**3))
        return round(float(T_kelvin - 273.15), 2)

    def _get_AI_status(self):
        """
        Get the status of the AI signals.
        """
        ## Flow sensor (Frequency Input)
        # Q=7.7769f−14.8605(R2≈0.992) -> approx linear relationship
        flow_sensor_frequency = self.get_can_sig_value("Com001_TPDO3.flow_sensor_freq_Hz")
        # Apply conversion and convert to liters per second. (result is in liters per hour)
        flow_sensor_LM = (7.7769 * flow_sensor_frequency - 14.8605) / 60.0
        self.local_axiomaticcontrol_outputs.coolant_flow_LM = flow_sensor_LM
        ## Thermistors temperature conversion
        inv_1_coolant_thermistor_resistance = self.get_can_sig_value("Com001_TPDO1.thermistor_1_resist_inv1_coolant")
        inv_2_coolant_thermistor_resistance = self.get_can_sig_value("Com001_TPDO1.thermistor_2_resist_inv2_coolant")
        mot_1_coolant_thermistor_resistance = self.get_can_sig_value("Com001_TPDO1.thermistor_3_resist_mot1_coolant")
        mot_2_coolant_thermistor_resistance = self.get_can_sig_value("Com001_TPDO1.thermistor_4_resist_mot2_coolant")
        coolant_in_thermistor_resistance = self.get_can_sig_value("Com001_TPDO3.thermistor_5_resist_coolant_in")
        # Convert resistance to temperature using Steinhart-Hart method
        self.local_axiomaticcontrol_outputs.coolant_inv_1_temperature_out_C = self._steinhart_hart_temperature_conversion(inv_1_coolant_thermistor_resistance)
        self.local_axiomaticcontrol_outputs.coolant_inv_2_temperature_out_C = self._steinhart_hart_temperature_conversion(inv_2_coolant_thermistor_resistance)
        self.local_axiomaticcontrol_outputs.coolant_mot_1_temperature_out_C = self._steinhart_hart_temperature_conversion(mot_1_coolant_thermistor_resistance)
        self.local_axiomaticcontrol_outputs.coolant_mot_2_temperature_out_C = self._steinhart_hart_temperature_conversion(mot_2_coolant_thermistor_resistance)
        self.local_axiomaticcontrol_outputs.coolant_temperature_in_C = self._steinhart_hart_temperature_conversion(coolant_in_thermistor_resistance)
        ## Pressure sensors
        # -- Pressure sensor 1 (Inverter 1) --
        pressure_sensor_voltage = self.get_can_sig_value("Com001_TPDO2.pressure_sensor_voltage_V")
        # Convert voltage to psi
        supply_voltage_V = 5
        voltage_offset_V = pressure_sensor_voltage - (0.1 * supply_voltage_V)
        psi_max = 150
        psi_min = 0
        sensor_pressure_psi = (voltage_offset_V * (psi_max - psi_min)) / (0.8 * supply_voltage_V) + psi_min
        sensor_pressure_psi = max(0, sensor_pressure_psi)
        self.local_axiomaticcontrol_outputs.inv1_coolant_pressure_in_psi = round(sensor_pressure_psi, 2)
        # -- Pressure sensor 2 (Inverter 2) --
        pressure_sensor_voltage = self.get_can_sig_value("Com001_TPDO2.pressure_sensor_2_voltage_V")
        # Convert voltage to psi
        supply_voltage_V = 5
        voltage_offset_V = pressure_sensor_voltage - (0.1 * supply_voltage_V)
        psi_max = 150
        psi_min = 0
        sensor_pressure_psi = (voltage_offset_V * (psi_max - psi_min)) / (0.8 * supply_voltage_V) + psi_min
        sensor_pressure_psi = max(0, sensor_pressure_psi)
        self.local_axiomaticcontrol_outputs.inv2_coolant_pressure_in_psi = round(sensor_pressure_psi, 2)
        # -- Pressure sensor 3 (Motor 1) --
        pressure_sensor_voltage = self.get_can_sig_value("Com001_TPDO2.pressure_sensor_3_voltage_V")
        # Convert voltage to psi
        supply_voltage_V = 5
        voltage_offset_V = pressure_sensor_voltage - (0.1 * supply_voltage_V)
        psi_max = 150
        psi_min = 0
        sensor_pressure_psi = (voltage_offset_V * (psi_max - psi_min)) / (0.8 * supply_voltage_V) + psi_min
        sensor_pressure_psi = max(0, sensor_pressure_psi)
        self.local_axiomaticcontrol_outputs.mot1_coolant_pressure_in_psi = round(sensor_pressure_psi, 2)
        # -- Pressure sensor 4 (Motor 2) --
        pressure_sensor_voltage = self.get_can_sig_value("Com001_TPDO2.pressure_sensor_4_voltage_V")
        # Convert voltage to psi
        supply_voltage_V = 5
        voltage_offset_V = pressure_sensor_voltage - (0.1 * supply_voltage_V)
        psi_max = 150
        psi_min = 0
        sensor_pressure_psi = (voltage_offset_V * (psi_max - psi_min)) / (0.8 * supply_voltage_V) + psi_min
        sensor_pressure_psi = max(0, sensor_pressure_psi)
        self.local_axiomaticcontrol_outputs.mot2_coolant_pressure_in_psi = round(sensor_pressure_psi, 2)

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
            if self.open_close_feedback_fault:
                faults += 'Open/Close Feedback Fault, '
            if self.local_axiomaticcontrol_outputs.contactor_feedback_fault:
                faults += 'Contactor Feedback Fault, '
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
                status['pre_charge_contactor_request'] = self.axiomaticcontrol.precharge_contactor_enable
                status['main_contactor_request'] = self.axiomaticcontrol.main_contactor_enable
                status['negative_contactor_request'] = self.axiomaticcontrol.negative_contactor_enable
            finally:
                self.control_lock.release()
        res = self.outputs_lock.acquire(timeout=self.control_lock_aquire_timeout_ms/1000.0)
        if res:
            try:
                status['coolant_flow_LM'] = self.axiomaticcontrol_outputs.coolant_flow_LM
                status['coolant_temperature_in_C'] = self.axiomaticcontrol_outputs.coolant_temperature_in_C
                status['coolant_inv_1_temperature_out_C'] = self.axiomaticcontrol_outputs.coolant_inv_1_temperature_out_C
                status['coolant_inv_2_temperature_out_C'] = self.axiomaticcontrol_outputs.coolant_inv_2_temperature_out_C
                status['coolant_mot_1_temperature_out_C'] = self.axiomaticcontrol_outputs.coolant_mot_1_temperature_out_C
                status['coolant_mot_2_temperature_out_C'] = self.axiomaticcontrol_outputs.coolant_mot_2_temperature_out_C
                status['main_contactor_feedback'] = self.axiomaticcontrol_outputs.main_contactor_feedback
                status['negative_contactor_feedback'] = self.axiomaticcontrol_outputs.negative_contactor_feedback
                status['device_fault'] = self.axiomaticcontrol_outputs.device_fault
                status['device_controller_ready'] = self.axiomaticcontrol_outputs.device_controller_ready
                status['system_enable_button'] = self.axiomaticcontrol_outputs.system_enable_button
                status['e_stop_button'] = self.axiomaticcontrol_outputs.e_stop_button
                status['inv_1_coolant_in_pressure_psi'] = self.axiomaticcontrol_outputs.inv1_coolant_pressure_in_psi
                status['inv_2_coolant_in_pressure_psi'] = self.axiomaticcontrol_outputs.inv2_coolant_pressure_in_psi
                status['mot_1_coolant_in_pressure_psi'] = self.axiomaticcontrol_outputs.mot1_coolant_pressure_in_psi
                status['mot_2_coolant_in_pressure_psi'] = self.axiomaticcontrol_outputs.mot2_coolant_pressure_in_psi
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
    parser.add_argument('-c', '--config', type=str, default='', help=f'An optional path to a config file if none is provided a default path is used.')
    args = parser.parse_args()
    try:
        with open(args.config, 'r') as config_file:
            config = yaml.load(config_file, Loader=yaml.FullLoader)
            config = config["axiomatic_dev"]
        axiomatic = AxiomaticDevice(config)
        axiomatic.start()

        time.sleep(100)
        axiomatic.stop_comms()
        log.info("Stopped axiomatic CAN comms")
        time.sleep(5)
        log.info("Exiting axiomatic_dev")

    except (KeyboardInterrupt, SystemExit):
        log.error("Exiting")