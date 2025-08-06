# -----------------------------------------------------------
# A base class defining control inputs for different device 
# and testbench modules in the supplyrack Testbench software.
#
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------


from logging import getLogger
import copy

log = getLogger("ControlInputs")


class ControlInputs:
    
    def set_all(self, new_instance, ranges_dict=None):
        """
        Set all the attributes of the current instance to the values of the attributes of the new instance.
        Check that the values of the attributes of the new instance are within the allowed ranges defined in the ranges_dict.
        Args:
            new_instance: The new instance of the dataclass type set the attributes of the current instance to.
            ranges_dict: A dictionary of the allowed ranges for the attributes of the new instance.
                         E.g {"var1": {"min": 0.0, "max": 100.0},
                              "var2": ["var2_state1", "var2_state2"]}
        """
        if isinstance(new_instance, type(self)):  # check if new_instance is the same type as current instance
            for attr, value in vars(new_instance).items():
                value_in_range = True
                if ranges_dict is not None and attr in ranges_dict and isinstance(value, (int, float)):
                    min_val = ranges_dict[attr]['min']
                    max_val = ranges_dict[attr]['max']
                    if not min_val <= value <= max_val:
                        log.error(f'{attr} with value: {value} out of range, min: {min_val}, max: {max_val}')
                        value_in_range = False
                elif ranges_dict is not None and attr in ranges_dict and isinstance(value, str):
                    allowed_vals = ranges_dict[attr]
                    if not value in allowed_vals:
                        log.error(f'{attr} with value: {value} out of range, allowed values: {allowed_vals}')
                        value_in_range = False
                if value_in_range:
                    setattr(self, attr, value)
                else:
                    log.error(f"Could not set attribute: {attr} to value: {value} of new instance.")
        else:
            log.error(f"The given instance {new_instance} is not of correct type.")

    def set_from_dict(self, new_controls: dict, ranges_dict=None):
        """
        Given a dictionary of control inputs, set the attributes of the current instance to the values of the dictionary.
        Only set the attributes that are in the dictionary i.e not all attributes may be defined in the dictionary.
        Check that the values of the attributes of the new instance are within the allowed ranges defined in the ranges_dict.
        Args:
            new_controls: A dictionary of the control inputs to set the attributes of the current instance to.
            ranges_dict: A dictionary of the allowed ranges for the attributes of the new instance.
                            E.g {"var1": {"min": 0.0, "max": 100.0},
                                 "var2": ["var2_state1", "var2_state2"]} 
        """
        # Get a copy of the current instance. We set this copies values and then if all values are in range we set the current instance to the copy.
        current_instance = copy.deepcopy(self)
        all_values_in_range = True

        for attr, value in new_controls.items():
            value_in_range = True
            value_cast = value
            if ranges_dict is not None and attr in ranges_dict and isinstance(getattr(self, attr), int):
                min_val = ranges_dict[attr]['min']
                max_val = ranges_dict[attr]['max']
                value_cast = int(value)
                if value_cast < min_val:
                    log.error(f'{attr} with value: {value_cast} out of range, min: {min_val}, max: {max_val}')
                    value_in_range = False
                if value_cast > max_val:
                    log.error(f'{attr} with value: {value_cast} out of range, min: {min_val}, max: {max_val}')
                    value_in_range = False
            elif ranges_dict is not None and attr in ranges_dict and isinstance(getattr(self, attr), float):
                min_val = ranges_dict[attr]['min']
                max_val = ranges_dict[attr]['max']
                value_cast = float(value)
                if value_cast < min_val:
                    log.error(f'{attr} with value: {value_cast} out of range, min: {min_val}, max: {max_val}')
                    value_in_range = False
                if value_cast > max_val:
                    log.error(f'{attr} with value: {value_cast} out of range, min: {min_val}, max: {max_val}')
                    value_in_range = False
            elif ranges_dict is not None and attr in ranges_dict and isinstance(getattr(self, attr), str):
                allowed_vals = ranges_dict[attr]
                value_cast = str(value)
                if not value in allowed_vals:
                    log.error(f'set dict {attr} with value: {value_cast} out of range, allowed values: {allowed_vals}')
                    log.error(f'current value: {getattr(self, attr)}')
                    value_in_range = False
            elif attr not in ranges_dict:
                # Cast the value to the type of the current instance attribute if this attr is in the current instance of the dataclass.
                if attr in current_instance.__dataclass_fields__:
                    value_cast = type(getattr(self, attr))(value)
                    value_in_range = True  # Since there was no range defined.
            if value_in_range:
                setattr(current_instance, attr, value_cast)
            else:
                log.error(f"Could not set attribute: {attr} to value: {value_cast} of new instance.")
            all_values_in_range = all_values_in_range and value_in_range
        # Now if all values are in range set the current instance to the copy
        if all_values_in_range:
            for field in current_instance.__dataclass_fields__:
                setattr(self, field, getattr(current_instance, field))
        else:
            log.error(f"Could not set any of the command inputs as at least one value was out of range.")

    def get_as_dict(self, filter_list: list = None) -> dict:
        """
        Return the attributes of the current instance as a dictionary.
        An optional argument can be given to filter the attributes to return.
        Args:
            filter_list (list): A list of strings defining the names of the attributes to return.    
        Returns 
            A dictionary of the attributes of the current instance.
        """
        if filter_list is None:
            return vars(self)
        else:
            return {attr: value for attr, value in vars(self).items() if attr in filter_list}
        