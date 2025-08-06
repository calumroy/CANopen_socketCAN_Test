# -----------------------------------------------------------
# Python utility functions for the supplyrack Testbench software
#
# (C) 2024 Switch, Perth, Australia
# -----------------------------------------------------------

import os
from logging import getLogger

log = getLogger('util_functions')

def simple_moving_average(new_value, old_avg, update_period, ma_time):
    """Calculate a simple moving average of a value.

    This function calculates a new moving average given a new value, the old average, 
    an update period, and the time over which the moving average is calculated.

    The ma_time is the time in seconds over which the moving average is calculated.
    If it is set as 10 secs then for a step change in the value, the moving average
    will reach 63% of the step change in 10 secs. The moving average will reach 95%
    of the step change in 20 secs.

    Args:
        new_value (float): The new value to update the moving average with.
        old_avg (float): The previous moving average.
        update_period (float): The period of the update in seconds.
        ma_time (float): The time over which the moving average is calculated, in seconds.

    Returns:
        float: The updated moving average.

    Raises:
        ValueError: If ma_time is less than or equal to 0.
    """
    
    if ma_time <= 0:
        return new_value
    else:
        # update_period is in the same units as the ma_time. It is expected that the update period is less than the ma_time.
        alpha = update_period / ma_time
        if alpha > 1:
            alpha = 1
        if alpha < 0:
            alpha = 0
        new_avg = (1 - alpha) * old_avg + alpha * new_value
        return new_avg
    
def flatten_directory_structure(directory, parent_path='', depth=0, file_types=None, exclude_dirs=None):
    """Flatten a directory structure into a list of file paths.

    This function traverses a given directory structure and flattens it into a dictionary
    where keys are directory paths and values are lists of file paths. It can filter by file types
    and exclude specific directories.

    Args:
        directory (dict): A dictionary representing the directory structure.
        parent_path (str): The path of the parent directory. Default is an empty string.
        depth (int): The current depth in the directory structure. Default is 0.
        file_types (list of str): Optional list of file extensions to include. Default is None.
        exclude_dirs (list of str): Optional list of directories to exclude. Default is None.

    Returns:
        dict: A dictionary representing the flattened directory structure.
    """
    flat_structure = {}
    for name, content in directory.items():
        current_path = os.path.join(parent_path, name)
        # Skip excluded directories
        if exclude_dirs and current_path in exclude_dirs:
            continue
        if depth == 0:
            if name == 'files':
                files = [os.path.join(parent_path, file) for file in content]
                if file_types:
                    files = [file for file in files if any(file.endswith('.' + ext) for ext in file_types)]
                flat_structure[parent_path] = files
            else:
                flat_structure[name] = flatten_directory_structure(content, current_path, depth + 1, file_types, exclude_dirs)
        else:
            if name == 'files':
                file_paths = [os.path.join(parent_path, file) for file in content]
                if file_types:
                    file_paths = [file for file in file_paths if any(file.endswith('.' + ext) for ext in file_types)]
                if parent_path in flat_structure:
                    flat_structure[parent_path].extend(file_paths)
                else:
                    flat_structure[parent_path] = file_paths
            else:
                sub_flat_structure = flatten_directory_structure(content, current_path, depth + 1, file_types, exclude_dirs)
                flat_structure = {**flat_structure, **sub_flat_structure}

    return flat_structure


def get_directory_structure(rootdir):
    """Get the structure of a directory as a nested dictionary.

    This function creates a nested dictionary that represents the folder structure
    of a specified directory.

    Args:
        rootdir (str): The root directory whose structure is to be obtained.

    Returns:
        dict: A nested dictionary representing the folder structure.
    """
    structure = {'files': []}  # Initialize with a 'files' key
    # List all entries in directory
    for entry in os.scandir(rootdir):
        if entry.is_dir():
            # Recursively get the structure of the subdirectory
            structure[entry.name] = get_directory_structure(os.path.join(rootdir, entry.name))
        elif entry.is_file():
            # Add file to the 'files' key
            structure['files'].append(entry.name)

    # If there are no files in the directory, remove the 'files' key
    if not structure['files']:
        del structure['files']

    return structure