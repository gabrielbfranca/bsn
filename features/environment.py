import subprocess
import time

def before_scenario(context, scenario):
    if 'happy_path' in scenario.tags:
        # This block will run for happy path scenarios only
        if hasattr(context, 'sad_launch'):
            context.sad_launch.terminate()
            context.sad_launch.wait()
        # Launch the system (e.g., 'bsn.launch')
        if not hasattr(context, 'bsn_launch'):
            context.bsn_launch = subprocess.Popen(
                ['roslaunch', 'bsn.launch'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
            )
        time.sleep(30)  # Ensure the system is fully started before proceeding with the test

    elif 'inactive_central_hub' in scenario.tags:
        # This block will run for sad path scenarios where the central hub is inactive
        
        # Terminate the default launch process if needed
        if hasattr(context, 'bsn_launch'):
            context.bsn_launch.terminate()
            context.bsn_launch.wait()

        # Start a modified environment for the sad path
        context.sad_launch = subprocess.Popen(
            ['roslaunch', 'bsn.launch'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT
        )
        time.sleep(30)  # Ensure the system is fully started before proceeding with the test


def after_scenario(context, scenario):
    if 'happy_path' in scenario.tags:
        # Cleanup after happy path scenarios
        if hasattr(context, 'bsn_launch'):
            context.bsn_launch.terminate()
            context.bsn_launch.wait()
        
    elif 'inactive_central_hub' in scenario.tags:
        # Cleanup after sad path scenarios
        if hasattr(context, 'sad_launch'):
            context.sad_launch.terminate()
            context.sad_launch.wait()
