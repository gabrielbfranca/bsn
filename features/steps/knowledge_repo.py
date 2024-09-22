from behave import given, when, then
import subprocess

# Function to check if ROS is running
def check_ros_is_running():
    result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return result.returncode == 0

@given('the ROS environment is up')
def step_ros_up(context):
    assert check_ros_is_running(), "ROS is not running."

@when('I check if /g4t1 is publishing data')
def step_check_g4t1_publishing(context):
    g4t1_node = '/g4t1'
    try:
        result = subprocess.run(['rostopic', 'echo', g4t1_node, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.g4t1_data = result.stdout.decode('utf-8')
        assert context.g4t1_data, f"No data published on topic {g4t1_node}"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: No data published on topic {g4t1_node}")

@then('/data_access should receive data from /g4t1')
def step_data_access_receives_from_g4t1(context):
    data_access_topic = '/TargetSystemData'
    try:
        result = subprocess.run(['rostopic', 'echo', data_access_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.data_access_g4t1_data = result.stdout.decode('utf-8')
        assert context.data_access_g4t1_data, "No data received by /data_access from /g4t1"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /data_access did not receive data from /g4t1 on {data_access_topic}")

@when('I check if /logger is publishing data')
def step_check_logger_publishing(context):
    logger_topic = '/logger'
    try:
        result = subprocess.run(['rostopic', 'echo', logger_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.logger_data = result.stdout.decode('utf-8')
        assert context.logger_data, f"No data published on topic {logger_topic}"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: No data published on topic {logger_topic}")

@then('/data_access should receive data from /logger')
def step_data_access_receives_from_logger(context):
    data_access_topic = '/persist'
    try:
        result = subprocess.run(['rostopic', 'echo', data_access_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.data_access_logger_data = result.stdout.decode('utf-8')
        assert context.data_access_logger_data, "No data received by /data_access from /logger"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /data_access did not receive data from /logger on {data_access_topic}")
