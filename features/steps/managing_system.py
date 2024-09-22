from behave import given, when, then
import subprocess

# Function to check if ROS is running
def check_ros_is_running():
    result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return result.returncode == 0

@given('the ROS environment is up')
def step_ros_up(context):
    assert check_ros_is_running(), "ROS is not running."

@when('I check if /loggers are publishing data')
def step_check_loggers_publishing(context):
    logger_topics = ['/logger1', '/logger2', '/logger3', '/reconfigure']
    context.logger_data = {}

    for topic in logger_topics:
        try:
            result = subprocess.run(['rostopic', 'echo', topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            context.logger_data[topic] = result.stdout.decode('utf-8')
            assert context.logger_data[topic], f"No data published on topic {topic}"
        except subprocess.TimeoutExpired:
            raise AssertionError(f"Timeout: No data published on topic {topic}")

@then('/enactor should receive data from /loggers')
def step_enactor_receives_data(context):
    enactor_topic = '/enactor_input'
    try:
        result = subprocess.run(['rostopic', 'echo', enactor_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.enactor_data = result.stdout.decode('utf-8')
        assert context.enactor_data, "No data received by /enactor from /loggers"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /enactor did not receive data from /loggers on {enactor_topic}")

@when('/enactor processes the data')
def step_enactor_processing_data(context):
    # Assume /enactor processes data after receiving it, no additional action needed in this step
    pass

@then('/enactor should publish data to /reli_engine')
def step_enactor_publishes_to_reli_engine(context):
    reli_engine_topic = '/exception'
    try:
        result = subprocess.run(['rostopic', 'echo', reli_engine_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.reli_engine_data = result.stdout.decode('utf-8')
        assert context.reli_engine_data, "No data published by /enactor to /reli_engine"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /enactor did not publish to /reli_engine on {reli_engine_topic}")

@then('/reli_engine should receive data from /enactor')
def step_reli_engine_receives_data(context):
    reli_engine_topic = '/reli_engine_input'
    try:
        result = subprocess.run(['rostopic', 'echo', reli_engine_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.reli_engine_data_received = result.stdout.decode('utf-8')
        assert context.reli_engine_data_received, "No data received by /reli_engine from /enactor"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /reli_engine did not receive data from /enactor on {reli_engine_topic}")

@then('/reli_engine should publish data back to /enactor')
def step_reli_engine_publishes_back_to_enactor(context):
    enactor_topic = '/enactor_output'
    try:
        result = subprocess.run(['rostopic', 'echo', enactor_topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
        context.enactor_output_data = result.stdout.decode('utf-8')
        assert context.enactor_output_data, "No data published back to /enactor by /reli_engine"
    except subprocess.TimeoutExpired:
        raise AssertionError(f"Timeout: /reli_engine did not publish data back to /enactor on {enactor_topic}")
