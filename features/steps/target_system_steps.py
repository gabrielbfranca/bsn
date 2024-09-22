from behave import given, when, then
import subprocess
from utils.parsers import get_rostopic_sensor_data, get_rosnode_info
    
def is_node_receiving_data(node_name):
    # Check if a node is subscribing to a topic (this would need rostopic or rosnode info)
    result = subprocess.run(['rosnode', 'info', node_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return result.returncode == 0 and "Subscriptions" in result.stdout.decode('utf-8')
def save_topics_nodes(context):
    context.sensor_nodes = ['/g3t1_6', '/g3t1_5', '/g3t1_4','/g3t1_3', '/g3t1_2', '/g3t1_1']
    context.sensor_topics = ['/abpd_data', '/abps_data', '/ecg_data','/glucosemeter_data', '/oximeter_data', '/thermometer_data']
@given('the ROS environment is up')
def step_ros_up(context):
    # Launch ROS or check if it is already running
    try:
        result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        save_topics_nodes(context)
        assert result.returncode == 0, "ROS is not running." + result.stderr.decode('utf-8')
    except subprocess.CalledProcessError as e:
        raise AssertionError(f"Failed to check ROS environment: {e}")
    
@when('I check if the sensors are publishing data')
def step_check_sensors_publishing(context):
    context.node_data = {}

    for node in context.sensor_nodes:
        try:
            node_data = subprocess.run(['rostopic', 'info', node], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            context.sensor_data[node] = get_rosnode_info(node_data)
            assert context.sensor_data[node]['publications'], f"No data published on topic {node}"
            assert context.sensor_data[node]['connections'], f"No connections in node {node}"
        except subprocess.TimeoutExpired:
            raise AssertionError(f"Timeout: No data published on topic {node}")

@when('I check if respective topics have data')
def step_check_topics_have_data(context):
    context.sensor_data = {}
    for topic in context.sensor_topics:
        try:
            topic_data = subprocess.run(['rostopic', 'echo', topic, '-n', '1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
            context.sensor_data[topic] = get_rostopic_sensor_data(topic_data)
            assert context.sensor_data[topic]['data'], f"No data published on topic {topic}"
        except subprocess.TimeoutExpired:
            raise AssertionError(f"Timeout: No data published on topic {topic}")

@when('I check if /g4t1 is receiving data')
def step_check_g4t1_receiving_data(context):
    assert is_node_receiving_data('/g4t1'), "/g4t1 is not receiving data"

@then('the sensors should publish data on their respective topics')
def step_sensors_should_publish(context):
    # Confirm that the sensors' topics are actively publishing
    for topic in context.sensor_topics:
        assert is_topic_publishing_data(topic), f"Topic {topic} is not publishing data"

@when('the /g4t1 node is publishing data')
def step_g4t1_publishing_data(context):
    assert is_topic_publishing_data('/TargetSystemData'), "/g4t1 is not publishing to /TargetSystemData"

@when('respective topics have data')
def step_respective_topics_have_data(context):
    assert is_topic_publishing_data('/TargetSystemData'), "/TargetSystemData does not have data"

@then('the /data_access node should received it')
def step_data_access_should_receive(context):
    assert is_node_receiving_data('/data_access'), "/data_access is not receiving data"

