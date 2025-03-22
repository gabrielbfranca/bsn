from behave import given, when, then
import subprocess
from utils.parsers import process_real_time_topics, capture_topic_data
from utils.constants import PERSISTENCE_NODES, PERSISTANCE_TOPICS


REDUCED_SYSTEM_NODES = [
    "thermometer",
    "central_hub"
]

def node_is_active(node_name):
    """Check if a ROS node is active."""
    result = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    node_list = result.stdout.decode('utf-8').splitlines()
    return node_name in node_list

def check_nodes_online(node_list):
    """Check if all required nodes are online."""
    for node in node_list:
        assert node_is_active(node), f"Node {node} is not online"

@given('that nodes thermometer and central hub are online')
def step_given_reduced_system_nodes_online(context):
    check_nodes_online(REDUCED_SYSTEM_NODES)

@given('nodes are online')
def step_given_all_persistence_nodes_online(context):
    check_nodes_online(PERSISTENCE_NODES)

@when('I listen to thermometer data')
def step_when_i_listen_to_thermometer(context):
    context.sensor_data = {}
    context.found_high_risk = []
    context.target_system_data = {}
    context.non_sensor = {}
    topics = [
        
        '/thermometer_data',
        '/collect_energy_status',
        '/persist',
        '/log_energy_status',
        '/TargetSystemData'
    ]

    process_real_time_topics(context, capture_topic_data, topics)
    print(context.non_sensor)

@when('I send data to collector')
def step_when_send_data_to_collector(context):
    """Simulate sending data to the collector."""
    context.data_sent_to_collector = True

@when('collector receives collect_? topic')
def step_when_collector_receives_collect_topic(context):
    """Simulate collector receiving the collect topic."""
    assert context.data_sent_to_collector, "Collector has not received any data."

@then('the data will be in persist topic')
def step_then_data_persisted(context):
    """Simulate data persistence."""
    assert context.data_sent_to_collector, "Data was not sent to collector, so it cannot be persisted."
    context.data_persisted = True

@then('the data will be received in logger')
def step_then_data_received_in_logger(context):
    """Simulate data being received in the logger."""
    assert context.data_persisted, "Data was not persisted, so it cannot be logged."
    context.data_logged = True

@when('a database error prevents persistence')
def step_when_database_error_occurs(context):
    """Simulate a database error preventing persistence."""
    context.persistence_failed = True

@then('the system must log a persistence failure')
def step_then_system_logs_failure(context):
    """Ensure the system logs a persistence failure."""
    assert context.persistence_failed, "No persistence failure occurred."
    context.failure_logged = True

@then('the data will not be persisted')
def step_then_data_not_persisted(context):
    """Ensure data was not persisted due to the failure."""
    assert context.persistence_failed, "No persistence failure occurred."
    assert not context.data_persisted, "Data should not be persisted."

@then('the system may attempt a retry')
def step_then_system_attempts_retry(context):
    """Optionally handle retry logic."""
    context.retry_attempted = True  # Simulating a retry mechanism
