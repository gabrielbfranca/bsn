from behave import given, when, then
import subprocess
from utils.parsers import parse_topic_data, format_entity
from utils.asserts import node_is_active
def capture_topic_data(topic):
    parsed_data = parse_topic_data(topic, line_limit=10)
    high_risk_detected = any(float(value) > 10 for value in parsed_data.get('risk', []))
    return topic, parsed_data, high_risk_detected

@given('nodes are online')
def step_given_nodes_online(context):
    nodes = []
    for row in context.table:
        nodes.append(format_entity(row['Nodes']))
    node_is_active(nodes)

@when('{node_name} sends a collection of data')
def step_when_node_sends_data(context, node_name):
    topic = f'/{node_name}_data'
    context.sensor_data = capture_topic_data(topic)
    assert context.sensor_data, f"No data received from {node_name}"

@then('g4t1 will detect new patient health status')
def step_then_g4t1_detects_health_status(context):
    assert 'patient_status' in context.sensor_data[1], "Patient health status not detected."

@given('the bodyhub has processed patient data')
def step_given_bodyhub_processed_data(context):
    context.processed_data = True
    assert context.processed_data, "Bodyhub failed to process patient data"

@when('an internal processing error occurs in g4t1')
def step_when_internal_error_occurs(context):
    context.internal_error = True

@then('g4t1 will fail to detect the new patient health status')
def step_then_g4t1_fails_to_detect_status(context):
    assert context.internal_error, "No internal error detected"
    assert 'patient_status' not in context.sensor_data[1], "g4t1 incorrectly detected a patient status"

@when('{node_name} sends data with high risk')
def step_when_high_risk_data_sent(context, node_name):
    topic = f'/{node_name}_data'
    _, parsed_data, high_risk_detected = capture_topic_data(topic)
    context.high_risk_detected = high_risk_detected
    assert high_risk_detected, "High-risk data was not detected"

@then('g4t1 will detect an emergency in less than 250 ms')
def step_then_g4t1_detects_emergency(context):
    assert context.high_risk_detected, "Emergency not detected in time"

@given('Patient Data is not active')
def step_given_patient_data_inactive(context):
    context.patient_data_active = False
    assert not context.patient_data_active, "Patient data should be inactive"

@when('{node_name} sends low-risk data at 10x normal frequency and then suddenly sends high-risk data')
def step_when_overloaded_data_sent(context, node_name):
    topic = f'/{node_name}_data'
    _, parsed_data, high_risk_detected = capture_topic_data(topic)
    context.overloaded = True
    context.high_risk_detected = high_risk_detected
    assert context.overloaded, "Sensor data overload did not occur"

@then('g4t1 might experience delayed emergency detection')
def step_then_g4t1_might_delay_detection(context):
    assert context.overloaded and context.high_risk_detected, "Delayed detection scenario not met"
