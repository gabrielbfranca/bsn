from behave import given, when, then
import subprocess
import json

# This function is assumed to parse the output and return the correct dictionary structure.
from utils import get_rosnode_info

def get_node_data(node_name):
    result = subprocess.run(['rosnode', 'info', node_name], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return get_rosnode_info(result)

@given('the {node_name} node is online')
def step_node_is_online(context, node_name):
    # Check if the node is online by trying to get its info
    context.node_info = get_node_data(node_name)
    assert context.node_info, f"Node {node_name} is not online"

@when('I check if topics {topic1} and {topic2} are inbound and outbound to {target_node}')
def step_check_topics(context, topic1, topic2, target_node):
    #target_node_data = get_node_data(f"/{target_node}")
    
    # Check connections in both directions
    context.topic1_found = any(conn['topic'] == topic1 for conn in context.node_info['connections'])
    context.topic2_found = any(conn['topic'] == topic2 for conn in context.node_info['connections'])

@when('I check if topics {topic} are inbound from {target_node}')
def step_check_inbound_topics(context, topic, target_node):
    #target_node_data = get_node_data(f"/{target_node}")
    context.inbound_topic_found = any(conn['topic'] == topic for conn in target_node_data['connections'])

@when('I check if topics {topic} are outbound to {target_node}')
def step_check_outbound_topics(context, topic, target_node):
    target_node_data = get_node_data(f"/{target_node}")
    context.outbound_topic_found = any(pub['topic'] == topic for pub in target_node_data['publications'])

@then('{node_name} connections should have the respective topics')
def step_check_connections(context, node_name):
    assert context.topic1_found, f"{node_name} does not have the topic {context.topic1} connected"
    assert context.topic2_found, f"{node_name} does not have the topic {context.topic2} connected"

@then('{node_name} node is connected appropriately')
def step_check_logger_connections(context, node_name):
    assert context.inbound_topic_found, f"{node_name} does not have the expected inbound topic"
    assert context.outbound_topic_found, f"{node_name} does not have the expected outbound topic"
