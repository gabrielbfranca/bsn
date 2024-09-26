from behave import given, when, then
from utils.parsers import get_rosnode_info
import subprocess

def isNodeCommunicating(node_info, target, topics, bound):
    return all(
        any(
            conn['topic'] == topic and conn['direction'].startswith(bound)
            for conn in node_info['connections'] if conn['to'] == target
        )
        for topic in topics
    )

@given('the {node_name} node is online')
def step_given_node_is_online(context, node_name):
    result = subprocess.run(['rosnode', 'info', node_name], capture_output=True)
    context.node_info = get_rosnode_info(result)
    context.inbound_results = []
    context.outbound_results = []

@when('I check if topics {inbound_topic} are inbound and {outbound_topic} are outbound to {target}')
def step_when_check_topics(context, inbound_topic, outbound_topic, target):
    node_info = context.node_info
    outbound_topics = outbound_topic.split(',')
    inbound_topics = inbound_topic.split(',')

    inbound_verified = isNodeCommunicating(node_info, target, inbound_topics, 'inbound')
    outbound_verified = isNodeCommunicating(node_info, target, outbound_topics, 'outbound')
    
    context.inbound_results.append(inbound_verified)
    context.outbound_results.append(outbound_verified)

    # Assert right after checking
    assert inbound_verified, f"Topics {inbound_topic} are not properly inbound from {target} to {node_info['node_name']}"
    assert outbound_verified, f"Topics {outbound_topic} are not properly outbound to {target} from {node_info['node_name']}"

@when('I check if topics {topic_list} are inbound from {target}')
def step_when_check_inbound_topics(context, topic_list, target):
    try:
        node_info = context.node_info
        topics = topic_list.split(',')

        inbound_verified = isNodeCommunicating(node_info, target, topics, 'inbound')
        context.inbound_results.append(inbound_verified)

        # Assert right after checking
        assert inbound_verified, f"Topics {topic_list} are not properly inbound from {target} to {node_info['node_name']}"
    
    except (KeyError, TypeError, Exception) as e:
        context.inbound_results.append(False)
        print(f"An error occurred while verifying inbound topics: {e}")
        assert False, f"Error occurred while checking inbound topics: {e}"

@when('I check if topics {topic_list} are outbound to {target}')
def step_when_check_outbound_topics(context, topic_list, target):
    try:
        node_info = context.node_info
        topics = topic_list.split(',')

        outbound_verified = isNodeCommunicating(node_info, target, topics, 'outbound')
        context.outbound_results.append(outbound_verified)

        # Assert right after checking
        assert outbound_verified, f"Topics {topic_list} are not properly outbound to {target} from {node_info['node_name']}"
    
    except (KeyError, TypeError, Exception) as e:
        context.outbound_results.append(False)
        print(f"An error occurred while verifying outbound topics: {e}")
        assert False, f"Error occurred while checking outbound topics: {e}"

@then('{node_name} node is connected appropriately')
def step_then_node_is_connected(context, node_name):
    # Final assertion that ensures all verifications are successful
    assert all(context.inbound_results), f"{node_name} is not receiving all expected inbound topics"
    assert all(context.outbound_results), f"{node_name} is not publishing all expected outbound topics"
