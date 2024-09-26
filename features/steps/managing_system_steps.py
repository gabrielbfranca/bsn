from behave import given, when, then
from utils.parsers import get_rosnode_info
import subprocess

# Assuming get_rosnode_info is a function that executes rosnode info for a given node and returns the parsed data
@given('the {node_name} node is online')
def step_given_node_is_online(context, node_name):
    # Call the function that checks the node status
    result = subprocess.run(['rosnode', 'info', node_name], capture_output=True)
    context.node_info = get_rosnode_info(result)

@when('I check if topics {inbound_topic} are inbound and {outbound_topic} are outbound to {target}')
def step_when_check_topics(context, inbound_topic, outbound_topic, target):
    node_info = context.node_info
    outbound_topics = outbound_topic.split(',')
    inbound_topics = inbound_topic.split(',')
    context.inbound_verified = any(
        any(
            conn['topic'] == topic and 
            conn['direction'].startswith('inbound') and
            conn['from'] == target  # Ensure the connection is inbound from the correct source
            for conn in node_info['connections']
        )
        for topic in inbound_topics
    )
    context.outbound_verified = any(
        any(
            conn['topic'] == topic and 
            conn['direction'].startswith('outbound') and
            conn['to'] == target  # Ensure the connection is outbound to the correct target
            for conn in node_info['connections']
        )
        for topic in outbound_topics
    )

@when('I check if topics {topic_list} are inbound from {target}')
def step_when_check_inbound_topics(context, topic_list, target):
    try:
        node_info = context.node_info
        topics = topic_list.split(',')

        context.inbound_verified = all(
        any(
            conn['topic'] == topic and 
            conn['direction'].startswith('outbound') and
            conn['from'] == target  
            for conn in node_info['connections']
        )
        for topic in topics
        )
    except KeyError as e:
        # Handle missing dictionary keys like 'topic', 'direction', or 'from'
        context.inbound_verified = False
        print(f"KeyError: {e}. The connection data might be missing necessary fields.")

    except TypeError as e:
        # Handle issues like trying to iterate over a NoneType or malformed data
        context.inbound_verified = False
        print(f"TypeError: {e}. Check the structure of node_info or topics.")

    except Exception as e:
        # Catch any other unexpected exceptions
        context.inbound_verified = False
        print(f"An error occurred while verifying inbound topics: {e}")

@when('I check if topics {topic_list} are outbound to {target}')
def step_when_check_outbound_topics(context, topic_list, target):
    try:
        node_info = context.node_info
        topics = topic_list.split(',')

        context.outbound_verified = all(
            any(
            conn['topic'] == topic and 
            conn['direction'].startswith('inbound') and
            conn['to'] == target  # Ensure the connection is inbound from the correct source
            for conn in node_info['connections']
            )
            for topic in topics
        )
    except KeyError as e:
        # Handle missing dictionary keys like 'topic', 'direction', or 'from'
        context.outbound_verified = False
        print(f"KeyError: {e}. The connection data might be missing necessary fields.")

    except TypeError as e:
        # Handle issues like trying to iterate over a NoneType or malformed data
        context.outbound_verified = False
        print(f"TypeError: {e}. Check the structure of node_info or topics.")

    except Exception as e:
        # Catch any other unexpected exceptions
        context.outbound_verified = False
        print(f"An error occurred while verifying inbound topics: {e}")

@then('{node_name} node is connected appropriately')
def step_then_node_is_connected(context, node_name):
    assert context.inbound_verified, f"{node_name} is not receiving the expected inbound topics"
    assert context.outbound_verified, f"{node_name} is not publishing the expected outbound topics"
