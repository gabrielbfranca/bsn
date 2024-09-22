import re

def get_rostopic_sensor_data(result):
    if result.returncode != 0:
        raise Exception(f"Error getting topic data: {result.stderr.decode('utf-8')}")
    
    # Decode the output from bytes to string
    output = result.stdout.decode('utf-8')
    
    # Parse the output using regex
    data = {}

    # Match key-value pairs
    header_seq = re.search(r'seq: (\d+)', output)
    header_stamp_secs = re.search(r'secs: (\d+)', output)
    header_stamp_nsecs = re.search(r'nsecs: (\d+)', output)
    data_type = re.search(r'type: "(.*?)"', output)
    data_value = re.search(r'data: ([\d\.]+)', output)
    risk_value = re.search(r'risk: ([\d\.]+)', output)
    batt_value = re.search(r'batt: ([\d\.]+)', output)

    # Fill the parsed data dictionary
    if header_seq:
        data['seq'] = int(header_seq.group(1))
    if header_stamp_secs:
        data['stamp_secs'] = int(header_stamp_secs.group(1))
    if header_stamp_nsecs:
        data['stamp_nsecs'] = int(header_stamp_nsecs.group(1))
    if data_type:
        data['type'] = data_type.group(1)
    if data_value:
        data['data'] = float(data_value.group(1))
    if risk_value:
        data['risk'] = float(risk_value.group(1))
    if batt_value:
        data['batt'] = float(batt_value.group(1))
    
    return data

def get_rosnode_info(result):
    if result.returncode != 0:
        raise Exception(f"Error getting node info: {result.stderr.decode('utf-8')}")
    
    # Decode the output from bytes to string
    output = result.stdout.decode('utf-8')
    
    # Initialize dictionaries for storing parsed data
    node_info = {
        "publications": [],
        "subscriptions": [],
        "services": [],
        "connections": []
    }

    # Parse publications
    publications_section = re.search(r'Publications:\s+((?:\*.*\n)+)', output)
    if publications_section:
        publications = re.findall(r'\*\s([^\[]+)\s\[(.*?)\]', publications_section.group(1))
        node_info["publications"] = [{"topic": pub[0].strip(), "type": pub[1].strip()} for pub in publications]

    # Parse subscriptions
    subscriptions_section = re.search(r'Subscriptions:\s+((?:\*.*\n)+)', output)
    if subscriptions_section:
        subscriptions = re.findall(r'\*\s([^\[]+)\s\[(.*?)\]', subscriptions_section.group(1))
        node_info["subscriptions"] = [{"topic": sub[0].strip(), "type": sub[1].strip()} for sub in subscriptions]

    # Parse services
    services_section = re.search(r'Services:\s+((?:\*.*\n)+)', output)
    if services_section:
        services = re.findall(r'\*\s([^\s]+)', services_section.group(1))
        node_info["services"] = services

    # Parse connections
    connections_section = re.search(r'Connections:\s+((?:\*.*\n)+)', output)
    if connections_section:
        connections = re.findall(r'\*\s+topic:\s+([^\n]+)\n\s+\*\s+to:\s+([^\n]+)\n\s+\*\s+direction:\s+([^\n]+)', connections_section.group(1))
        node_info["connections"] = [{"topic": conn[0].strip(), "to": conn[1].strip(), "direction": conn[2].strip()} for conn in connections]

    return node_info
