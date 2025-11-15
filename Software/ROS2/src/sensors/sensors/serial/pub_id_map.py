import json
from rosidl_runtime_py.utilities import get_message

def read_id_map_file(node, file_path: str):
    # read the id map file
    json_data = '[]'
    with open(file_path, 'r') as f:
        json_data = f.read()
    # parse the json data
    ids = json.loads(json_data)
    for i in ids:
        pub_id = i['id']
        ros_type = i['message_type']
        topic = i['topic']
        is_ros_pub = i['ros_pub_or_sub'] == 'pub'
        arduino_folder = i['arduino_folder']
        is_nano = i['is_nano']
        ros_msg_type = get_message(ros_type)
        node.arduino_folders[pub_id] = arduino_folder
        node.is_board_nano[pub_id] = is_nano
        node.is_board_old_nano[pub_id] = i.get('old_nano', False)
        if is_ros_pub:
            node.add_ros_pub(pub_id, ros_msg_type, topic)
        else:
            node.add_ros_sub(pub_id, ros_msg_type, topic)