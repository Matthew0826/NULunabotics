from rclpy.serialization import serialize_message, deserialize_message

# according to Fast-CDR, the first 2 bytes of the serialized message are 0x0001 if the message is little-endian, and 0x0000 if the message is big-endian
LITTLE_ENDIAN = b'\x00\x01'
BIG_ENDIAN = b'\x00\x00'


def deserialize(message: bytes, ros_type: any):
    # add 2 bytes to the beginning of the message to represent little-endianess
    # and 2 more 0 bytes to fill the 4 byte header of a CDR message
    message = LITTLE_ENDIAN + b'\x00\x00' + message
    # deserialize the message
    deserialized_msg = deserialize_message(message, ros_type)
    return deserialized_msg


def serialize(message: any):
    # serialize the message
    serialized_msg = serialize_message(message)
    # remove the first 4 bytes (CDR header, we don't need it)
    serialized_msg = serialized_msg[4:]
    return serialized_msg
