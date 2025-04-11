import struct
import serial

def pack_package()
def pack_as_hex(data):
    if (type(data) != int):
        exit
    hex_data = struct.pack("!h", data)

    return(hex_data)

def unpack_from_hex(data):
    unpacked_data = struct.unpack("!h", data)

    return(unpacked_data)

print(pack_as_hex(200))
print(pack_as_hex(90))

print(unpack_from_hex(pack_as_hex(90)))



