import struct

# Create an unsigned short integer value
value = 12345

# Pack the value into bytes using little-endian byte order
packed_data = struct.pack('<H', value)

# Unpack the bytes into the original value
unpacked_value = struct.unpack('<H', packed_data)[0]

print("Original Value:", value)
print("Packed Data:", packed_data)
print("Unpacked Value:", unpacked_value)
