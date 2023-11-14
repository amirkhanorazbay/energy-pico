# CRC-16 calculation function (little-endian byte order)
def crc16(data):
    crc = 0xFFFF
    for i in data:
        crc ^= i
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

# Function to check CRC of the response
def check_crc(data):
    received_crc = int.from_bytes(data[-2:], 'little')  # Extract received CRC from the response
    calculated_crc = int.from_bytes(crc16(data[:-2]), 'little')  # Calculate CRC of the response data
    return received_crc == calculated_crc
