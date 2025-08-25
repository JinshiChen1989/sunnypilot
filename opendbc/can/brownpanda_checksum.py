"""
BrownPanda checksum implementation for DBC parsing
Following BYD pattern for hardware-level safety enforcement
"""

def brownpanda_checksum(address, data):
    """
    Calculate brownpanda checksum following BYD pattern
    """
    byte_key = 0xAF
    
    sum_first = 0   # Sum for upper 4-bit nibbles
    sum_second = 0  # Sum for lower 4-bit nibbles
    
    # skip checksum at the last byte
    for byte in data[:-1]:
        sum_first += byte >> 4    # Extract upper nibble
        sum_second += byte & 0xF  # Extract lower nibble
    
    remainder = (sum_second >> 4) & 0xFF
    sum_first += (byte_key & 0xF)    # Low nibble of byte_key
    sum_second += (byte_key >> 4)    # High nibble of byte_key
    
    # Inline inverse computation for each sum: inv = (-sum + 0x9) & 0xF
    inv_first = (-sum_first + 0x9) & 0xF
    inv_second = (-sum_second + 0x9) & 0xF
    
    checksum = (((inv_first + (5 - remainder)) << 4) + inv_second) & 0xFF
    return checksum


# Messages that require checksum validation
BROWNPANDA_CHECKSUM_ADDRS = {
    0x60: 6,  # pandaMessage
    0x61: 8,  # userCommand
    0x62: 8,  # driverState
    0x63: 8,  # carState
    0x64: 8,  # carState2
    0x65: 8,  # wheelSensor
    0x66: 6,  # adasState
    0x67: 8,  # chassisState
    0x68: 8,  # powertrainState
    0x69: 8,  # imuSensor
    0x6A: 8,  # longCommand
    0x6B: 8,  # longCommand2
    0x6C: 8,  # latCommand
    0x6D: 8,  # latCommand2
    0x6E: 8,  # dispCommand
    0x6F: 8,  # visionInfo
    0x70: 6,  # modelIdAtto3
    0x71: 4,  # modelIdDolphin
    0x72: 8,  # modelIdDeepal
}