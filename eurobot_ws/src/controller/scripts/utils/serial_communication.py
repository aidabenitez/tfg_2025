import serial, time

def open_serial_port(port, baudrate, timeout=0.5):
    """
    Opens the serial port with the specified parameters.
    
    Arguments:
        port (str): The name of the serial port (e.g. '/dev/ttyUSB0').
        baudrate (int): The transmission speed (bits per second).
        timeout (int): Wait time for read operations.

    Returns:
        serial.Serial: A configured serial object.
    """

    try:
        serial_port = serial.Serial(port, baudrate, timeout=timeout)
        serial_port.reset_input_buffer()
        serial_port.reset_output_buffer()

        print(f"Serial port {port} opened successfully.")
        return serial_port
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        return None

def prepare_serial_message(message):
    """
    Prepares a message to send over the serial port by adding the checksum.
    
    Args:
        message (str): The message to be sent, without checksum.
    
    Returns:
        str: Complete message with checksum.
    """

    message_stripped = message.strip()
    checksum = calculate_checksum(message_stripped)
    return f"{message_stripped},{checksum}\n"

def calculate_checksum(message):
    """
    Calculates the checksum of the given message by summing the ASCII values of the characters.
    
    Arguments:
        message (str): The message for which to calculate the checksum.
    
    Returns:
        int: The calculated checksum (module 256).
    """

    checksum = sum(ord(c) for c in message) % 256
    return checksum

def verify_serial_message(message):
    """
    Verifies if a received message has a correct checksum.
    
    Args:
        message (str): The received message, formatted as "Type,Value,Checksum".
    
    Returns:
        bool: True if the checksum is correct, false otherwise.
    """

    # Split the message into parts using the comma as a delimiter
    message_parts = message.split(',')
    
    # Validate if the message has minimum three parts
    if len(message_parts) < 3:
        print("Invalid message format, must have at least 3 parts separated by commas.")
        return False

    # Verify the checksum
    try:
        # Extract the message content by joining all parts except the checksum 
        msg_content = ','.join(message_parts[:-1])
        received_checksum = int(message_parts[-1])

        # Calculate the expected checksum
        calculated_checksum = calculate_checksum(msg_content)
        
        # Compare the received checksum with the calculated checksum and return the result
        return received_checksum == calculated_checksum
    
    except ValueError:
        print("Checksum is not a valid number.")
        return False

def read_serial_data(serial_port):
    """
    Reads data from the serial port if messages are available.

    Args:
        serial_port (serial.Serial): The open serial port object.

    Returns:
        str or None: The received message if one is available, otherwise None.
    """

    # If there is data read a line from the serial port, decode it from ASCII, and strip any extra whitespace
    if serial_port.in_waiting > 0:
        return serial_port.readline().decode('utf-8', errors='ignore').strip()
    
    # Otherwise return None
    return None



def read_message(serial_port):
    """
    Performs the entire process of reading a message from the ESP32.

    Args:
        serial_port (serial.Serial): The open serial port object.
    """

    try:
        # Read a message from the serial port
        message = read_serial_data(serial_port)

        # If a message is received
        if message:
            # print(f"Received message: {message}")
            # Verify the integrity of the message
            if verify_serial_message(message):
                # print("Correct checksum.")
                return message

    except KeyboardInterrupt:
        print("Read operation interrupted.")
    
    return None


def send_message(serial_port, message):
    """
    Performs the entire process of sending a message to the ESP32.
    
    Args:
        serial_port (serial.Serial): The open serial port object.
        message (string): Message to be sent in the format "type,value".
    """

    # Attempt to send message
    try:
        serial_message = prepare_serial_message(message)
        serial_port.write(serial_message.encode('ascii'))
        # print(f"Sended: {serial_message.strip()}")

    except Exception as e:
        print(f"Error sending message: {e}")