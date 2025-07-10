from serial_communication import open_serial_port, send_message, read_message


## ******************************
## FIRST DRIVER CLASS
## ******************************

class FirstDriver:

    def __init__(self, port = '/dev/ttyUSB0', baudrate = 115200):
        """
        Initializes all the attributes of the class.
        """

        self.port        = port
        self.baudrate    = baudrate
        self.serial_port = open_serial_port(self.port, self.baudrate)


    def send_motors_pow(self, left_motor_vel, right_motor_vel):
        """
        Gets each motor velocity values and send them to the Esp32.

        Args:
            left_motor_vel (double): Left motor velocity in m/s.
            right_motor_vel (double): Right motor velocity in m/s.
        """

        send_message(self.serial_port, f"ML,{(left_motor_vel)},MR,{(right_motor_vel)}")


    def read_motors_data(self):
        """
        Reads the message sended by Esp32 containing the encoders value and returns this data.

        Returns:
            tuple: Encoder left and encoder right distance in centimeters
        """

        encoder_left  = 0
        encoder_right = 0
        
        received_msg = read_message(self.serial_port)

        if received_msg != None:
            # Split the message into parts separated by commas and remove the last part (checksum)
            msg_parts = received_msg.split(",")[:-1]

            # Process command pairs
            for i_msg_parts in range(0, len(msg_parts), 2):
                msg_element = msg_parts[i_msg_parts]
                msg_value   = float(msg_parts[i_msg_parts + 1])

                if msg_element == "EL":
                    encoder_left = msg_value
                elif msg_element == "ER":
                    encoder_right = msg_value

        
        return encoder_left, encoder_right