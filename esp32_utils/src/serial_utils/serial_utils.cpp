#include "serial_utils.h"

void setupSerial() {
    /*
      Sets up the serial port with the specified baudrate.
    */

    Serial.begin(115200);
}

int calculateChecksum(String message) {
    /*
      Calculates the checksum of the given message by summing the ASCII values of the characters.
    
      Arguments:
        message (str): The message for which to calculate the checksum.
    
      Returns:
        int: The calculated checksum (module 256).
    */

    int checksum = 0;

    // Iterates over each character in the message and sums their ASCII values
    for (int i = 0; i < message.length(); i++) {
        checksum += (int)message[i];
    }
    return checksum % 256;
}

String prepareSerialMessage(String message) {
    /*
      Prepares a message to send over the serial port by adding the checksum.
    
      Args:
          message (String): The message to be sent, without checksum. 
                            Should be in the format "type,value".
    
      Returns:
          String: Complete message with checksum formatted as "type,value,checksum\n".
    */

    // Remove leading and trailing whitespace from the message
    message.trim();

    // Calculate the checksum
    int checksum = calculateChecksum(message);

    // Return the complete message with checksum
    return (message + "," + String(checksum));
}

String readMessage() {
    /*
      Performs the entire process of reading a message from the ESP32.
    */

    // If there is any data reads the incoming message until a finds character ('\n')
    if (Serial.available()) {
        String received = Serial.readStringUntil('\n');
        return received;
    }
    return "";
}

void sendMessage(String message) {
    /*
      Performs the entire process of sending a message to the ESP32.
    
      Args:
        message (string): Message to be sent in the format "type,value".
    */  

    String serial_message = prepareSerialMessage(message);
    //Serial.write(serial_message.c_str(), serial_message.length());

    Serial.println(serial_message);
    fflush(stdout);
    //delay(1000);
}

bool verifyChecksum(String message) {
    /*
      Verifies if a received message has a correct checksum.
    
      Args:
        message (str): The received message, formatted as "Type,Value,Checksum".
    
      Returns:
        bool: True if the checksum is correct, false otherwise.
    */

    // Finds the position of the last comma, which separates the message from the checksum
    int last_comma = message.lastIndexOf(',');

    // If there is no comma, the message format is incorrect, return false
    if (last_comma == -1) {
        return false;
    }

    // Extract the message and the checksum content
    String msg_content = message.substring(0, last_comma);
    String checksum_str = message.substring(last_comma + 1);

    // Calculates the checksum of the message content
    int received_checksum = checksum_str.toInt();
    int calculated_checksum = calculateChecksum(msg_content);

    return received_checksum == calculated_checksum;
}
