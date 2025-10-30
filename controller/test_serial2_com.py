#This is a test .py file to check if serial2 works or not
#Using this to check if we can connect Neo-6m GPS device with nvidia

'''
🔌 Serial Communication Test Script for Jetson AGX Xavier - UART
🧠 What This Script Does (Python Side)
        This Python script establishes UART-based serial communication using the pyserial library on the Jetson AGX Xavier via /dev/ttyTHS0. Here's a breakdown:

        Opens a serial port (/dev/ttyTHS0) at a baud rate of 9600.

        Continuously:

                Prompts the user for an integer input (0-255).

                Converts the integer to a byte and sends it over UART.

                Reads any response coming back on the same serial line and prints it to the terminal.

🛠️ Hardware Setup (Jetson AGX Xavier UART)
        To ensure proper UART communication on the Jetson AGX Xavier:
        to check what you sent and what is recieve:

        UART_TX (Transmit) Pin 8  (Connected directly to ) → UART_RX (Receive) Pin 10

'''

import serial

def main(args=None):
    try:
        ser = serial.Serial('/dev/ttyTHS0', 9600, timeout=1)
        print("Opened successfully")
    except Exception as e:
        print(f"Failed: {e}")

    while(1):
        value = int(input("Enter your value: "))
        print(f"value: {value}")
        packet = bytes([value])
        ser.write(packet)
        data = ser.readline()
        print(f"Received: {data}")

if __name__ == '__main__':
    main()