import serial

port = serial.Serial("/dev/ttyUSB0",
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=10)

while True:
    command = raw_input('Enter your command, or press q to quit: ')
    if command == 'q':
        break
        port.close()
        exit()
    else:
        port.write(str.encode(command + '\r'))

port.close()
