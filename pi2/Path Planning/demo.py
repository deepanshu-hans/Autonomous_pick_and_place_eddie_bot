import serial

port = serial.Serial("/dev/ttyUSB0",
    baudrate = 115200,
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE,
    bytesize = serial.EIGHTBITS,
    timeout = 10)

forward = str.encode('GO 1C 1C\r')
reverse = str.encode('GO DF DF\r')
left = str.encode('GO DF 1C\r')
right = str.encode('GO 1C DF\r')
watch = str.encode('WATCH 0\r')
stop = str.encode('STOP 0\r')
reset = str.encode('RST\r')
verbon = str.encode('VERB 1\r')

try:
    port.write(watch)
    while True:
        command = raw_input('Enter command: ')
        if command == 'f':
            port.write(forward)
        elif command == 'b':
            port.write(reverse)
        elif command == 'l':
            port.write(left)
        elif command == 'r':
            port.write(right)
        elif command == 's':
            port.write(stop)
        else:
            print('Invalid Command!')

except KeyboardInterrupt:
    pass

port.close()
