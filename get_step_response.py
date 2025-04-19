import serial
import time

arduino = serial.Serial('/dev/esp', 115200)  # COM port của Arduino, chỉnh lại nếu cần
file = open("output.csv", "w")

start_time = time.time()
while True:

    if (time.time() - start_time) > 21.0:
        arduino.write(b"12\n")

    elif (time.time() - start_time) > 15.0:
        arduino.write(b"9\n")

    elif (time.time() - start_time) > 9.0:
        arduino.write(b"6\n")

    elif (time.time() - start_time) > 3.0:
        arduino.write(b"3\n")
    
    else:
        arduino.write(b"0\n")

    data = arduino.readline().decode('utf-8')
    print(data.strip())
    file.write(data)