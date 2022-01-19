# Terpulence II Flight Computer - Ground Station
# Author: Garrett Alessandrini

import time
import datetime
import board
import busio
import digitalio
import adafruit_rfm9x

# Settings we can mess with - edit these
dev_output = True
delay_between_data = 0.05

# Setting up sensors - do not edit
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi,
                             digitalio.DigitalInOut(board.CE1),
                             digitalio.DigitalInOut(board.D25),
                             433.0,
                             baudrate=1000000)  # last parameter slows down speed for breadboards


# Variables for the sensors - do not edit
startup_timestamp = time.time()
startup_altitude = -1
rfm9x.tx_power = 23
rfm9x.node = 2
rfm9x.destination = 1
rfm9x.enable_crc = True


# Reads data from radio, parses it, and stores it
def read_data():
    # receive data from air transmitter
    packet = rfm9x.receive(timeout=5.0)

    if packet is not None:
        # Parse Data
        packet_string = packet.decode("UTF-8")
        packet_obj = packet_string.split(';')

        temperature_c = float(packet_obj[0])
        temperature_f = float(packet_obj[1])
        relative_humidity = float(packet_obj[2])
        pressure = float(packet_obj[3])
        altitude_m = float(packet_obj[4])
        altitude_f = float(packet_obj[5])
        acceleration = float(packet_obj[6])
        linear_acceleration = float(packet_obj[7])
        gravity = float(packet_obj[8])
        angular_velocity = float(packet_obj[9])
        orientation_e = float(packet_obj[10])
        orientation_q = float(packet_obj[11])
        magnetic_field_strength = float(packet_obj[12])
        latitude = float(packet_obj[13])
        longitude = float(packet_obj[14])
        gps_altitude = float(packet_obj[15])
        gps_speed = float(packet_obj[16])

        # Dev Mode: Print data to console
        if dev_output:
            # SHTC3
            print("SHTC3:")
            print("Temperature (C): %0.1f C" % temperature_c)
            print("Temperature (F): %0.1f F" % temperature_f)
            print("Humidity: %0.1f %%" % relative_humidity)
            print("BMP388:")
            print("Pressure: %6.1f" % pressure)
            print("Altitude (meters): %5.2f meters" % altitude_m)
            print("Altitude (feet): %5.2f feet" % altitude_f)
            print("BNO055:")
            print("Accelerometer: {} m/s^2".format(acceleration))
            print("Linear Acceleration: {} m/s^2".format(linear_acceleration))
            print("Gravity: {} m/s^2".format(gravity))
            print("Angular Velocity: {} rad/sec".format(angular_velocity))
            print("Euler Orientation: {}".format(orientation_e))
            print("Quaternion Orientation: {}".format(orientation_q))
            print("Magnetic Field Strength: {} microteslas".format(magnetic_field_strength))
            print("GPS:")
            print("Latitude: {0:.6f} degrees".format(latitude))
            print("Longitude: {0:.6f} degrees".format(longitude))
            print("Altitude: {} meters".format(gps_altitude))
            print("Speed: {} knots".format(gps_speed))
            print("")

        # Write data to disk
        log = open("log_" + str(startup_timestamp) + ".txt", "a")

        # Start header w/ timestamp
        log.write("START_READING " + datetime.datetime.now().strftime('%H:%M:%S:%f'))

        # SHTC3
        log.write("%0.1f" % temperature_c)
        log.write("Temperature (F): %0.1f" % temperature_f)
        log.write("%0.1f" % relative_humidity)
        log.write("%6.1f" % pressure)
        log.write("%5.2f" % altitude_m)
        log.write("%5.2f" % altitude_f)
        log.write(str(acceleration))
        log.write(str(linear_acceleration))
        log.write(str(gravity))
        log.write(str(angular_velocity))
        log.write(str(orientation_e))
        log.write(str(orientation_q))
        log.write(str(magnetic_field_strength))
        log.write("{0:.6f}".format(latitude))
        log.write("{0:.6f}".format(longitude))
        log.write(str(gps_altitude))
        log.write(str(gps_speed))

        # End header
        log.write("END_READING")

        log.close()


while True:
    read_data()
    time.sleep(delay_between_data)
