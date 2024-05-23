from machine import Pin, SoftI2C
import time
from struct import unpack, unpack_from

SCD41_I2C_ADDRESS = 0x62
BMP180_I2C_ADDRESS = 0x77

i2c = SoftI2C(scl=Pin(33, pull=Pin.PULL_UP), sda=Pin(32, pull=Pin.PULL_UP))
print([hex(i) for i in i2c.scan()])

i2c.writeto(SCD41_I2C_ADDRESS, bytearray([0x00]))
print(f"SCD41 sensor found at {SCD41_I2C_ADDRESS:#x}")

def main():
    scd41_stop_periodic_measurement()
    time.sleep(1)
    scd41_start_periodic_measurement()
    print("SCD41: initialization finished")

    while True:
        for i in range(15, 0, -1):
            print(i)
            time.sleep(1)
        if scd41_get_data_ready_status():
            raw_measurement = scd41_read_measurement()
            if scd41_is_data_crc_correct(raw_measurement):
                co2 = (raw_measurement[0] << 8) | raw_measurement[1]

                raw_temperature = (raw_measurement[3] << 8) | raw_measurement[4]
                temperature = round(-45 + 175 * (raw_temperature / (2 ** 16 - 1)), 1)

                raw_humidity = (raw_measurement[6] << 8) | raw_measurement[7]
                humidity = round(100 * (raw_humidity / (2 ** 16 - 1)), 1)

                print(f"SCD41: CO2: {co2} ppm, Humidity: {humidity} %, Temperature: {temperature} °C")
        else:
            print("SCD41: no new data available")

        raw_temperature = bmp180_read_temperature(i2c)
        raw_pressure = bmp180_read_pressure(i2c)
        compute(coef, raw_temperature, raw_pressure)

def scd41_start_periodic_measurement():
    write_buffer = bytearray([0x21, 0xb1])
    i2c.writeto(SCD41_I2C_ADDRESS, write_buffer)

def scd41_stop_periodic_measurement():
    write_buffer = bytearray([0x3f, 0x86])
    i2c.writeto(SCD41_I2C_ADDRESS, write_buffer)

def scd41_get_data_ready_status():
    write_buffer = bytearray([0xe4, 0xb8])
    read_buffer = bytearray(3)
    i2c.writeto(SCD41_I2C_ADDRESS, write_buffer)
    time.sleep(0.001)
    i2c.readfrom_into(SCD41_I2C_ADDRESS, read_buffer, len(read_buffer))
    return (unpack(">H", read_buffer)[0] & 0x07ff)

def scd41_read_measurement():
    write_buffer = bytearray([0xec, 0x05])
    read_buffer = bytearray(9)
    i2c.writeto(SCD41_I2C_ADDRESS, write_buffer)
    time.sleep(0.001)
    i2c.readfrom_into(SCD41_I2C_ADDRESS, read_buffer, len(read_buffer))
    return read_buffer

def sensirion_common_generate_crc(buffer):
    crc = 0xff
    for byte in buffer:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = crc << 1
    return crc & 0xff

def scd41_is_data_crc_correct(buffer):
    crc_buffer = bytearray(2)
    for i in range(0, len(buffer), 3):
        crc_buffer[0] = buffer[i]
        crc_buffer[1] = buffer[i + 1]
        if sensirion_common_generate_crc(crc_buffer) != buffer[i + 2]:
            raise RuntimeError("CRC check failed while reading data")
    return True

def bmp180_read_chip_id(i2c):
    bmp180_reg_chip_id = b'\xd0'

    i2c.writeto(BMP180_I2C_ADDRESS, bmp180_reg_chip_id, False)
    chip_id = i2c.readfrom(BMP180_I2C_ADDRESS, 1)

    if chip_id[0] == 0x55:
        print("chip id is 0x55, BMP180 detected")
    else:
        print(f"chip id is: {hex(chip_id[0])}, NOT BMP180!")

def bmp180_read_coefficients(i2c)-> Bytes:
    bmp180_coef_reg_base = b'\xaa'
    bmp180_coef_size = 22

    i2c.writeto(BMP180_I2C_ADDRESS, bmp180_coef_reg_base, False)
    coefs = i2c.readfrom(BMP180_I2C_ADDRESS, bmp180_coef_size)

    return coefs

def bmp180_perform_measurement(i2c, command: Bytes, ms: int) -> Bytes :
    bmp180_reg_out_msb = b'\xf6'

    i2c.writeto(BMP180_I2C_ADDRESS, command, True)
    time.sleep_ms(ms)

    i2c.writeto(BMP180_I2C_ADDRESS, bmp180_reg_out_msb, False)
    out = i2c.readfrom(BMP180_I2C_ADDRESS, 3)

    return out

def bmp180_read_temperature(i2c)->int:
    bmp180_cmd_meas_temp = b'\xf4\x2e'

    return bmp180_perform_measurement(i2c, bmp180_cmd_meas_temp, 5)

def bmp180_read_pressure(i2c)->int:
    bmp180_cmd_meas_temp = b'\xf4\xf4'

    return bmp180_perform_measurement(i2c, bmp180_cmd_meas_temp, 26)

def compute(coef, raw_temp, raw_press):
    #this is horrible, but it is what the spec sheet says you should do
    #first, let's parse our coefficients

    #int.from_bytes exists, but more limited to struct
    #UT = int.from_bytes(raw_temp, 'big', True)
    UT = unpack_from(">h", raw_temp)[0]
    #Q what do we do with xlsb?
    #UP = unpack_from(">h", raw_press)[0]
    #UP is.. special, time to shift things around
    oss = 3
    UP = raw_press[0] << 16 | raw_press[1] << 8 | raw_press[2]
    UP = UP >> (8 - oss)

    AC1 = unpack_from(">h", coef)[0]
    AC2 = unpack_from(">h", coef, 2)[0]
    AC3 = unpack_from(">h", coef, 4)[0]
    AC4 = unpack_from(">H", coef, 6)[0]
    AC5 = unpack_from(">H", coef, 8)[0]
    AC6 = unpack_from(">H", coef, 10)[0]
    B1 = unpack_from(">h", coef, 12)[0]
    B2 = unpack_from(">h", coef, 14)[0]
    MB = unpack_from(">h", coef, 16)[0]
    MC = unpack_from(">h", coef, 18)[0]
    MD = unpack_from(">h", coef, 20)[0]

    #compute temperature
    X1 = (UT - AC6) * AC5 // 0x8000
    X2 = MC * 0x0800 // (X1 + MD)
    B5 = X1 + X2
    T = (B5 + 8) // 0x0010

    #compute pressure
    B6 = B5 - 4000
    X1 = (B2 * (B6 * B6 // (1 << 12))) // (1 < 11)
    X2 = AC2 * B6 // (1 << 11)
    X3 = X1 + X2
    B3 = (((AC1 * 4 + X3) << oss) + 2) // 4
    X1 = AC3 * B6 // (1 << 13)
    X2 = (B1 * (B6 * B6 // (1 << 12))) // (1 << 16)
    X3 = ((X1 + X2) + 2) // 4

    #unsigned longs here, check later
    B4 = AC4 * (X3 + 32768) // (1 << 15)
    B7 = (UP - B3) * (50000 >> 3)
    if B7 < 0x80000000 :
        p = (B7 * 2) // B4
    else:
        p = (B7 // B4) * 2
    X1 = (p // 256) * (p // 256)
    X1 = (X1 * 3038) // ( 1 << 16)
    X2 = (-7357 * p) // (1 << 16)
    p = p + (X1 + X2 + 3791) // 16

    print(f"BMP180: Temperature: {T / 10} °C, Pressure: {p / 100} hPa")

bmp180_read_chip_id(i2c)
coef = bmp180_read_coefficients(i2c)

main()
