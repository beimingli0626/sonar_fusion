#!/usr/bin/env python3

import ctypes
import os
import sys
import time

UART_J7 = 9
UART_J10 = 7
UART_J11 = 12
UART_J12 = 5

BAUD_RATE = 9600
# BAUD_RATE = 57600

class SerialAPI():
    def __init__(self, bus, baud):
        print("SerialAPI: bus: {}, baud: {}".format(bus, baud))
        self.bus = bus
        self.baud = baud
        self.buf_size = 5

        # Wrapper. Connect with UART so lib. 
        self.voxl_io = ctypes.CDLL(os.path.abspath('/usr/lib/libvoxl_io.so'))
        self.voxl_io.voxl_uart_init.argtypes = (ctypes.c_int, ctypes.c_int)
        # self.voxl_io.voxl_uart_write.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)
        self.voxl_io.voxl_uart_read.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_uint8), ctypes.c_size_t)
        # self.voxl_io.voxl_uart_flush.argtype = (ctypes.c_int)
        # self.voxl_io.voxl_uart_drain.argtype = (ctypes.c_int)
        # self.voxl_io.voxl_uart_close.argtype = (ctypes.c_int)

        # Init UART Communication, 0 success, -1 fail
        ret = self.voxl_io.voxl_uart_init(bus, baud)
        if ret == 0: 
            print("UART Init Success!")
        else:
            print("UART Init Fail!")
            sys.exit()


    def read(self):
        """
        Read byte messages

        :return distrance: a list of distance in centimeters
        """
        read_buffer = ctypes.c_uint8 * self.buf_size    # c.ubyte_Array class type
        pt_read_buf = read_buffer() # c.ubyte_Array instance
        bytes_read = self.voxl_io.voxl_uart_read(self.bus, pt_read_buf, self.buf_size)

        if bytes_read == -1:
            print("Read Fail, bytes read: {}, buf_size: {}}".format(bytes_read, self.buf_size))
        else:
            print("Bytes read: {}".format(bytes_read))
            msgs = bytearray(pt_read_buf)
            distances = self.decode_msg(msgs, bytes_read)
            return distances


    def decode_msg(self, msgs, bytes_read):
        """
        Decode byte messages

        :param msgs: bytearray, might contain multiple msg
        :param bytes_read: number of bytes contained in bytearray
        :return distances: a list of distance in centimeters

        Note: for msg content, checkout https://www.maxbotix.com/documents/XL-MaxSonar-EZ_Datasheet.pdf
        """
        distances = []
        if bytes_read % 5 != 0:
            print("Number of bytes read should be multiple of 5!")
            return distances

        while len(distances) * 5 != bytes_read:
            idx = len(distances)
            msg = str(msgs[idx * 5 : (idx+1) * 5], 'utf-8')
            if msg[0] != 'R': # msg should start with ASCII R
                print("MSG doesn't start with R, error reading")
                break
            if msg[4] != '\r': # msg should end with ASCII carriage return
                print("MSG doesn't end with carriage return, error reading")
                break
            
            distances.append(int(msg[1:4]))

        return distances

        
if __name__ == "__main__":
    serial = SerialAPI(UART_J10, BAUD_RATE)
    while True:
        print(serial.read())   # Readings can occur up to every 100ms
        time.sleep(0.2)
