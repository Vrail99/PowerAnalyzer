"""MIT License

Copyright (c) 2021 Sebastian Zisch

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import serial
import time
from serial.serialutil import SerialTimeoutException, SerialException
import serial.tools.list_ports
import tkinter.messagebox


class SerialBus:
    def __init__(self):
        self.sBus = serial.Serial()
        self.name = "None"
        self.linereader = None
        self.writeTimeout = 2  # Seconds
        self.readTimeout = 2  # Seconds
        self.currPort = None  # Current Port
        self.port_list = []  # Empty Port List

    def openPort(self, port: str) -> int:
        """Opens the port. Reopens the port, if it was already open.
        Performs a handshake with the teensy to check if the connection can be established.

        Keyword Arguments:

        port -- String of the port to connect to.

        Returns 1 if successful, or 0 if an error occurred."""
        try:
            if (self.deviceOpen()):  # If open: close port and reopen with timings
                self.sBus.close()
                time.sleep(0.5)  # Wait for 1s
            self.sBus = serial.Serial(port, timeout=self.readTimeout,
                                      write_timeout=self.writeTimeout, baudrate=115200)

            self.currPort = port
            self.linereader = ReadLine(self.sBus)
            # Check if the port is correct
            try:
                self.writeString('x')
                time.sleep(0.25)
                self.writeString('ci')
                answer = self.linereader.readline().decode(
                    'utf-8').rstrip()  # Remove any control-characters
                if (answer != "tconn"):
                    print("Error connecting")
                    self.sBus.close()
                    return 0
                else:
                    print("Connected")

            except SerialTimeoutException as e:
                tkinter.messagebox.showinfo("Info",
                                            "Timeout. Connection with Teensy-port '"+str(self.currPort) +
                                            "' not possible.\n Maybe a wrong port?")
                self.sBus.close()
                return 0

        except ValueError:
            print("Error, couldn't open Serial port. "
                  "Maybe already connected in another program?")
            return 0
        except SerialException as e:
            print("The Port does not exist", e)
            return 0

        return 1

    def closePort(self) -> None:
        """Closes the port, if possible"""
        try:
            self.writeString('cd')
            self.writeString('bf')
            self.sBus.close()
        except:
            print("Error! Port cannot be closed")

    def deviceOpen(self) -> bool:
        """Returns the state of the connection"""
        return self.sBus.is_open

    # Read-Functions
    def readIntValue(self, address: str = None) -> int:
        """Read a certain volatile memory location, determined by address-codes in \n
        volatile_cfg.txt

        Keyword Arguments:

        address -- The address to read a value from

        Returns the integer value from the teensy or None"""
        value = None
        if (self.deviceOpen() and address != None):
            self.writeString(str(address))
            try:
                value = int(self.readLine())
            except TypeError as e:
                print(e)

        return value

    def readEEPROMValue(self, address: str, mask: str = 0, pos: str = 0) -> int:
        """Reads a value from the EEPROM of the ACS71020

        Keyword Arguments:

        address -- the adress to be read from \n
        mask    -- the mask for the value. 0's for every bit (default: 0x0) \n
        pos     -- the position of the LSB of the value (default: 0x0)"""
        value = -999
        if (self.deviceOpen()):
            try:
                stri = "ev<" + str(address) + " " + str(mask) + " " + str(pos) + ">"
            except TypeError as e:
                print(e)
                return None
            self.writeString(stri)
            try:
                value = int(self.readLine())
            except TypeError as e:
                print(e)

        return value

    def readAddress(self, adr: str = None) -> int:
        """Reads the value of an address

        Keyword Arguments:

        adr -- The adress in form of an int"""
        if (adr == None):
            print("No address")
            return
        val = -999
        req = 'ea<' + str(adr) + '>'
        self.writeString(req)
        val = self.readLine()
        return val

    def readLine(self, dec: str = 'utf-8') -> str:
        """Reads a line from the serial bus.

        Keyword Arguments:

        dec -- the encoding as a string(default: utf-8)

        Returns the string or None, if reading failed"""
        k = 0
        if (self.deviceOpen()):
            try:
                k = str(self.linereader.readline().decode(dec))
            except SerialException as e:
                print(e)
            except AttributeError as e:
                print(e)
            except SerialTimeoutException:
                print("Timout while reading string. (Serial bus timed out)")

        return k

    # Write Functions
    def writeEEPROMValue(self, address: str, value: int, mask: str, pos: str) -> int:
        """
        Writes a value to the EEPROM of the ACS71020

        Keyword Arguments:

        adress  -- Address of EEPROM-Register \n
        value   -- EEPROM Value \n
        mask    -- Value Mask \n
        pos     -- Position of LSB of value

        Returns 1 if successful, 0 otherwise
        """
        if (self.deviceOpen()):
            stream = 'ww<' + str(address) + ' ' + str(value) + ' ' + str(mask) + ' ' + str(pos) + '>'
            self.writeString(stream)
        else:
            print("Cannot write: device not open")
            return 0

        return 1

    def writeIntValue(self, adr: str, val: int) -> None:
        """Writes a value to an address

        Keyword Arguments:

        adr -- address to be written to

        val -- value to write"""
        req = 'wa<'+str(adr)+' '+str(val)+'>'
        self.writeString(req)

    def writeString(self, string: str) -> None:
        """Writes a string to the serial Bus

        Keyword Arguments:

        string -- the requested string"""
        if (self.deviceOpen()):
            try:
                self.sBus.write(string.encode())
            except SerialTimeoutException:
                print(
                    "Timeout while writing. Not able to reach the teensy or is it busy?")
            except SerialException as e:
                print("Serial Error", e)
            except UnicodeError:
                print("Error in encoding")

    # Getter

    def getPorts(self) -> list[str]:
        """Reads all ports connected to the computer

        Returns a list with all port names, compatible to open with openPort() function"""
        self.port_list = [str(port[0])
                          for port in serial.tools.list_ports.comports()]
        if len(self.port_list) == 0:
            self.port_list.append("No serial port found")

        return self.port_list

    def getCurrentPort(self):
        """Returns the current connected port"""
        return self.currPort

    # Setter

    def setReadTimeout(self, t: int = 2) -> None:
        """Sets the read timeout of the serial bus

        Keyword Arguments:

        t -- integer timeout in seconds (default: 2)"""
        self.readTimeout = t
        self.openPort(self.currPort)
        print("Read Timeout set to ", t, "seconds")

    def setWriteTimeout(self, t: int = 2) -> None:
        """Sets the write timeout of the serial bus

        Keyword arguments:

        t -- integer timeout in seconds (default: 2)"""
        self.writeTimeout = t
        self.openPort(self.currPort)
        print("Write Timeout set to ", t, "seconds")

    # Auxiliary

    def flushBus(self) -> None:
        """Deletes input buffer and reconnects to clear all buffered Data"""
        if self.deviceOpen():
            self.linereader.flush()
            self.sBus.reset_input_buffer()

    def inwaiting(self) -> tuple[int, int]:
        """Returns a tuple of in-waiting in serial bus and buffered data in linereader helperclass

        (in_waiting, buffered)"""
        if self.deviceOpen():
            return (self.sBus.in_waiting, self.linereader.getBufLen())

    def stopReading(self, stop: bool) -> None:
        """Stops reading of linereader helperclass"""
        self.linereader.stopReading(stop)


##############################################################################################################
################                        Readline Helperclass                          ########################
##############################################################################################################


class ReadLine:
    """Linereader Helperclass, which introduces a buffer for incoming values. It is
    faster than the built-in function readline of pyserial."""

    def __init__(self, s: serial.Serial):
        """Inits the helperclass with an empty buffer

        Keyword Arguments:

        s -- the serialBus instance"""
        self.buf = bytearray()
        self.s = s
        self.stopped = False

    def readline(self) -> bytearray:
        """Reads a Line from the Serial bus, without a newline character.
        reads a maximum of 4096 bytes at once.

        Returns a bytearray with the line
        """
        self.counter = 0
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(8192, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i]
                self.buf[0:] = data[i + 1:]
                self.counter = 0
                return r
            else:
                if (self.stopped):
                    self.buf = bytearray()
                    return
                else:
                    if self.counter < 2:
                        self.buf.extend(data)
                        self.counter += 1
                    else:
                        self.counter = 0
                        raise SerialTimeoutException

    def getBufLen(self) -> int:
        """Returns the number of bytes in the buffer"""
        return len(self.buf)

    def flush(self) -> None:
        """Empties the buffer"""
        self.buf = bytearray()

    def stopReading(self, stop):
        """Defines if the readline function returns a value when reading a stream """
        self.stopped = stop
