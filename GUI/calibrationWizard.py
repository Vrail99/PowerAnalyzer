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

import tkinter as tk
from tkinter import messagebox
from tkinter import PhotoImage, ttk
from typing import final

from matplotlib import style
import serial_device
import utils
import time


class CalibrationWizard(tk.Toplevel):
    def __init__(self, controller: ttk.Frame, bus: serial_device.SerialBus) -> None:
        super().__init__(controller, background='#d9d9d9', takefocus=True)
        self.geometry("480x360")
        self.wm_minsize(480, 360)
        self.wm_maxsize(480, 360)
        self.wm_title("Calibration Wizard")
        self.fn = 0
        self.calibrationSteps = []
        self.sBus = bus
        self.contr = controller

        self._guiSetup()

        self._callFrame(0)

    def _guiSetup(self) -> None:
        """Sets up UI Elements"""

        self.rbo = PhotoImage(file="rbo.png")

        # Main UI elements
        frameContainer = ttk.Frame(self, style="TFrame")
        frameContainer.pack(fill=tk.BOTH, expand=True)
        frameContainer.grid_columnconfigure(0, weight=1)
        frameContainer.grid_rowconfigure(0, weight=1)

        buttonContainer = ttk.Frame(self, style="TFrame")
        buttonContainer.pack(fill=tk.X)
        self.cancelButton = ttk.Button(buttonContainer, text="Cancel", command=self._endCalibration)
        self.cancelButton.pack(side=tk.LEFT)  # grid(column=0, row=0)
        self.nextButton = ttk.Button(buttonContainer, width=8, text="Next Step",
                                     command=lambda: self._callFrame(1))
        self.nextButton.pack(side=tk.RIGHT)  # grid(column=3, row=0)
        self.backButton = ttk.Button(buttonContainer, text="Back", command=lambda: self._callFrame(-1))
        self.backButton.pack(side=tk.RIGHT)  # grid(column=2, row=0)

        # Step-Frames
        zeroFrame = ttk.Frame(frameContainer, style="TFrame")
        zeroFrame.grid(row=0, column=0, sticky="nsew")
        self._zeroFrame_setup(zeroFrame)
        gainFrame = ttk.Frame(frameContainer, style="TFrame")
        gainFrame.grid(row=0, column=0, sticky="nsew")
        self._gainFrame_setup(gainFrame)
        ac_V_Frame = ttk.Frame(frameContainer, style="TFrame")
        ac_V_Frame.grid(row=0, column=0, sticky="nsew")
        self._ac_V_Frame_setup(ac_V_Frame)
        ac_I_Frame = ttk.Frame(frameContainer, style="TFrame")
        ac_I_Frame.grid(row=0, column=0, sticky="nsew")
        self._ac_I_Frame_setup(ac_I_Frame)

        # Add Frames to calibrationSteps to cycle through them
        self.calibrationSteps = [zeroFrame, gainFrame, ac_V_Frame, ac_I_Frame]

    def _zeroFrame_setup(self, ref: ttk.Frame) -> None:
        label = ttk.Label(ref, style="Calibtext.TLabel", justify='center', wraplength=int(480*0.7),
                          text="Zero-Offset-Calibration. \nPlease disconnect every voltage source"
                          " and press the button")
        label.pack(pady=5)
        calBtn = ttk.Button(ref, text="Toggle Calibration Mode",
                            command=self._toggleCalibrationMode)
        calBtn.pack()
        button = tk.Button(ref, image=self.rbo, borderwidth=0, highlightthickness=0,
                           bd=0, text="Zero", command=self._calcOffset)
        button.pack()
        self.zeroStatusLabel = ttk.Label(ref, justify='center')
        self.zeroStatusLabel.pack()

    def _gainFrame_setup(self, ref: ttk.Frame) -> None:
        label = ttk.Label(ref, style="Calibtext.TLabel", justify='center', wraplength=int(480*0.7),
                          text="Fine Trim-Calibration. \nPlease connect a known DC Current and"
                          " enter the value in Ampere.\n"
                          "Then press the button.")
        label.pack(pady=5)
        calBtn = ttk.Button(ref, text="Toggle Calibration Mode",
                            command=self._toggleCalibrationMode)
        calBtn.pack()

        f = ttk.Frame(ref, style="TFrame")
        f.pack()
        self.gainEntry = ttk.Entry(f, style="TEntry")
        self.gainEntry.pack(side=tk.LEFT)
        ttk.Label(f, text="A").pack(side=tk.RIGHT)
        button = tk.Button(ref, image=self.rbo, borderwidth=0, highlightthickness=0,
                           bd=0,  text="Calculate", command=self._calcFineTrim)
        button.pack()
        self.gainStatusLabel = ttk.Label(ref, justify='center')
        self.gainStatusLabel.pack()

    def _ac_V_Frame_setup(self, ref: ttk.Frame) -> None:
        label = ttk.Label(
            ref, style="Calibtext.TLabel", justify='center', wraplength=int(480 * 0.7),
            text="AC Voltage RMS Calibration. \nPlease connect a known AC voltage.\n"
            "Enter the value (in Volt) and press the button.")
        label.pack(pady=5)
        f = ttk.Frame(ref, style="TFrame")
        f.pack()
        self.VRMSEntry = ttk.Entry(f, style="TEntry")
        self.VRMSEntry.pack(side=tk.LEFT)
        ttk.Label(f, text="V").pack(side=tk.RIGHT)
        button = tk.Button(ref, image=self.rbo, borderwidth=0, highlightthickness=0,
                           bd=0, text="Calculate", command=self._calcVRMS_calFactor)
        button.pack()
        self.VstatusLabel = ttk.Label(ref, justify='center')
        self.VstatusLabel.pack()

    def _ac_I_Frame_setup(self, ref: ttk.Frame) -> None:
        label = ttk.Label(ref, style="Calibtext.TLabel", justify='center', wraplength=int(480*0.7),
                          text="AC Current RMS Calibration. \nPlease connect a load with known RMS current."
                          "Enter the expected value (in Ampere) and press the button")
        label.pack(pady=5)
        f = ttk.Frame(ref, style="TFrame")
        f.pack()
        self.IRMSEntry = ttk.Entry(f, style="TEntry")
        self.IRMSEntry.pack(side=tk.LEFT)
        ttk.Label(f, text="A").pack(side=tk.RIGHT)
        button = tk.Button(ref, image=self.rbo, borderwidth=0, highlightthickness=0,
                           bd=0, text="Calculate", command=self._calcIRMS_calFactor)
        button.pack()
        self.IstatusLabel = ttk.Label(ref, justify='center')
        self.IstatusLabel.pack()

    def _callFrame(self, offset: int) -> None:
        self.fn += offset
        maxF = len(self.calibrationSteps)
        if self.fn < 0 or self.fn >= maxF:
            return
        nextFrame = self.calibrationSteps[self.fn]
        nextFrame.tkraise()
        if self.fn == maxF-1:
            self.nextButton.config(text="End", command=self._endCalibration)
        else:
            self.nextButton.config(text="Next", command=lambda: self._callFrame(1))

    def _endCalibration(self) -> None:
        self.destroy()

    #---------------------------- Calculation functions -------------------------------------#

    def _readXTimes(self, address: str, number: int = 10, delay: float = 0.1) -> int:
        """Reads an address X times and calculates the mean

            address -- the address to be read from (e.g 'vr')

            number -- the amount of times to be read

            delay -- the time between reads

        Returns the rounded integer mean value
        """
        sum = 0
        for i in range(0, number):
            sum += self.sBus.readIntValue(address)
            time.sleep(delay)
        sum /= number

        return round(sum)

    def _readICodesXTimesConv(self, address: str, number: int, delay: float) -> float:
        sum = 0
        for i in range(0, number):
            if (address == 'id'):
                data = self.sBus.readIntValue(address)
                data = utils.ConvertSignedFixedPoint(data, 15, 17)*15
                sum += data
            else:
                sum += utils.ConvertUnsignedFixedPoint(self.sBus.readIntValue(address), 15, 17)*15
            time.sleep(delay)
        sum = sum / number
        return sum

    def _calcOffset(self) -> None:
        # Reset fine trim to 0
        self.sBus.writeEEPROMValue('0x0B', 0, '0xFFFC01FF', 0)
        nums = 256
        delay = nums*pow(10, -6)  # Read every nums microsecond
        sum = 0
        for i in range(0, 9):
            curr_value = self._readICodesXTimesConv('id', nums, delay)
            F = 0-curr_value
            stepsize = 64*(1/2184)
            sum += F/stepsize

        corr_steps = round(sum/9)
        current_offset = utils.ConvertTwosComplement(self.sBus.readEEPROMValue('0x0B', '0xFFFFFE00', 0), 9)
        final_offset = current_offset+corr_steps

        if final_offset > 255 or final_offset < -256:
            print("Offset too large/too small:", final_offset)

            return
        try:
            self.sBus.writeEEPROMValue('0x0B', final_offset, '0xFFFFFE00', 0)
            self.zeroStatusLabel["text"] = "Zero successful.\nThe qvo_fine offset value was calculated to\n"\
                + str(final_offset)
        except:
            self.zeroStatusLabel["text"] = "Writing the zero-value was not successful, due to a communication problem.\n"\
                "The calculated value for qvo_fine was:"+str(final_offset)

    def _toggleCalibrationMode(self):
        self.sBus.writeString('ca')  # Enter calibration mode
        ret = self.sBus.readLine()
        if(ret == "1"):
            self.gainStatusLabel["text"] = "In Calibration Mode"
            self.nextButton['state'] = tk.DISABLED
            self.backButton['state'] = tk.DISABLED
            self.cancelButton['state'] = tk.DISABLED
        else:
            self.nextButton['state'] = tk.NORMAL
            self.backButton['state'] = tk.NORMAL
            self.cancelButton['state'] = tk.NORMAL

    def _calcFineTrim(self) -> None:
        # crs_sns,0x0B,0xFFE3FFFF,18,0,7,0
        # sns_fine,0x0B,0xFFFC01FF,9,-256,255,0
        try:
            expected = float(self.gainEntry.get())
        except TypeError as e:
            print(e)

        nums = 256
        delay = nums*pow(10, -6)  # Read every nums microsecond
        stepSize = 100/511  # %

        sum = 0
        for i in range(0, 9):
            curr_value = self._readICodesXTimesConv('id', nums, delay)
            print("Measured Value:", curr_value)
            relErr = ((curr_value-expected)/curr_value)*100  # %
            sum += relErr

        measured_value = sum/9
        print("Measured Value", measured_value)
        corr_steps = -measured_value/stepSize
        print("corr_steps:", corr_steps)
        current_steps = self.sBus.readEEPROMValue('0x0B', '0xFFFC01FF', '9')
        current_steps = utils.ConvertTwosComplement(current_steps, 9)
        print("curr_steps:", current_steps)
        final_value = round(current_steps+corr_steps)

        if(final_value > 255):
            msgbox = messagebox.askokcancel(
                "The fine trim value cannot be set", message="The required fine-trim value of " +
                str(final_value) + "is too small.\n Increase the coarse gain by 1 and retry.\n" +
                "Do it automatically?")
            if msgbox == 1:
                before = int(self.sBus.readEEPROMValue('0x0B', '0xFFE3FFFF', '18'))
                if before == 7:
                    print("Cannot increase coarse gain")
                self.sBus.writeEEPROMValue('0x0B', str(before+1), '0xFFE3FFFF', '18')
            return
        elif(final_value < -256):
            msgbox = messagebox.askokcancel(
                "The fine trim value cannot be set", message="The required fine-trim value of " +
                str(final_value) + "is not reachable.\nDecrease the coarse gain by 1 and retry.\n" +
                "Do it automatically?")
            if msgbox == 1:
                before = int(self.sBus.readEEPROMValue('0x0B', '0xFFE3FFFF', '18'))
                if before == 0:
                    print("Cannot decrease coarse gain")
                self.sBus.writeEEPROMValue('0x0B', str(before-1), '0xFFE3FFFF', '18')
            return

        try:
            self.sBus.writeEEPROMValue('0x0B', final_value, '0xFFFC01FF', '9')
            self.gainStatusLabel["text"] = "Fine Trim successful.\nThe sns_fine value was calculated to\n"\
                + str(final_value)
        except:
            self.gainStatusLabel["text"] = "Writing the fine trim value was not successful, due to a communication problem.\n"\
                "The calculated value for qvo_fine was:"+str(final_value)

    def _calcVRMS_calFactor(self) -> None:
        self.VstatusLabel["text"] = "Calculating VRMS conversion factor"
        # Read current RMS value measured by the chip 9 times and calculate the mean
        calibFactor = self._readXTimes('vr')

        # Read the entered Value
        try:
            exp = float(self.VRMSEntry.get())
        except TypeError as e:
            print(e)

        # Calculate the Value at RSense
        exp = exp * 3000/4003000

        # Calculate the conversion factor
        conv = exp/calibFactor
        conversionFactor = utils.ConvertFloatToUnsignedFP(conv)

        # Write the request-string to the bus
        mode = 0
        req = 'wc<'+str(conversionFactor) + ' ' + str(mode) + '>'
        self.sBus.writeString(req)
        err = str(self.sBus.readLine())
        if (err == "SUCCESS"):
            self.VstatusLabel["text"] = "Calibration Successful.\nConversion-factor:" + str(conv)

    def _calcIRMS_calFactor(self) -> None:
        self.IstatusLabel["text"] = "Calculating IRMS conversion factor"
        # Read current RMS value measured by the chip 9 times and calculate the mean
        calibFactor = self._readXTimes('ir')

        # Read the entered Value
        try:
            exp = float(self.IRMSEntry.get())
        except TypeError as e:
            print(e)

        # Calculate the conversion factor
        conv = exp/calibFactor
        conversionFactor = utils.ConvertFloatToUnsignedFP(conv)

        # Write the request-string to the bus
        mode = 1
        req = 'wc<'+str(conversionFactor) + ' ' + str(mode) + '>'
        self.sBus.writeString(req)

        err = str(self.sBus.readLine())
        if (err == "SUCCESS"):
            self.IstatusLabel["text"] = "Calibration Successful.\nConversion-factor:" + str(conv)
