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
from tkinter import simpledialog
from tkinter import PhotoImage, ttk

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
        cancelButton = ttk.Button(buttonContainer, text="Cancel", command=self._endCalibration)
        cancelButton.pack(side=tk.LEFT)  # grid(column=0, row=0)
        self.nextButton = ttk.Button(buttonContainer, width=8, text="Next Step",
                                     command=lambda: self._callFrame(1))
        self.nextButton.pack(side=tk.RIGHT)  # grid(column=3, row=0)
        backButton = ttk.Button(buttonContainer, text="Back", command=lambda: self._callFrame(-1))
        backButton.pack(side=tk.RIGHT)  # grid(column=2, row=0)

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
        f = ttk.Frame(ref, style="TFrame")
        f.pack()
        self.gainEntry = ttk.Entry(f, style="TEntry")
        self.gainEntry.pack(side=tk.LEFT)
        ttk.Label(f, text="V").pack(side=tk.RIGHT)
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

    def _calcOffset(self) -> None:
        print("Offset Calculation")

    def _calcFineTrim(self) -> None:
        print("Calculating Fine Trim")
        try:
            expected = float(self.gainEntry.get())
        except TypeError as e:
            print(e)

        print("Value in Entry:", expected)

    def _addToCoarseGain(self, off: int) -> None:
        print("Added", off, "to crs_sns")

    def _calcVRMS_calFactor(self) -> None:
        self.VstatusLabel["text"] = "Calculating VRMS conversion factor"
        # Read current RMS value measured by the chip 9 times and calculate the mean
        calibFactor = 0
        for i in range(0, 9):
            calibFactor += self.sBus.readIntValue('vr')
            time.sleep(0.1)
        calibFactor = round(calibFactor/9)

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
        calibFactor = 0
        for i in range(0, 9):
            calibFactor += self.sBus.readIntValue('ir')
            time.sleep(0.1)
        calibFactor = round(calibFactor/9)

        # Read the entered Value
        try:
            exp = float(self.IRMSEntry.get())
        except TypeError as e:
            print(e)

        # Calculate the conversion factor
        conv = exp/calibFactor
        self.contr
        conversionFactor = utils.ConvertFloatToUnsignedFP(conv)

        # Write the request-string to the bus
        mode = 1
        req = 'wc<'+str(conversionFactor) + ' ' + str(mode) + '>'
        self.sBus.writeString(req)

        err = str(self.sBus.readLine())
        if (err == "SUCCESS"):
            self.IstatusLabel["text"] = "Calibration Successful.\nConversion-factor:" + str(conv)
