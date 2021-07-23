# Matplot Imports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as anim
import matplotlib.pyplot as plt
import time
import _thread
from tkinter import ttk
import tkinter as tk
import numpy as np
from serial.serialposix import Serial
import utils
import tkinter.filedialog
import serial_device


class ScopeWindow(tk.Toplevel):
    """A tkinter toplevel instance for viewing the waveforms."""

    def __init__(self, controller: ttk.Frame, bus: serial_device.SerialBus) -> None:
        """Inits the Function

        Keyword Arguments:

        controller -- the backend class with base functionality

        bus -- the serial bus instance for communication
        """
        super().__init__(controller, background='#d9d9d9', takefocus=True)
        self.contr = controller
        self.sBus = bus
        self.name = "Scope"

        # Oscilloscope Variables
        # Sampling time, default: 200ms (10 50Hz Periods) -> Standard
        # Bin-Width: 5Hz -> From sampling time
        # Resulting Sample Size with 31 250 Hz Sampling Frequency: 6250 Sa
        self.sampleWindow = 200  # ms
        self.sampleFrequency = 31250  # Hz
        self.storageWidth = 6250  # SampleWindow*sampleFrequency, should be constant
        self.samplingDone = False

        self.logfile_counter = 0  # Counter for logfiles
        self.logfile = 0

        self.ratedVoltage = 366  # V_pk
        self.ratedCurrent = 15  # A_pk
        self.vcodes = []  # Buffer for codedisplay
        self.vcodes2 = []  # 2nd Buffer for appending vcodes
        self.icodes = []
        self.icodes2 = []
        self.precision = 3  # Numbers behind decimal point
        self.animRate = 50  # Milliseconds, Graph update time
        self.showPlots = 0  # Mode-switch

        self.fullFFT = []  # For 150-Period
        self.fullTHD = 0  # THD over 150 Periods value
        self.thdCounter = 0  # Counter for 10-period thd and fft

        self.currRange = 1  # 0: automode
        self.voltRange = 350  # Volt
        self.timeRange = 200  # Milliseconds
        self.freq_range = 2000  # Hz

        self.runCodeThread = False  # Status flag for readCode Thread

        # Plotfigure and axes
        self.plotfig = plt.Figure(
            figsize=(860 * 0.0104166667, 400 * 0.0104166667), dpi=100)
        self.vc_plot = self.plotfig.add_subplot(2, 1, 1)
        self.ic_plot = self.plotfig.add_subplot(2, 1, 2)
        self.plotfig.patch.set_facecolor('#d9d9d9')

        self._guiSetup()

        self.ani = anim.FuncAnimation(
            self.plotfig, self._animate, interval=self.animRate)

    def _guiSetup(self) -> None:
        """Sets up UI Elements"""
        topFrame = ttk.Frame(self, style="TFrame")
        radioFrm = ttk.Frame(topFrame, style="TFrame")
        btnFrm = ttk.Frame(topFrame, style="TFrame")
        self.scopeFrame = ttk.Frame(self, style="TFrame")
        topFrame.pack()
        btnFrm.pack(side=tk.LEFT, pady=5, padx=5)
        radioFrm.pack(side=tk.LEFT, pady=5, padx=5)
        self.scopeFrame.pack()

        self.startBtn = ttk.Button(
            btnFrm, text="Start", style="TButton", command=self._startCodes)
        self.stopBtn = ttk.Button(btnFrm, text="Stop", style="TButton", command=self._stopCodes)
        self.saveBtn = ttk.Button(
            btnFrm, text="Save", style="TButton", command=self._saveWaveform)
        self.startBtn.pack()
        self.stopBtn.pack()
        self.saveBtn.pack()

        self.modevar = tk.IntVar()
        self.modeSelectV = ttk.Radiobutton(radioFrm, text="Voltage", style="TRadiobutton",
                                           variable=self.modevar, value=0, width=10,
                                           command=self._changeMode)
        self.modeSelectV.pack()
        self.modeSelectI = ttk.Radiobutton(radioFrm, text="Current", style="TRadiobutton",
                                           variable=self.modevar, value=1, width=10,
                                           command=self._changeMode)
        self.modeSelectI.pack()
        self.modeSelectB = ttk.Radiobutton(radioFrm, text="Both", style="TRadiobutton",
                                           variable=self.modevar, value=2, width=10,
                                           command=self._changeMode)
        self.modeSelectB.pack()
        self.modeSelectP = ttk.Radiobutton(radioFrm, text="Power", style="TRadiobutton",
                                           variable=self.modevar, value=4, width=10,
                                           command=self._changeMode)
        self.modeSelectP.pack()

        self.endButton = ttk.Button(
            self.scopeFrame, text="Exit", command=self.closeScope)

        # FFT Plot:
        canv = FigureCanvasTkAgg(self.plotfig, self.scopeFrame)
        canv.draw()
        canv.get_tk_widget().pack(fill="both")
        self.endButton.pack()

    def _changeMode(self) -> None:
        """Changes the mode of showPlots"""
        self.showPlots = self.modevar.get()

    def _animate(self, interval: int) -> None:
        """Animation Function for the plot
         Only displays data if a reading-thread is activated and samplingDone-flag is set
        Clears the plot

        Keyword Arguments:

        interval -- Update interval for plots in milliseconds"""
        if self.runCodeThread:
            if self.samplingDone:
                self.vc_plot.clear()
                self.ic_plot.clear()
                if (self.showPlots == 0 or self.showPlots == 4):  # Voltage mode / Power Mode
                    codeArray = np.array(self.vcodes)
                    xAxis, yAxis = utils.calculateFFT(
                        codeArray, self.sampleFrequency)
                    self.fftArray = yAxis
                    if (len(self.fullFFT) == 0):  # Add the FFT to Full FFT
                        self.fullFFT = pow(yAxis, 2)  # squared value
                    else:
                        self.fullFFT += pow(yAxis, 2)  # squared value

                    thd = round(utils.calcTHD(
                        yAxis, 17, logfile=self.logfile), self.precision)
                    print("Peak: {:.3f}, THD: {:.3f}".format(
                        max(codeArray), thd))

                    self.fullTHD += thd
                    self.thdCounter += 1
                    if self.thdCounter == 15:
                        self.thdCounter = 0
                        self.fullFFT = np.sqrt((1 / 15) * self.fullFFT)
                        fft_thd = round(utils.calcTHD(
                            self.fullFFT, 17, logfile=self.logfile), self.precision)
                        stri = "THD über 15 EinzelTHDs ohne Aufrechnungsvorschrift: {:.3f} %".format(
                            self.fullTHD / 15)
                        self.logfile.write(stri + "\n")
                        print(stri)
                        stri = "THD der FFT über 150 Perioden: {:.3f} %".format(
                            fft_thd)
                        self.logfile.write(stri + "\n")
                        print(stri)

                    # Plot
                    tmp = np.linspace(0, self.sampleWindow, len(codeArray))
                    self.vc_plot.plot(tmp, codeArray)
                    # Cosmetics
                    self.vc_plot.set_xlim(0, self.timeRange)
                    self.vc_plot.set_ylim(-self.voltRange, self.voltRange)
                    self.vc_plot.set_title("Spannung (ms)")
                    self.vc_plot.set_xlabel("Zeit (ms)", loc='right')
                    self.vc_plot.set_ylabel("Amplitude (V)")
                    # Plot
                    self.ic_plot.stem(xAxis, yAxis, markerfmt=".")
                    # Cosmetics
                    self.ic_plot.set_title("Spannung Frequenzanteile")
                    self.ic_plot.set_xlabel("Frequenz (Hz)", loc='right')
                    self.ic_plot.set_ylabel("|F(x)|")
                    self.ic_plot.set_xlim(0, self.freq_range)
                elif (self.showPlots == 1):  # Current mode
                    codeArray = np.array(self.icodes)
                    xAxis, yAxis = utils.calculateFFT(
                        codeArray, self.sampleFrequency)
                    self.fftArray = yAxis

                    if (len(self.fullFFT) == 0):  # Add the FFT to Full FFT
                        self.fullFFT = pow(yAxis, 2)  # squared value
                    else:
                        self.fullFFT += pow(yAxis, 2)  # squared value
                    thd = round(utils.calcTHD(
                        yAxis, 17), self.precision)
                    print("Peak: {:.3f}, THD: {:.3f}".format(
                        max(codeArray), thd))

                    self.fullTHD += thd
                    self.thdCounter += 1
                    if self.thdCounter == 15:
                        self.thdCounter = 0
                        self.fullFFT = np.sqrt((1 / 15) * self.fullFFT)
                        fft_thd = round(utils.calcTHD(
                            self.fullFFT, 17, logfile=self.logfile), self.precision)
                        stri = "Current THD over 15 single THDs: {:.3f} %".format(
                            self.fullTHD / 15)
                        self.logfile.write(stri + "\n")
                        print(stri)
                        stri = "Current THD of FFT over 150 Periods: {:.3f} %".format(
                            fft_thd)
                        self.logfile.write(stri + "\n")
                        print(stri)
                    avg = round(sum(self.icodes) / len(self.icodes), 6)
                    print("Avg over", len(self.icodes), ":", avg)
                    # Plot
                    tmp = np.linspace(0, self.sampleWindow, len(codeArray))
                    self.vc_plot.plot(tmp, codeArray)
                    # Cosmetics
                    self.vc_plot.set_title("Strom")
                    if (self.currRange > 0):
                        self.vc_plot.set_ylim(
                            (-self.currRange, self.currRange))
                    self.vc_plot.set_xlim(0, self.timeRange)
                    self.vc_plot.set_xlabel("Zeit (ms)", loc='right')
                    self.vc_plot.set_ylabel("Amplitude (A)")
                    # Plot
                    self.ic_plot.stem(xAxis, yAxis, markerfmt=".")
                    # Cosmetics
                    self.ic_plot.set_title("Strom Frequenzanteile")
                    self.ic_plot.set_xlabel("Frequency (Hz)", loc='right')
                    self.ic_plot.set_ylabel("|F(x)|")
                    self.ic_plot.set_xlim(0, self.freq_range)
                else:
                    tmp = np.linspace(0, self.sampleWindow, len(self.vcodes))
                    # Plot
                    self.vc_plot.plot(tmp, self.vcodes)
                    # Cosmetics
                    self.vc_plot.set_title("Spannung")
                    self.vc_plot.set_xlim(0, self.timeRange)
                    self.vc_plot.set_ylim(-self.voltRange, self.voltRange)
                    self.vc_plot.set_xlabel("Zeit (ms)", loc='right')
                    self.vc_plot.set_ylabel("Amplitude (V)")

                    # Plot
                    tmp = np.linspace(0, self.sampleWindow, len(self.icodes))
                    self.ic_plot.plot(tmp, self.icodes)
                    # Cosmetics
                    self.ic_plot.set_title("Strom")
                    if (self.currRange > 0):
                        self.ic_plot.set_ylim(
                            (-self.currRange, self.currRange))
                    self.ic_plot.set_xlim(0, self.timeRange)
                    self.ic_plot.set_xlabel("Zeit (ms)", loc='right')
                    self.ic_plot.set_ylabel("Amplitude (A)")

                # Show Plots
                plt.show()
                # Reset Sampling
                self.samplingDone = False
                self.vcodes2 = []
                self.icodes2 = []

    def _readCodes(self, mode: str) -> None:
        """
        Thread-Function for reading Codes.

        Keyword Arguments:

        mode -- string to determine the runmode: 'vc': Voltage, 'ic': Current, 'bc': Both
        """

        # Activate Serial transmission of icodes and/or vcodes
        self.sBus.writeString(mode)

        while self.runCodeThread:  # When the thread is running
            data = self.sBus.readLine()  # Read a line
            if (data == 0 or data == None):
                print("None")
                continue
            else:
                # if multiple values are sent
                tmp = data.split(',')
                try:
                    ic = 0
                    vc = 0
                    if mode == 'bc':  # Convert both received datapoints to int
                        vc = int(tmp[0])
                        # Important: before Board-Rev. 2.0: invert value
                        ic = int(tmp[1][:-1])
                        # ic = -1*int(tmp[1][:-1]) #Only for Board Rev. 1.0
                    elif mode == 'ic':
                        ic = int(tmp[0])
                    else:
                        vc = int(tmp[0])

                    # Delete failed readings, if they appear
                    if (vc > 131072 or ic > 131072 or len(tmp) > 2):
                        vc = 0
                        ic = 0
                        print("Skipped value")
                        continue

                    a = self._appendIcode(ic)
                    if mode == 'pc':
                        b = self._appendPcode(vc)
                    else:
                        b = self._appendVcode(vc)

                except ValueError as e:
                    print("No Value read, only:", tmp, e)
                except IndexError:
                    print("Indexing wasn't possible:", tmp)
                except UnicodeDecodeError as e:
                    print("Error: Unicode Decode Error", e)

        self.sBus.writeString('x')
        self.startBtn["state"] = tk.NORMAL

    def _appendVcode(self, code: int) -> float:
        """Adds a VCode. When sample buffer is full, sets samplingDone flag

        Keyword Arguments:

        code -- integer vcode to be appended

        Returns the Integer value, if buffer is full and the converted number, if successful"""
        if len(self.vcodes2) < self.storageWidth:
            code = round(utils.ConvertSignedFixedPoint(
                code, 16, 17) * self.ratedVoltage, self.precision)
            self.vcodes2.append(code)

        elif (self.showPlots == 0 or self.showPlots == 2):
            self.vcodes = self.vcodes2
            self.samplingDone = True

        return code

    def _appendIcode(self, code: int) -> float:
        """Adds an ICode. When sample buffer is full, sets samplingDone flag

        Keyword Arguments:

        code -- integer icode to be appended

        Returns the Integer value, if buffer is full and the converted number, if successful"""
        if len(self.icodes2) < self.storageWidth:
            code = round(utils.ConvertSignedFixedPoint(
                code, 15, 17) * self.ratedCurrent, self.precision)
            self.icodes2.append(code)

        elif (self.showPlots == 1 or self.showPlots == 2):
            self.icodes = self.icodes2
            self.samplingDone = True

        return code

    def _appendPcode(self, code: int) -> float:
        """Adds a Power Code. When sample buffer is full, sets samplingDone flag

        Keyword Arguments:

        code -- integer pcode to be appended

        Returns the Integer value, if buffer is full and the converted number, if successful"""
        if len(self.vcodes2) < self.storageWidth:
            code = round(utils.ConvertSignedFixedPoint(
                code, 15, 17) * self.ratedCurrent*self.ratedVoltage, self.precision)
            self.vcodes2.append(code)

        elif (self.showPlots == 4):
            self.vcodes = self.vcodes2
            self.samplingDone = True

        return code

    def _startCodes(self) -> None:
        """Starts a new thread for reading codes, either in voltage, current or both mode"""
        if (not self.runCodeThread):
            self._setRadioState(tk.DISABLED)
            self.fullFFT = []
            self.logfile = open("logfile_" + str(self.logfile_counter) + ".txt", "w")
            self.fullTHD = 0
            self.thdCounter = 0
            if self.showPlots == 0:
                mode = 'vc'
            elif self.showPlots == 1:
                mode = 'ic'
            elif self.showPlots == 4:
                mode = 'pc'  # PCode- mode
            else:
                mode = 'bc'
            self.runCodeThread = True
            _thread.start_new_thread(self._readCodes, (mode,))
            self.sBus.stopReading(False)
            self.startBtn["state"] = tk.DISABLED

    def _stopCodes(self):
        """Stops the running thread and stops Teensy"""
        try:
            self.logfile.close()
            self.logfile_counter += 1
        except:
            pass
        self._setRadioState(tk.NORMAL)
        self.runCodeThread = False  # Stops the runCode Thread
        self.sBus.stopReading(True)
        self.sBus.writeString('x')
        time.sleep(.5)
        self.sBus.flushBus()

    def _saveWaveform(self):
        """Saves the current waveform to a file"""
        if not self.runCodeThread:
            file = tkinter.filedialog.asksaveasfile()
            if (self.showPlots == 0):  # Voltage waveform
                file.write("Voltage Values\n")
                for e in self.vcodes:
                    file.write(str(e)+"\n")
                file.write("FFT Values\n")
                for e in self.fftArray:
                    file.writelines(str(e)+"\n")
            elif (self.showPlots == 1):  # Current Waveform
                file.write("Current Values\n")
                for e in self.icodes:
                    file.write(str(e) + "\n")
                file.write("FFT Values\n")
                for e in self.fftArray:
                    file.writelines(str(e) + "\n")
            elif (self.showPlots == 2):  # Both Waves
                file.write("Voltage;Current\n")
                for v, i in zip(self.vcodes, self.icodes):
                    file.write(str(v)+";"+str(i)+"\n")
            file.close()

    def closeScope(self):
        """Closes the scope view"""
        self.destroy()

    def _setRadioState(self, s: str) -> None:
        """Sets the state of the Radiobuttons to s

        Keyword Arguments:

        s -- state for the radiobuttons (e.g. tk.NORMAL, tk.DISABLED)"""

        try:
            self.modeSelectV['state'] = s
            self.modeSelectB['state'] = s
            self.modeSelectI['state'] = s
            self.modeSelectP['state'] = s
        except:
            pass
