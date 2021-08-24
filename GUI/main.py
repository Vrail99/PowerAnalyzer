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


# Class main
import serial_device
import tkinter as tk
from tkinter import ttk
import time
import sys
import os

# Own modules
from settingsPage import SettingsPage
from livePage import LivePage
from mainPage import MainPage
from scopePage import ScopeWindow
from calibrationWizard import CalibrationWizard
import styles
import utils

# Matplot Imports
from matplotlib import style

style.use('ggplot')


############################################################################################################################
################                        Backbone / Main Window class                                ########################
############################################################################################################################


class mainWindow(tk.Tk):
    """tkinter.Tk() backend"""

    def __init__(self, pages: list, *args, **kwargs) -> None:
        """Initializes
            - the serial bus,
            - individual pages as classes,
            - the menu

            and looks for serial ports.

        """
        # TK init
        super().__init__()

        self.version = "1.1"

        # Height and width for Windows
        self.wm_minsize(700, 450)
        self.wm_maxsize(700, 450)
        self.width = 700
        self.height = 450
        self.title("Power Analyzer v." + self.version)

        # Serial Device, serial devices combobox
        self.sBus = serial_device.SerialBus()  # Serial bus instance
        self.allDevices = tk.StringVar()  # Serial devices

        # Pages in Frame
        self.pagenames = pages

        # GUI Setup
        styles.initStyles()
        self._guiSetup(self.pagenames)

        # Create Menu
        self._createMenu()

        # Init serial ports
        self._searchSerialPorts()
        self._openSerial()

    def _guiSetup(self, pages: list) -> None:
        """Creates frames, buttons, comboboxes.
            Adds the pages.

            Keyword Arguments:
            pages -- list of pages to be added. This list should contain the classes of names
        """

        # Controller for Page Buttons
        page_contr = ttk.Frame(self, style="TFrame")
        page_contr.pack(fill="x", expand=True)

        # Window Container for Pages
        main_cont = ttk.Frame(self, style="TFrame")  # Windowcontroller
        main_cont.pack(fill="both", expand=True)
        main_cont.grid_columnconfigure(0, weight=1)
        main_cont.grid_rowconfigure(0, weight=1)

        status_buttons = ttk.Frame(self, style="TFrame")  # Buttoncontroller
        status_buttons.pack(fill="x", expand=True)

        # Top Bar with Tab-Selection
        b1 = ttk.Button(
            page_contr, style="TButton", text="Main", command=lambda: self._show_frame(MainPage))
        b1.pack(side=tk.LEFT)
        b2 = ttk.Button(
            page_contr, style="TButton", text="Values", command=lambda: self._show_frame(LivePage))
        b2.pack(side=tk.LEFT)
        b3 = ttk.Button(page_contr, style="TButton", text="Settings",
                        command=lambda: self._show_frame(SettingsPage))
        b3.pack(side=tk.LEFT)
        self.scopeButton = ttk.Button(page_contr, style="TButton", text="Scope", state='disabled',
                                      command=lambda: self._openScope())
        self.scopeButton.pack(side=tk.LEFT)

        self.calButton = ttk.Button(page_contr, style="TButton", text="Calibration", state='disabled',
                                    command=lambda: self._openCalibration())
        self.calButton.pack(side=tk.LEFT)

        # BottomBar with Statuslabel
        self.l_status = ttk.Label(
            status_buttons, style="TLabel", text="Not connected", width=25)
        self.connbtn = ttk.Button(
            status_buttons, style="TButton", text="Connect", command=lambda: self._openSerial())
        self.disconnbtn = ttk.Button(
            status_buttons, style="TButton", text="Disconnect", state=tk.DISABLED,
            command=lambda: self._closeSerial())
        self.reloadbtn = ttk.Button(status_buttons, style="TButton", text="⟲",
                                    command=lambda: self._searchSerialPorts())  # Circle: U+21BA u"\21ba"
        self.opt = ttk.Combobox(
            status_buttons, style="TCombobox", textvariable=self.allDevices)

        # Placement of status bar
        self.l_status.grid(column=0, row=0)  # .pack(side=tk.LEFT)
        self.reloadbtn.grid(column=1, row=0)  # .pack(side=tk.RIGHT)
        self.disconnbtn.grid(column=2, row=0)  # .pack(side=tk.RIGHT)
        self.connbtn.grid(column=3, row=0)  # .pack(side=tk.RIGHT)
        self.opt.grid(column=4, row=0)  # .pack(side=tk.RIGHT)

        # Add pages to the frame
        self._addPages(main_cont, pages)

    def _addPages(self, frame: tk.Frame, pageclasses: list, gridform: bool = False) -> None:
        """Adds the pages in the list to the frame

        Keyword Arguments: \n
        frame       -- The parent Frame \n
        pageclasses -- The pages list, containing the pages as classes \n
        gridform    -- Page orientation in grid or as single window (default: False)"""
        self.pages = {}  # Page dictionary
        for F in pageclasses:
            page = F(frame, self)
            self.pages[F] = page
            if not gridform:
                page.grid(column=0, row=0, sticky="nsew")
        if gridform:
            self.pages[MainPage].grid(
                column=0, columnspan=2, row=0, sticky="nsew")
            self.pages[LivePage].grid(column=0, row=1)
            self.pages[SettingsPage].grid(column=1, row=1)
        else:
            # Only necessary in grid view
            self._show_frame(MainPage)

    def _createMenu(self) -> None:
        """Creates the menu bar"""

        mBar = tk.Menu(self)  # Create Menu

        mFile = tk.Menu(mBar, tearoff=0)  # File Menu
        mFile.add_command(label="End", command=self.endProgram)

        mOpt = tk.Menu(mBar, tearoff=0)
        mOpt.add_command(label="Find Ports", command=self._searchSerialPorts)
        self.mSerPor = tk.Menu(mOpt, tearoff=0)
        mOpt.add_cascade(label="Ports", menu=self.mSerPor)
        mOpt.add_separator()
        mReadTimeout = tk.Menu(mOpt, tearoff=0)
        mWriteTimeout = tk.Menu(mOpt, tearoff=0)
        mOpt.add_cascade(label="Write-Timeout", menu=mWriteTimeout)
        mOpt.add_cascade(label="Read-Timeout", menu=mReadTimeout)
        mOpt.add_separator()
        mOpt.add_command(label="Live Page Read Timing",
                         command=self._setReadInterval)
        mOpt.add_command(label="Ignore Voltage Level",
                         command=self._setIgnoreVoltageLevel)
        self.rTimeout = tk.IntVar(self, 2)
        self.wTimeout = tk.IntVar(self, 2)
        mOpt.add_separator()
        mReadTimeout.add_radiobutton(
            label="0s", value=0, variable=self.rTimeout, command=self._setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="1s", value=1, variable=self.rTimeout, command=self._setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="2s", value=2, variable=self.rTimeout, command=self._setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="5s", value=5, variable=self.rTimeout, command=self._setReadTimeout)
        mWriteTimeout.add_radiobutton(
            label="0s", value=0, variable=self.wTimeout, command=self._setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="1s", value=1, variable=self.wTimeout, command=self._setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="2s", value=2, variable=self.wTimeout, command=self._setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="5s", value=5, variable=self.wTimeout, command=self._setWriteTimeout)

        mOpt.add_separator()

        mView = tk.Menu(mBar, tearoff=0)
        mView.add_command(label="Main Page",
                          command=lambda: self._show_frame(MainPage))
        mView.add_command(label="Live View",
                          command=lambda: self._show_frame(LivePage))
        mView.add_command(label="Settings Page",
                          command=lambda: self._show_frame(SettingsPage))

        mHelp = tk.Menu(mBar, tearoff=0)

        mBar.add_cascade(label="File", menu=mFile)
        mBar.add_cascade(label="Options", menu=mOpt)
        mBar.add_cascade(label="View", menu=mView)
        mBar.add_cascade(label="Help", menu=mHelp)

        self["menu"] = mBar

    def _setReadTimeout(self) -> None:
        """Read timeout of serial bus. Reads the value from the top Menu"""
        self.sBus.setReadTimeout(self.rTimeout.get())

    def _setWriteTimeout(self) -> None:
        """Write timeout of serial bus. Reads the value from menu"""
        self.sBus.setWriteTimeout(self.wTimeout.get())

    def _openSerial(self, port: str = None) -> None:
        """Takes a port name and tries to open it. Enables GUI Elements if successful

        Keyword Arguments:

        port -- string with the name of the port. For example 'COM1' or '/dev/ttyACM0'
                default None

        """
        if (port == None):
            try:
                port = self.opt.get()
            except:
                print("Not a valid Port")

        err = self.sBus.openPort(port)
        if (err == 1):
            self.l_status["text"] = "Connected to " + \
                self.sBus.getCurrentPort()
            self.disconnbtn["state"] = tk.NORMAL
            self.scopeButton["state"] = tk.NORMAL
            self.calButton["state"] = tk.NORMAL
            self.pages[LivePage].get_btn["state"] = tk.NORMAL
            self.pages[LivePage].get_single_btn["state"] = tk.NORMAL
            self.pages[LivePage].readCont_btn["state"] = tk.NORMAL
            self.pages[LivePage].stopRead_btn["state"] = tk.NORMAL
            self.pages[LivePage].timeSync_btn["state"] = tk.NORMAL
            self.pages[SettingsPage].readBtn["state"] = tk.NORMAL
            # Get conversion factors
            try:
                self.sBus.writeString('es')
                tmp = str(self.sBus.readLine()).split(',')
                v_conv_factor = 0
                i_conv_factor = 0
                if (tmp[0] == "VRMS"):
                    v_conv_factor = utils.ConvertUnsignedFixedPoint(int(tmp[1]), 23, 24)
                tmp = str(self.sBus.readLine()).split(',')
                if (tmp[0] == "IRMS"):
                    i_conv_factor = utils.ConvertUnsignedFixedPoint(int(tmp[1]), 23, 24)
                self.pages[LivePage].setConversionFactors(v_conv_factor, i_conv_factor)
            except TypeError as e:
                print(e)

    def _openScope(self) -> None:
        """Opens the Oscilloscope-Window"""
        self.scopeWindow = ScopeWindow(self, self.sBus)
        self.scopeWindow.focus_set()
        self.scopeWindow.protocol("WM_DELETE_WINDOW", self._endScopeProgram)
        self.scopeWindow.attributes("-topmost", 'true')

    def _openCalibration(self) -> None:
        """Opens the Calibration Wizard"""
        self.calWindow = CalibrationWizard(self, self.sBus)
        self.calWindow.focus_set()
        self.calWindow.grab_set()
        self.calWindow.protocol("WM_DELETE_WINDOW", lambda: self.calWindow.destroy())
        self.calWindow.attributes("-topmost", 'true')

    def _endScopeProgram(self) -> None:
        """Destroys the scope window"""
        self.scopeWindow._stopCodes()
        self.scopeWindow.destroy()

    def _closeSerial(self) -> None:
        """Closes the serial connection, flushes the bus and
        disables GUI Elements which use this connection"""
        self.sBus.flushBus()
        self.sBus.closePort()
        self.l_status["text"] = "Not connected"
        self.disconnbtn["state"] = tk.DISABLED
        self.scopeButton["state"] = tk.DISABLED
        self.calButton["state"] = tk.DISABLED
        self.pages[LivePage].get_btn["state"] = tk.DISABLED
        self.pages[LivePage].get_single_btn["state"] = tk.DISABLED
        self.pages[LivePage].readCont_btn["state"] = tk.DISABLED
        self.pages[LivePage].stopRead_btn["state"] = tk.DISABLED
        self.pages[LivePage].timeSync_btn["state"] = tk.DISABLED
        self.pages[SettingsPage].readBtn["state"] = tk.DISABLED
        self.pages[SettingsPage].writeBtn["state"] = tk.DISABLED
        self.pages[SettingsPage].init = False

    def _searchSerialPorts(self) -> None:
        """Searches all available Serial Ports and add them to the menubar and the combobox"""
        # Check if already open
        if (self.sBus.deviceOpen()):
            print("Serial Connection already open, cannot search for ports")
            return
        # Get all Ports
        tmp = []
        for p in self.sBus.getPorts():
            tmp.insert(0, p)
        # Clear Menu
        self.mSerPor.delete(0, tk.END)
        for port in tmp:
            # Add Commands for each port. Name of port is the first element
            self.mSerPor.add_command(
                label=port, command=lambda: self._openSerial(port[0]))
        # Set default port value
        self.opt["values"] = tmp
        self.allDevices.set(tmp[0])

    def _show_frame(self, frame: tk.Frame) -> None:
        """Raises the frame on top of all others

            Keyword Arguments:

            frame -- tk.Frame Object to be raised on top"""
        t = self.pages[frame]
        t.tkraise()

    ##################################    Getter   ################################################
    def getWidth(self) -> int:
        """Returns width of the window"""
        return self.width

    def getHeight(self) -> int:
        """Returns height of the window"""
        return self.height

    def getSerialBus(self) -> serial_device.SerialBus:
        """Returns the current serial bus instance"""
        return self.sBus

    ##################################    Setter   ################################################

    def _setIgnoreVoltageLevel(self) -> None:
        """Tells the teensy to ignore the current voltage level. Cannot be reverted"""
        self.sBus.writeString('cv')

    def _setReadInterval(self) -> None:
        """Displays a dialog to change the values-view continuous-reading delay"""
        k = tk.simpledialog.askinteger(
            "Value Read Delay(ms)", "Value Read Delay (ms)\nInterval = [1,1000]", maxvalue=1000,
            minvalue=1)
        self.pages[LivePage].readDelay = k

    def endProgram(self) -> None:
        """Ends the program and destroys the Window"""
        try:
            self.pages[LivePage]._endScopeProgram()
        except:
            pass
        if (self.sBus.deviceOpen()):
            print("Closing program")
            self.sBus.writeString('x')
            time.sleep(1)
            self.sBus.writeString('bf')
            self.sBus.closePort()

        self.destroy()


if __name__ == '__main__':
    p = [MainPage, LivePage, SettingsPage]
    os.chdir(os.path.dirname(__file__))
    app = mainWindow(p)
    app.protocol("WM_DELETE_WINDOW", app.endProgram)
    app.mainloop()
