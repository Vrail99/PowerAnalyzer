"""GUI for the Power Analyzer based on an ACS71020 Power Monitoring IC"""


# Class main
from tkinter.constants import LEFT
import serial_device
import tkinter as tk
from tkinter import simpledialog
from tkinter import ttk

# Own modules
from settingsPage import SettingsPage
from livePage import LivePage
from mainPage import MainPage
from scopePage import ScopeWindow

# Matplot Imports
from matplotlib import style

style.use('ggplot')


############################################################################################################################
################                        Backbone / Main Window class                                ########################
############################################################################################################################


class mainWindow(tk.Tk):
    def __init__(self, pages: list, *args, **kwargs) -> None:
        """Initializes 
            - the serial bus,
            - individual pages as classes,
            - the menu

            and looks for serial ports.

        """
        # Window Setup
        tk.Tk.__init__(self)
        self.version = "1.0"

        # Height and width for Windows
        self.width = 680
        self.height = 450

        # Serial Device, serial devices combobox
        self.sBus = serial_device.SerialBus()  # Serial bus instance
        self.allDevices = tk.StringVar()  # Serial devices

        # Pages in Frame
        self.pagenames = pages

        # GUI Setup
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

        self.title("Power Analyzer v." + self.version)

        # Controller for Page Buttons
        page_contr = ttk.Frame(self, relief="raised")
        page_contr.pack(fill="x")

        # Window Container for Pages
        main_cont = ttk.Frame(self, relief="groove")  # Windowcontroller
        main_cont.pack(fill="both", expand=True)
        main_cont.grid_columnconfigure(0, weight=1)
        main_cont.grid_rowconfigure(0, weight=1)

        status_buttons = ttk.Frame(self, relief="raised")  # Buttoncontroller
        status_buttons.pack(fill="x")

        # Top Bar with Tab-Selection
        b1 = ttk.Button(
            page_contr, text="Main", command=lambda: self._show_frame(MainPage))
        b1.pack(side=LEFT)
        b2 = ttk.Button(
            page_contr, text="Values", command=lambda: self._show_frame(LivePage))
        b2.pack(side=LEFT)
        b3 = ttk.Button(
            page_contr, text="Settings", command=lambda: self._show_frame(SettingsPage))
        b3.pack(side=LEFT)
        self.scopeButton = ttk.Button(
            page_contr, text="Scope", state='disabled', command=lambda: self._openScope())
        self.scopeButton.pack(side=LEFT)

        # BottomBar with Statuslabel
        self.l_status = ttk.Label(
            status_buttons, text="Not connected", width=self.width / 2)
        self.connbtn = ttk.Button(
            status_buttons, text="Connect", command=lambda: self._openSerial())
        self.disconnbtn = ttk.Button(
            status_buttons, text="Disconnect", state=tk.DISABLED, command=lambda: self._closeSerial())
        self.reloadbtn = ttk.Button(
            status_buttons, text="⟲", command=lambda: self._searchSerialPorts())  # Circle: U+21BA u"\21ba"
        self.opt = ttk.Combobox(status_buttons, textvariable=self.allDevices)

        # Placement of status bar
        self.l_status.pack(side="left")
        self.opt.pack(side="right")
        self.connbtn.pack(side="right")
        self.disconnbtn.pack(side="right")
        self.reloadbtn.pack(side="right")

        # Add pages to the frame
        self._addPages(main_cont, pages)

    def _addPages(self, frame: tk.Frame, pageclasses: list, gridform: bool = False) -> None:
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
            label="0s", value=0, variable=self.rTimeout, command=self.setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="1s", value=1, variable=self.rTimeout, command=self.setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="2s", value=2, variable=self.rTimeout, command=self.setReadTimeout)
        mReadTimeout.add_radiobutton(
            label="5s", value=5, variable=self.rTimeout, command=self.setReadTimeout)
        mWriteTimeout.add_radiobutton(
            label="0s", value=0, variable=self.wTimeout, command=self.setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="1s", value=1, variable=self.wTimeout, command=self.setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="2s", value=2, variable=self.wTimeout, command=self.setWriteTimeout)
        mWriteTimeout.add_radiobutton(
            label="5s", value=5, variable=self.wTimeout, command=self.setWriteTimeout)

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

    def setReadTimeout(self) -> None:
        """Read timeout of serial bus. Reads the value from the top Menu"""
        self.sBus.setReadTimeout(self.rTimeout.get())

    def setWriteTimeout(self) -> None:
        """Write timeout of serial bus. Reads the value from menu"""
        self.sBus.setWriteTimeout(self.wTimeout.get())

    def _openSerial(self, port: str = None) -> None:
        """Takes a port name and tries to open it.

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
            self.pages[LivePage].get_btn["state"] = tk.NORMAL
            self.pages[LivePage].get_single_btn["state"] = tk.NORMAL
            self.pages[LivePage].readCont_btn["state"] = tk.NORMAL
            self.pages[LivePage].stopRead_btn["state"] = tk.NORMAL
            self.pages[LivePage].timeSync_btn["state"] = tk.NORMAL
            self.pages[SettingsPage].initBtn["state"] = tk.NORMAL
            self.pages[SettingsPage].readBtn["state"] = tk.NORMAL
            self.pages[SettingsPage].writeBtn["state"] = tk.NORMAL

    def _openScope(self):
        print("Opening Scope")
        # Create a New Window for Scope Page
        self.scopeWindow = tk.Toplevel(self, background='#d9d9d9')
        self.liveScope = ScopeWindow(self.scopeWindow, self.sBus)
        self.scopeWindow.protocol("WM_DELETE_WINDOW", self.endScopeProgram)

    def endScopeProgram(self):
        self.liveScope._stopCodes()
        self.scopeWindow.destroy()

    #
    #   Closes the Serial Port
    #

    def _closeSerial(self):
        self.sBus.flushBus()
        self.sBus.closePort()
        self.l_status["text"] = "Not connected"
        self.disconnbtn["state"] = tk.DISABLED
        self.scopeButton["state"] = tk.DISABLED
        self.pages[LivePage].get_btn["state"] = tk.DISABLED
        self.pages[LivePage].get_single_btn["state"] = tk.DISABLED
        self.pages[LivePage].readCont_btn["state"] = tk.DISABLED
        self.pages[LivePage].stopRead_btn["state"] = tk.DISABLED
        self.pages[LivePage].timeSync_btn["state"] = tk.DISABLED
        self.pages[SettingsPage].initBtn["state"] = tk.DISABLED
        self.pages[SettingsPage].readBtn["state"] = tk.DISABLED
        self.pages[SettingsPage].writeBtn["state"] = tk.DISABLED

    #
    #   Searches all available Serial Ports and add them to the menubar and the combobox
    #

    def _searchSerialPorts(self):
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

    def _show_frame(self, frame: tk.Frame):
        """Raises the frame on top of all others

            Keyword Arguments: 

            frame -- tk.Frame Object to be raised on top"""
        t = self.pages[frame]
        t.tkraise()

    #
    #   Getter Functions
    #
    def getWidth(self) -> int:
        """Returns width of the window"""
        return self.width

    def getHeight(self) -> int:
        """Returns height of the window"""
        return self.height

    def getSerialBus(self) -> serial_device.SerialBus:
        """Returns the current serial bus instance"""
        return self.sBus

    def _setIgnoreVoltageLevel(self):
        """Tells the teensy to ignore the current voltage level"""
        self.sBus.writeString('cv')

    def _setReadInterval(self):
        k = tk.simpledialog.askinteger(
            "Live Measurement Read Time(ms)", "Live Measurement Read Time (ms)\nInterval = [1,1000]", maxvalue=1000,
            minvalue=1)
        self.pages[LivePage].readDelay = k

    def endProgram(self):
        try:
            self.pages[LivePage].endScopeProgram()
        except:
            pass
        if (self.getSerialBus().deviceOpen()):
            self.sBus.writeString('x')
            self._closeSerial()
        # if tk.messagebox.askyesno("Save current settings","Do you want to save the current settings?"):
        #    self._saveSettings()

        self.destroy()


app = None

if __name__ == '__main__':
    p = [MainPage, LivePage, SettingsPage]
    app = mainWindow(p)
    app.protocol("WM_DELETE_WINDOW", app.endProgram)
    app.mainloop()
