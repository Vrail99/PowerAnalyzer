# Bachelorarbeit Sebastian Zisch
# Implementierung eines Evaluierungsboards für den ACS71020 Leistungsmesschip
# in Kombination mit einem Teensy 4


# Class main
import serial_device
import tkinter as tk
from tkinter import simpledialog
from tkinter import ttk

# Own modules
from settingsPage import SettingsPage
from livePage import LivePage
from mainPage import MainPage

# Matplot Imports
from matplotlib import style

style.use('ggplot')


############################################################################################################################
################                        Backbone / Main Window class                                ########################
############################################################################################################################


class mainWindow(tk.Tk):
    def __init__(self, *args, **kwargs):
        # Window Setup
        tk.Tk.__init__(self)
        version = "1.0"
        # Height and width for Windows
        self.width = 600
        self.height = 450
        assert self.width % 2 == 0
        assert self.width % 2 == 0
        self.title("Power Analyzer v." + version)

        # Window Container for Pages
        pgcont = ttk.Frame(self, relief="raised")  # Registercardcontroller
        pgcont.pack(fill="x")

        cont = ttk.Frame(self, relief="groove")  # Windowcontroller
        cont.pack(fill="both", expand=True)
        cont.grid_columnconfigure(0, weight=1)
        cont.grid_rowconfigure(0, weight=1)
        gridform = True  # Window Appearance

        btncont = ttk.Frame(self, relief="raised")  # Buttoncontroller
        btncont.pack(fill="x")

        # Serial Device, serial devices combobox
        self.sBus = serial_device.SerialBus()  # Serial bus instance
        self.allDevices = tk.StringVar()  # Serial devices

        # Pages in Frame
        self.pagenames = (MainPage, LivePage, SettingsPage)
        self.pages = {}  # Page dictionary
        for F in self.pagenames:
            page = F(cont, self)
            self.pages[F] = page
            if gridform:
                page.grid(column=0, row=0, sticky="nsew")
        if not gridform:
            self.pages[MainPage].grid(
                column=0, columnspan=2, row=0, sticky="nsew")
            self.pages[LivePage].grid(column=0, row=1)
            self.pages[SettingsPage].grid(column=1, row=1)

        # Create Menu
        self._createMenu()

        # Top Bar with Tab-Selection
        self.p1 = ttk.Button(pgcont, text="Main",
                             command=lambda: self._show_frame(MainPage))
        self.p2 = ttk.Button(pgcont, text="Live Measurement",
                             command=lambda: self._show_frame(LivePage))
        self.p3 = ttk.Button(pgcont, text="EEPROM Settings",
                             command=lambda: self._show_frame(SettingsPage))
        self.p1.pack(side="left")
        self.p2.pack(side="left")
        self.p3.pack(side="left")

        # BottomBar with Statuslabel
        self.l_status = ttk.Label(
            btncont, text="Not connected", width=self.width / 2)
        self.connbtn = ttk.Button(
            btncont, text="Connect", command=self._openSerial)
        self.disconnbtn = ttk.Button(
            btncont, text="Disconnect", command=self._closeSerial)
        self.opt = ttk.Combobox(btncont, textvariable=self.allDevices)

        # Placement of status bar
        self.l_status.pack(side="left")
        self.opt.pack(side="right")
        self.connbtn.pack(side="right")
        self.disconnbtn.pack(side="right")

        # Only necessary in grid view
        self._show_frame(MainPage)
        # Init serial ports
        self._searchSerialPorts()
        self._openSerial()

    #
    #   Creating the Menu
    #

    def _createMenu(self):
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

    #
    #   Read Timeout of Serial Bus
    #

    def setReadTimeout(self):
        self.sBus.setReadTimeout(self.rTimeout.get())

    #
    #   Write Timeout of Serial Bus
    #

    def setWriteTimeout(self):
        self.sBus.setWriteTimeout(self.wTimeout.get())

    #
    #   Opens a Serial Port
    #   @param port
    #           String of Port, example: "COM1" or "/dev/tty/ACM0". Defaults to no port, chooses from combobox

    def _openSerial(self, port=None):
        if (port == None):
            try:
                port = self.opt.get()
            except:
                print("Not a valid Port")

        err = self.sBus.openPort(port)
        if (err == 1):
            self.l_status["text"] = "Connected to " + \
                                    self.sBus.getCurrentPort()
            self.pages[LivePage].openScope_btn["state"] = tk.NORMAL
            self.pages[LivePage].get_btn["state"] = tk.NORMAL
            self.pages[LivePage].get_single_btn["state"] = tk.NORMAL
            self.pages[LivePage].readCont_btn["state"] = tk.NORMAL
            self.pages[LivePage].stopRead_btn["state"] = tk.NORMAL
            self.pages[LivePage].timeSync_btn["state"] = tk.NORMAL
            self.pages[SettingsPage].initBtn["state"] = tk.NORMAL
            self.pages[SettingsPage].readBtn["state"] = tk.NORMAL
            self.pages[SettingsPage].writeBtn["state"] = tk.NORMAL

    #
    #   Closes the Serial Port
    #

    def _closeSerial(self):
        self.sBus.flushBus()
        self.sBus.closePort()
        self.l_status["text"] = "Not connected"
        self.pages[LivePage].openScope_btn["state"] = tk.DISABLED
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

    def _show_frame(self, cont):
        t = self.pages[cont]
        t.tkraise()

    #
    #   Getter Functions
    #
    def _getWidth(self):
        return self.width

    def _getHeight(self):
        return self.height

    def _getSerialBus(self):
        return self.sBus

    def _setIgnoreVoltageLevel(self):
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
        if (self._getSerialBus().deviceOpen()):
            self.sBus.writeString('x')
            self._closeSerial()
        # if tk.messagebox.askyesno("Save current settings","Do you want to save the current settings?"):
        #    self._saveSettings()

        self.destroy()


app = None

if __name__ == '__main__':
    app = mainWindow()
    app.protocol("WM_DELETE_WINDOW", app.endProgram)
    app.mainloop()
