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

from tkinter import ttk
import os
import sys
from PIL import Image, ImageTk

############################################################################################################################
################                        Main Page Class                                             ########################
############################################################################################################################


class MainPage(ttk.Frame):
    def __init__(self, parent: ttk.Frame, controller) -> None:
        """
        Init of Page

        Keyword Arguments: \n
        parent -- Parent widget it is placed within \n
        controller -- controller class
        """
        """This is a very long line. It is used to test whether the autopep8 autoformatting works or not"""
        super().__init__(parent, style="TFrame")
        self.contr = controller
        self.sBus = controller.getSerialBus()
        self.name = "Main"

        self._guiSetup()

    def _guiSetup(self) -> None:
        """GUI Setup with picture and titlelabel"""
        acslabel = ttk.Label(
            self, text="ACS71020 Evaluation Adaptor", style="Titletext.TLabel")
        acslabel.pack()
        path = os.path.join(os.getcwd(), "acspic2.jpg")
        im = Image.open(path)
        photo = ImageTk.PhotoImage(im)
        label2 = ttk.Label(self, image=photo)
        label2.image = photo
        label2.pack()

    def _getName(self):
        return self.name
