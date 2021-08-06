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

from tkinter import ttk, PhotoImage


def initStyles():
    """Configures Styles"""
    bgColor = "#ECECEC"
    baseFont = 'Arial'

    rbo = PhotoImage(file="rbo.png")
    rbp = PhotoImage(file="rbp.png")

    style = ttk.Style()
    style.configure("TLabel", font=(
        baseFont, 12), background=bgColor)

    style.configure("Titletext.TLabel", font=(baseFont, 16))
    style.configure("Calibtext.TLabel", font=(baseFont, 14), height=10)
    style.configure("TButton", font=(baseFont, 12))
    #style.configure("Calibbtn.TButton", font=(baseFont, 12))
    style.configure("Calibbtn.TButton", borderwidth=0, highlightthickness=0)
    style.configure("TCombobox", font=(baseFont, 12))
    style.configure("TFrame", borderwidth=0, background=bgColor)
    style.configure("TEntry", font=(baseFont, 12))
    style.configure("Treeview", font=(baseFont, 12))
    style.configure("TRadiobutton", font=(baseFont, 12))
