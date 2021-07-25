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


def initStyles():
    """Configures Styles"""
    bgColor = "#ECECEC"
    baseFont = 'Arial'

    labelstyle = ttk.Style()
    labelstyle.configure("TLabel", font=(
        baseFont, 12), background=bgColor)

    titlelabelstyle = ttk.Style()
    titlelabelstyle.configure("Titletext.TLabel", font=(baseFont, 16))

    buttonstyle = ttk.Style()
    buttonstyle.configure("TButton", font=(baseFont, 12))

    comboboxstyle = ttk.Style()
    comboboxstyle.configure("TCombobox", font=(baseFont, 12))

    framestyle = ttk.Style()
    framestyle.configure("TFrame", borderwidth=0, background=bgColor)

    entrystyle = ttk.Style()
    entrystyle.configure("TEntry", font=(baseFont, 12))

    treestyle = ttk.Style()
    treestyle.configure("Treeview", font=(baseFont, 12))

    radiostyle = ttk.Style()
    radiostyle.configure("TRadiobutton", font=(baseFont, 12))
