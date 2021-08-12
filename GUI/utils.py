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

#
#   Module Utils
#   Conversion Functions and Calculation helpers
#
# ////////////////////////////////////////////////////////////////////////////////
# // Conversion functions
# ////////////////////////////////////////////////////////////////////////////////

import numpy as np


def ConvertUnsignedFixedPoint(inputValue, binaryPoint, width):
    """
    Convert an unsigned bitfield which is right justified, into a floating point number

      data        - the bitfield to be converted
      binaryPoint - the binary point (the bit to the left of the binary point)
      width       - the width of the bitfield
      returns     - the floating point number
    """
    mask = 0
    if (width == 32):
        mask = 0xFFFFFFFF
    else:
        mask = pow(2, width) - 1

    return (float(inputValue & mask) / float(pow(2, binaryPoint)))


def ConvertSignedFixedPoint(inputValue, binaryPoint, width):
    """
    Convert a signed bitfield which is right justified, into a floating point number

      data        - the bitfield to be sign extended then converted
      binaryPoint - the binary point (the bit to the left of the binary point)
      width       - the width of the bitfield
      returns     - the floating point number
    """
    signedValue = SignExtendBitfield(inputValue, width)
    return (float(signedValue) / pow(2, binaryPoint))


def SignExtendBitfield(data, width):
    """
    Sign extend a bitfield which if right justified

      data        - the bitfield to be sign extended
      width       - the width of the bitfield
      returns     - the sign extended bitfield
    """
    # If the bitfield is the width of the variable, don't bother trying to sign extend (it already is)
    if (width == 32):
        return int(data)

    x = int(data)
    mask = int(pow(2, (width - 1)))
    x = x & ((pow(2, width)) - 1)  # make sure the upper bits are zero

    return int((x ^ mask) - mask)


def ConvertFloatToUnsignedFP(f: float, bitwidth: int = 24, fraction: int = 23) -> int:
    """
    Converts a floating point number to a fixed point format of a bitwidth.

        f           -- the floating point number
        bitwidth    -- the required bitwidth
        fraction    -- the length of the fractional part

    Returns the fixed point representation of the number with the given parameters
    """
    scaling_factor = (1 << fraction)
    largest = (1 << bitwidth-fraction)-(pow(2, -(bitwidth-fraction)))
    if f > largest:
        print("{} doesn't fit into a {}-bit FP with {}-bit fractional part".format(f, bitwidth, fraction))
        return 0

    return round(f * scaling_factor)


#
# Converts a 2s complement number
#

def ConvertTwosComplement(data, bitwidth):
    """
    Converts a two's complement fixed point number
    """
    # Is the sign bit 0 ?
    if (data & (1 << (bitwidth - 1))) != 0:
        data = data - (1 << bitwidth)

    return data


"""
Calculates the FFT of a list of arrays.

    @param samples:         numpy.array of the samples
    @param sampleFrequency: sampleFrequency
"""


def calculateFFT(zeroSamples, sampleFrequency):
    # Calulcate the FFT
    fft = 2 * (np.fft.fft(zeroSamples) / len(zeroSamples))  # Normalize
    # Only use left half of the array
    fft = fft[range(int(len(zeroSamples) / 2))]

    # Calculate FFT Bins
    fBin = sampleFrequency / len(zeroSamples)
    frequencies = fBin * np.arange(0, int(len(zeroSamples) / 2))  # Frequency Axis

    return frequencies, abs(fft)


"""
Calculates the THD of frequencies

    @param samples:         numpy.array of the samples
    @param sampleFrequency: sampleFrequency
"""


def calcTHD(frequencies, order, logfile=None, baseFrequency=50, binsize=5):
    # Exmpl: 50Hz/5Hz = 10. Start with 2nd order Harmonic

    harmonicIndices = int(baseFrequency / binsize)
    baseHarmonic = frequencies[harmonicIndices]

    # Check if enough data is in the frequency array
    if ((order - 1) * harmonicIndices > len(frequencies)):
        print("Harmonic order(", order, ") out of range.")
        return 0
    thd = 0
    if (logfile != None):
        logfile.write("Ordnung;Value;RatioToBase\n")
        logfile.write("1.0;{:.6f};{:.6f}\n".format(baseHarmonic, baseHarmonic / baseHarmonic * 100))
    for i in range(harmonicIndices * 2, harmonicIndices * order, harmonicIndices):
        try:
            thd += pow((frequencies[i] / baseHarmonic), 2)
            if (logfile != None):
                ratio = "{:.6f};{:.6f}".format(frequencies[i], (frequencies[i] / baseHarmonic) * 100)
                logfile.write(str(i / harmonicIndices) + ";" + ratio + "\n")
        except:
            pass

    thd_root = np.sqrt(thd) * 100

    return thd_root


#
#   Calculates the VRMS of an array
#
def calcRMS(data):
    n = len(data)
    res = 0

    for i in data:
        res += pow(i, 2)

    res = np.sqrt((1 / n) * res)

    return res
