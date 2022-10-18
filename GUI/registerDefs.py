# Register classes for easier
class reg0x0B:
    def __init__(self, value) -> None:
        self.completeReg = value
        self._splitReg()

    def _setReg(self):
        self.completeReg = self.crc | self.iavgselen << 21 | self.crs_sns << 18 | \
            self.sns_fine << 9 | self.qvo_fine

    def _splitReg(self):
        self.crc = self.completeReg & 0xFFC00000
        self.iavgselen = self.completeReg >> 21 & 0x1
        self.crs_sns = self.completeReg >> 18 & 0x7
        self.sns_fine = self.completeReg >> 9 & 0x1FF
        self.qvo_fine = self.completeReg & 0x1FF

    def __getitem__(self, regNo):
        return getattr(self, regNo)

    def __setitem__(self, regNo: str, __value) -> None:
        try:
            self.__getattribute__(regNo)
        except AttributeError:
            print("Attribute", regNo, "not found")
            return

        setattr(self, regNo, __value)
        if(regNo == "completeReg"):
            self._splitReg()
        else:
            self._setReg()

    def __str__(self) -> str:
        string = self.__class__.__name__+":\n"
        for e in dir(self):
            if not e.startswith("_"):
                string += "{}: {}\n".format(e, self.__getattribute__(e))
        return string


class reg0x0C:
    def __init__(self, value) -> None:
        self.completeReg = value
        self._splitReg()

    def _setReg(self):
        self.completeReg = self.crc | self.rms_avg_2 << 7 | self.rms_avg_1

    def _splitReg(self):
        self.crc = self.completeReg & 0xFFFE00000
        self.rms_avg_2 = self.completeReg >> 7 & 0x3FF
        self.rms_avg_1 = self.completeReg & 0x7F

    def __getitem__(self, regNo):
        return getattr(self, regNo)

    def __setitem__(self, regNo: str, __value) -> None:
        try:
            self.__getattribute__(regNo)
        except AttributeError:
            print("Attribute", regNo, "not found")
            return

        setattr(self, regNo, __value)
        if(regNo == "completeReg"):
            self._splitReg()
        else:
            self._setReg()

    def __str__(self) -> str:
        string = self.__class__.__name__ + ":\n"
        for e in dir(self):
            if not e.startswith("_"):
                string += "{}: {}\n".format(e, self.__getattribute__(e))
        return string


class reg0x0D:
    def __init__(self, value) -> None:
        self.completeReg = value
        self._splitReg()

    def _splitReg(self):
        self.crc = self.completeReg & 0xFC001100
        self.squarewave_en = self.completeReg >> 25 & 0x1
        self.halfcycle_en = self.completeReg >> 24 & 0x1
        self.fltdly = self.completeReg >> 21 & 0x7
        self.fault = self.completeReg >> 13 & 0xFF
        self.chan_del_sel = self.completeReg >> 9 & 0x7
        self.ichan_del_en = self.completeReg >> 7 & 0x1
        self.pacc_trim = self.completeReg & 0x7F

    def _setReg(self):
        self.completeReg = self.crc | self.squarewave_en << 25 | self.halfcycle_en << 24 | \
            self.fltdly << 21 | self.fault << 13 | self.chan_del_sel << 9 | \
            self.ichan_del_en << 7 | self.pacc_trim

    def __getitem__(self, regNo):
        return getattr(self, regNo)

    def __setitem__(self, regNo: str, __value) -> None:
        try:
            self.__getattribute__(regNo)
        except AttributeError:
            print("Attribute", regNo, "not found")
            return

        setattr(self, regNo, __value)
        if(regNo == "completeReg"):
            self._splitReg()
        else:
            self._setReg()

    def __str__(self) -> str:
        string = self.__class__.__name__ + ":\n"
        for e in dir(self):
            if not e.startswith("_"):
                string += "{}: {}\n".format(e, self.__getattribute__(e))
        return string


class reg0x0E:
    def __init__(self, value) -> None:
        self.completeReg = value
        self._splitReg()

    def _splitReg(self):
        self.crc = self.completeReg & 0xFF000080
        self.delaycnt_sel = self.completeReg >> 20 & 0x1
        self.undervreg = self.completeReg >> 14 & 0x3F
        self.overvreg = self.completeReg >> 8 & 0x3F
        self.vadc_rate_set = self.completeReg >> 6 & 0x1
        self.vevent_cycs = self.completeReg & 0x3F

    def _setReg(self):
        self.completeReg = self.crc | self.delaycnt_sel << 20 | self.undervreg << 14 | \
            self.overvreg << 8 | self.vadc_rate_set << 6 | self.vevent_cycs

    def __getitem__(self, regNo):
        return getattr(self, regNo)

    def __setitem__(self, regNo: str, __value) -> None:
        try:
            self.__getattribute__(regNo)
        except AttributeError:
            print("Attribute", regNo, "not found")
            return

        setattr(self, regNo, __value)
        if(regNo == "completeReg"):
            self._splitReg()
        else:
            self._setReg()

    def __str__(self) -> str:
        string = self.__class__.__name__ + ":\n"
        for e in dir(self):
            if not e.startswith("_"):
                string += "{}: {}\n".format(e, self.__getattribute__(e))
        return string
