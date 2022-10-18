#include "PA_DataLogger.h"


/*!
    @brief The DataLogger uses SPI to communicate with an SD Card
    @param ACS71020* pointer to ACS71020 Instance
    @param usb_serial2_class* Instance to serialBus to print Debug messages to
    @param cs unsigned 16-bit integer for chip-select
*/
DataLogger::DataLogger(ACS71020* chip, usb_serial2_class* serialBus, uint16_t cs) :
    _acsChip(chip),
    _serialBus(serialBus) {
    _cs = cs;
}

/*!
    @brief Tries to connect the SD Card
*/
boolean DataLogger::init() {
    if (_card.init(SPI_FULL_SPEED, _cs)) {
        _serialBus->println("Initing");
        if (!SD.sdfs.exists("energy/energyconfig.txt")) {
            _serialBus->printf("Found no existing Energy-Log. Creating File... \n");
            _resetEnergy();
        } else {
            _initEnergy();
        }
        if (!(_volume.init(_card))) {
            _serialBus->printf("No FAT16/FAT32 Partition found!\n");
            return false;
        }
        return true;
    }
    return false;
}

void DataLogger::_saveEnergy() {
    if (energy.logHourly)
        sprintf(fileNameBuffer, "%s/log_%u.txt\0", energy_prefix, currDay);
    FsFile f = SD.sdfs.open(fileNameBuffer, O_CREAT | O_WRITE);
    f.printf("Energy\n");
    for (uint8_t i = 0; i < 23; i++) {
        f.printf("%f\n", energy.perHour[i]);
    }
    f.close();
    sprintf(fileNameBuffer, "%s/log_%u.txt\0", energy_prefix, currDay);
    f.printf("M ");
    for (uint8_t i = 0; i < 31; i++) {
        f.printf("%f", energy.perMonth[i]);
    }
    f.printf("\n");
    f.printf("Y ");
    for (uint8_t i = 0; i < 12; i++) {
        f.printf("%f ", energy.perYear[i]);
    }
    f.printf("\n");
    f.close();
    _serialBus->printf("Done with saving the energy data\n");
}

void DataLogger::_initEnergy() {
    uint16_t temptime = 0;
    uint16_t tempdate = 0;
    uint16_t latestFile = 0;
    FsFile root = SD.sdfs.open("energy/");
    while (1) {
        FsFile f = root.openNextFile();
        if (!f) break;
        f.getModifyDateTime(&tempdate, &temptime);
        if (temptime > latestFile) {
            latestFile = temptime;
        }
        f.close();
    }
    root.close();
    FsFile f = SD.sdfs.open("energy/.txt", O_CREAT | O_WRITE);
    char* endptr;
    char buffer[BUFSIZE];
    uint8_t energytype = 0;
    uint16_t index = 0;
    while (f.available() > 0) {
        f.readBytesUntil('\n', buffer, BUFSIZE);
        _serialBus->printf("Read: %s\n", buffer);
        /*switch (c) {
        case 'H':
            energytype = 0; break;
        case 'M':
            energytype = 1; break;
        case 'Y':
            energytype = 2; break;
        case '\n':
            if (energytype == 0) {
                _serialBus->printf("Per Hour: %s\n", buffer);
            } else if (energytype == 1) {
                _serialBus->printf("Per Month: %s\n", buffer);
            } else if (energytype == 2) {
                _serialBus->printf("Per Year: %s\n", buffer);
            }
            for (uint16_t i = 0; i < BUFSIZE; i++) {
                buffer[i] = 0;
            }
            index = 0;

        default:
            buffer[index++] = c; break;
        }*/
    }
    f.close();
}

void DataLogger::_resetEnergy() {
    memset(&energy, 0, sizeof(energy));
}
/*!
    @brief Formats all values except FFT-harmonics in a nice string
*/
void DataLogger::_formatLogValues() {
    char formatString[255];
    char tempBuffer[20]; //Temporary buffer for measurement data
    if (_config->flags._logVoltageAndCurrent) {
        sprintf(tempBuffer, "%f;%f;", _acsChip->readVRMS(1), _acsChip->readIRMS(1));
        strcat(formatString, tempBuffer);
    }
    if (_config->flags._logEnergy) {
        _logEnergy();
    }
    _serialBus->printf("Current string: %s", formatString);
}

/*!
    @brief Formats all FFT-harmonics in a nice String
*/
void DataLogger::_formatFFTLog() {

}

/*!
    @brief Reads Power Measurements and stores them in a structure
*/
void DataLogger::_logEnergy() {
    power.pInstant = _acsChip->readPINST();
    power.activeAvgSec = _acsChip->readPACTIVE(1);
    power.imagPower = _acsChip->readPIMAG();
    power.apparentPower = _acsChip->readPAPP();
    power.powerFactor = _acsChip->readPOWERFACTOR();

    energy.perHour[hour() % 24] += power.activeAvgSec / 3600.0F; //One hour in Wattsecond
    energy.perMonth[month() % 31] += power.activeAvgSec / (1000.0F * 86400.0F); //One day in Wattsecond

    //Write Log Info
    sprintf(fileNameBuffer, "%s/%s.txt", power_prefix, currentDateString);
    FsFile f = SD.sdfs.open(fileNameBuffer, O_CREAT | O_APPEND); //Create or/and append
    if (f.size() == 0) f.print("InstantPowerWatt;ActivePowerWatt;ImagPowerVAR;ApparPowerVA;PowerFactor\n");
    f.printf("%.3f;%.3f;%.3f;%.3f;%.3f\n", power.pInstant,
    power.activeAvgSec,
    power.imagPower,
    power.apparentPower,
    power.powerFactor);
    f.close();

    sprintf(fileNameBuffer, "%s/%s.txt", energy_prefix, currentDateString);
    f = SD.sdfs.open(fileNameBuffer, O_CREAT | O_APPEND);
    if (f.size() == 0) f.print("Watt_per_hour;kWh_per_day\n");


}

/*!
    @brief Writes a log entry to a logfile
*/
boolean DataLogger::writeLog(File logFile, String log) {
    return false;
}

/*!
    @brief Logs the values
*/

void DataLogger::LogAll() {
    _serialBus->printf("Logging Values\n");
    _logEnergy();
    _logFFT();
    _logRMS();
    //_formatLogValues();
}

/*!
    @brief Prints the current directory contents
*/
void DataLogger::printDirectory(File dir, int numTabs) {
    SD.sdfs.ls(_serialBus, LS_R);
}


/*!
    @brief gets the FileCount in the current directory
*/
uint32_t DataLogger::getFileCount() {
    return 0;
}

/*!
    @brief Toggles the logging of raw voltage and current levels
*/

void DataLogger::Log_RAW_VandC(boolean state) {
    _config->flags._logVoltageAndCurrent = state;
}

/*!
    @brief Toggles the logging of RMS voltage and current levels
*/

void DataLogger::Log_RMS_VandC(boolean state) {
    _config->flags._logRMSVoltageAndCurrent = state;
}
/*!
    @brief Toggles the logging of the power factor
*/

void DataLogger::Log_PowerFactor(boolean state) {
    _config->flags._logPowerFactor = state;
}
/*!
    @brief Toggles the logging of active power values avg over 1 second
*/

void DataLogger::Log_Power(boolean state) {
    _config->flags._logPower = state;
}
/*!
    @brief Toggles the logging of power-calculations
*/

void DataLogger::Log_Energy(boolean state) {
    _config->flags._logEnergy = state;
}
/*!
    @brief Toggles the logging of voltage and current events
*/

void DataLogger::Log_Events(boolean state) {
    _config->flags._logEvents = state;
}
/*!
    @brief Toggles the logging of Total Harmonic Distortion
*/

void DataLogger::Log_THD(boolean state) {
    _config->flags._logTHD = state;
}

/*!
    @brief Toggles the logging of FFT harmonics
*/
void DataLogger::Log_FFT(boolean state) {
    _config->flags._logFFT = state;
}

/*!
    @brief Updates the current time
*/
void DataLogger::_updateLogfile() {
    uint32_t tempday = day();
    if (tempday != currDay) {
        _saveEnergy(); //Store daily energy log in file
        currDay = day();
        currMonth = month();
        currYear = year();
    }
}