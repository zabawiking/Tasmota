#ifdef USE_EASUNINVERTER

#define XSNS_101 101
#define XSNS_101_COMMAND_PERIOD_SECONDS 10;

int sns_easun_inverter_seconds = 0;

#define EASUN_BUFFER_SIZE       256

//QPIGS => rec 108
int easunStatusCommandIndex = 0;
const char* easunStatusCommands[] PROGMEM = { "QPIGS", "QMOD" };
const char *commandSeparator PROGMEM = const_cast<char *>(" ");
const int easunStatusCommandSize = sizeof(easunStatusCommands) / sizeof(char*);

#include <TasmotaSerial.h>
TasmotaSerial *EasunSerial = nullptr;

struct EASUN {
    bool active;
    char *buffer = nullptr;                 // Serial receive buffer
    int byteCounter = 0;                   // Index in serial receive buffer
    uint8_t cmdStatus = 0;                 // 0 - awaiting command, 1 - command send - waiting for rec, 2 - receiving, 3 - received ok
    char* currentResponse;

    //QMOD
    char* CurrentMode;
    //QPIGS
    float GridVoltage;
    float GridFrequency;
    float ACOutputVoltage;
    float ACOutputFrequency;
    float ACOutputActivePower;
    float ACOutputLoadPercent;
    float BusVoltage;
    float BatteryVoltage;
    float BatteryChargingCurrent;
    float BatteryCapacityPercent;
    float InverterTemperature;
    float PVInputCurrent;
    float PVInputVoltage;
    float BatteryDischargeCurrent;
    float PVChargingPower;
    char* EasunDeviceStatus1;
    char* EasunDeviceStatus2;
} Easun;

void EasunInit(void) {
    Easun.active = false;
    Easun.cmdStatus = 0;
    easunStatusCommandIndex = 0;

    Easun.CurrentMode = (char*) "?";
    Easun.EasunDeviceStatus1 = (char*) "";
    Easun.EasunDeviceStatus2 = (char*) "";
    Easun.GridVoltage = 0.0;
    Easun.GridFrequency = 0.0;
    Easun.ACOutputVoltage = 0.0;
    Easun.ACOutputFrequency = 0.0;
    Easun.ACOutputActivePower = 0.0;
    Easun.ACOutputLoadPercent = 0.0;
    Easun.BusVoltage = 0.0;
    Easun.BatteryVoltage = 0.0;
    Easun.BatteryChargingCurrent = 0.0;
    Easun.BatteryCapacityPercent = 0.0;
    Easun.InverterTemperature = 0.0;
    Easun.PVInputCurrent = 0.0;
    Easun.PVInputVoltage = 0.0;
    Easun.BatteryDischargeCurrent = 0.0;
    Easun.PVChargingPower = 0.0;

    if (EasunSerial != nullptr) {
        AddLog(LOG_LEVEL_INFO, PSTR("[EASUN]: Closing serial port"));
        EasunSerial->end();
    }

    if (Easun.buffer != nullptr) {
        free(Easun.buffer);
    }

    int baudrate = 2400;
    Easun.buffer = (char*)(malloc(EASUN_BUFFER_SIZE));
    if (Easun.buffer != nullptr) {
        EasunSerial = new TasmotaSerial(Pin(GPIO_EASUN_RX), Pin(GPIO_EASUN_TX), 2);
        if (EasunSerial->begin(baudrate)) {
            if (EasunSerial->hardwareSerial()) {
                ClaimSerial();
            }

            AddLog(LOG_LEVEL_INFO, PSTR("[EASUN]: Opened serial port"));
            sns_easun_inverter_seconds = XSNS_101_COMMAND_PERIOD_SECONDS;
            Easun.active = true;
            return;
        }
        free(Easun.buffer);
    }
    AddLog(LOG_LEVEL_INFO, PSTR("[EASUN]: Error while opening serial port"));
    Easun.active = false;
}

void EasunSerialInput(void)
{
  while (EasunSerial->available()) {
    yield();
    uint8_t dataIn = EasunSerial->read();
    if (Easun.cmdStatus == 1 && dataIn == 40) {// '(' start
        Easun.cmdStatus = 2;
        Easun.byteCounter = 0;
    }
    else if (Easun.cmdStatus == 2) {
        if (Easun.byteCounter >= EASUN_BUFFER_SIZE - 1) {
            Easun.cmdStatus = 0;
            Easun.byteCounter = 0;
        }
        else if (dataIn == 13) {// '<CR>' end
            const char *command = easunStatusCommands[easunStatusCommandIndex];
            if (Easun.byteCounter > 2)
                Easun.currentResponse = strndup(Easun.buffer, Easun.byteCounter - 2); // strip CRC

            AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Command: %s, Response: %s"), command, Easun.currentResponse);
            Easun.cmdStatus = 3;
            //parse
            ParseReceivedData();

            //send another commmand
            Easun.cmdStatus = 0;
            easunStatusCommandIndex++;
            if (easunStatusCommandIndex >= easunStatusCommandSize)
                easunStatusCommandIndex = 0;
        }
        else {
            Easun.buffer[Easun.byteCounter++] = dataIn;
        }
    }
  }
}

void EasunSendStatusCommand()
{
    const char *command = easunStatusCommands[easunStatusCommandIndex];
    int commandSize = strlen(command);
    ushort crc = CRC16XMODEM((char*) command, strlen(command));
    AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Sending command: %s"), command);
    for (;commandSize > 0; commandSize--) {
        EasunSerial->write(*command++);
    }
    EasunSerial->write(crc >> 8);
    EasunSerial->write(crc & 0xFF);
    EasunSerial->write(0x0D); // <CR>

    Easun.cmdStatus = 1;
}

ushort CRC16XMODEM(char *buf, int len)
{
    ushort crc = 0;
    for (; len > 0; len--)
    {
        crc ^= (ushort)(*buf++ << 8);
        for (int i = 0; i < 8; i++)
        {
            if ((crc & 0x8000) != 0)
                crc = (ushort)((crc << 1) ^ 0x1021);
            else
                crc = (ushort)(crc << 1);
        }
    }

    // lo-hi
    return crc;

    // ..or
    // hi-lo reordered
    //return (ushort)((crc >> 8) | (crc << 8));
}

void ParseReceivedData()
{
    switch (easunStatusCommandIndex) {
        case 0: //QPIGS
        {
            char *token = strtok(Easun.currentResponse, commandSeparator);
            int index = 0;
            while (token != nullptr)
            {
                if (index == 0) Easun.GridVoltage = atof(token);
                if (index == 1) Easun.GridFrequency = atof(token);
                if (index == 2) Easun.ACOutputVoltage = atof(token);
                if (index == 3) Easun.ACOutputFrequency = atof(token);
                if (index == 5) Easun.ACOutputActivePower = atof(token);
                if (index == 6) Easun.ACOutputLoadPercent = atof(token);
                if (index == 7) Easun.BusVoltage = atof(token);
                if (index == 8) Easun.BatteryVoltage = atof(token);
                if (index == 9) Easun.BatteryChargingCurrent = atof(token);
                if (index == 10) Easun.BatteryCapacityPercent = atof(token);
                if (index == 11) Easun.InverterTemperature = atof(token);
                if (index == 12) Easun.PVInputCurrent = atof(token);
                if (index == 13) Easun.PVInputVoltage = atof(token);
                if (index == 15) Easun.BatteryDischargeCurrent = atof(token);
                if (index == 19) Easun.PVChargingPower = atof(token);

                token = strtok(nullptr, commandSeparator);
                index++;
            }
            break;
        }
        case 1: //QMOD
        {
            Easun.CurrentMode = strndup(Easun.currentResponse, 1);
            break;
        }
    }
}

void EasunSensorsShow(bool json)
{
    if (json) {
        ResponseAppend_P(PSTR(",\"Easun\":{"));
        ResponseAppend_P(PSTR(",\"CurrentMode\": \"%s\""), Easun.CurrentMode);
        ResponseAppend_P(PSTR(",\"GridVoltage\": %.1f"), Easun.GridVoltage);
        ResponseAppend_P(PSTR(",\"GridFrequency\": %.1f"), Easun.GridFrequency);
        ResponseAppend_P(PSTR(",\"ACOutputVoltage\": %.1f"), Easun.ACOutputVoltage);
        ResponseAppend_P(PSTR(",\"ACOutputFrequency\": %.1f"), Easun.ACOutputFrequency);
        ResponseAppend_P(PSTR(",\"ACOutputActivePower\": %.0f"), Easun.ACOutputActivePower);
        ResponseAppend_P(PSTR(",\"ACOutputLoadPercent\": %.0f"), Easun.ACOutputLoadPercent);
        ResponseAppend_P(PSTR(",\"BusVoltage\": %.0f"), Easun.BusVoltage);
        ResponseAppend_P(PSTR(",\"BatteryVoltage\": %.2f"), Easun.BatteryVoltage);
        ResponseAppend_P(PSTR(",\"BatteryChargingCurrent\": %.0f"), Easun.BatteryChargingCurrent);
        ResponseAppend_P(PSTR(",\"BatteryCapacityPercent\": %.0f"), Easun.BatteryCapacityPercent);
        ResponseAppend_P(PSTR(",\"InverterTemperature\": %.0f"), Easun.InverterTemperature);
        ResponseAppend_P(PSTR(",\"PVInputCurrent\": %.1f"), Easun.PVInputCurrent);
        ResponseAppend_P(PSTR(",\"PVInputVoltage\": %.1f"), Easun.PVInputVoltage);
        ResponseAppend_P(PSTR(",\"BatteryDischargeCurrent\": %.1f"), Easun.BatteryDischargeCurrent);
        ResponseAppend_P(PSTR(",\"PVChargingPower\": %.0f"), Easun.PVChargingPower);
        ResponseJsonEnd();
    }
    #ifdef USE_WEBSERVER
    else {
        WSContentSend_PD("EASUN");
        WSContentSend_P(PSTR("{s}CurrentMode{m}%s{e}"), Easun.CurrentMode);
        WSContentSend_P(PSTR("{s}GridVoltage{m}%.1f{e}"), Easun.GridVoltage);
        WSContentSend_P(PSTR("{s}GridFrequency{m}%.1f{e}"), Easun.GridFrequency);
        WSContentSend_P(PSTR("{s}ACOutputVoltage{m}%.1f{e}"), Easun.ACOutputVoltage);
        WSContentSend_P(PSTR("{s}ACOutputFrequency{m}%.1f{e}"), Easun.ACOutputFrequency);
        WSContentSend_P(PSTR("{s}ACOutputActivePower{m}%.0f{e}"), Easun.ACOutputActivePower);
        WSContentSend_P(PSTR("{s}ACOutputLoadPercent{m}%.0f{e}"), Easun.ACOutputLoadPercent);
        WSContentSend_P(PSTR("{s}BusVoltage{m}%.0f{e}"), Easun.BusVoltage);
        WSContentSend_P(PSTR("{s}BatteryVoltage{m}%.1f{e}"), Easun.BatteryVoltage);
        WSContentSend_P(PSTR("{s}BatteryChargingCurrent{m}%.0f{e}"), Easun.BatteryChargingCurrent);
        WSContentSend_P(PSTR("{s}BatteryCapacityPercent{m}%.0f{e}"), Easun.BatteryCapacityPercent);
        WSContentSend_P(PSTR("{s}InverterTemperature{m}%.0f{e}"), Easun.InverterTemperature);
        WSContentSend_P(PSTR("{s}PVInputCurrent{m}%.1f{e}"), Easun.PVInputCurrent);
        WSContentSend_P(PSTR("{s}PVInputVoltage{m}%.1f{e}"), Easun.PVInputVoltage);
        WSContentSend_P(PSTR("{s}BatteryDischargeCurrent{m}%.1f{e}"), Easun.BatteryDischargeCurrent);
        WSContentSend_P(PSTR("{s}PVChargingPower{m}%.0f{e}"), Easun.PVChargingPower);
    }
    #endif
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns101(uint8_t function)
{
    bool result = false;
    if (FUNC_INIT == function)
    {
    }

    if (FUNC_PRE_INIT == function)
    {
        EasunInit();
    }

    if (!Easun.active)
        return result;

    switch (function)
    {
    case FUNC_LOOP:
        if (EasunSerial) { EasunSerialInput(); }
        break;
    case FUNC_EVERY_100_MSECOND:
        break;
    case FUNC_EVERY_SECOND:
        sns_easun_inverter_seconds--;
        if (EasunSerial && sns_easun_inverter_seconds <= 0) {
            if (Easun.cmdStatus != 0) {
                AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: No response received, resetting COM PORT"));
                EasunInit();
            }
            else {
                EasunSendStatusCommand();
            }
            sns_easun_inverter_seconds = XSNS_101_COMMAND_PERIOD_SECONDS;
        }

        if (Easun.cmdStatus == 0 && easunStatusCommandIndex > 0) {
            EasunSendStatusCommand();
        }
        break;
    case FUNC_COMMAND:
        //result = DecodeCommand(kOpenThermCommands, OpenThermCommands);
        break;
    case FUNC_JSON_APPEND:
        EasunSensorsShow(1);
        break;
    case FUNC_SERIAL:
        break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        EasunSensorsShow(0);
        break;
    case FUNC_WEB_GET_ARG:
        //sns_opentherm_web_get_arg();
        break;
#endif // USE_WEBSERVER
    }

    return result;
}

#endif // USE_EASUNINVERTER