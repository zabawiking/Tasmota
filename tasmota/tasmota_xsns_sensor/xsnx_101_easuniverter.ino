#ifdef USE_EASUNINVERTER

#define XSNS_101 101
#define XSNS_101_COMMAND_PERIOD_SECONDS 10;

int easunLoopSeconds = 0;

#define EASUN_BUFFER_SIZE       256

//QPIGS => rec 108
//int easunStatusCommandIndex = 0;
const char* easunStatusCommands[] PROGMEM = { "QPIGS", "QMOD", "QPIRI", "QPIWS" };
const char *easunCommandSeparator PROGMEM = const_cast<char *>(" ");
const int easunStatusCommandSize = sizeof(easunStatusCommands) / sizeof(char*);

char* easunCommandBuffer[10];
int easunCommandIndex = -1;

#include <TasmotaSerial.h>
TasmotaSerial *EasunSerial = nullptr;

//commands
#define D_PRFX_EASUN "ea_"
const char kEasunCommand[] PROGMEM = D_PRFX_EASUN "|send";
void (*const EasunCommand[])(void) PROGMEM = {
    &EasunProcessCommand
};

struct EASUN {
    bool active;
    char *buffer = nullptr;                 // Serial receive buffer
    int byteCounter = 0;                   // Index in serial receive buffer
    uint8_t cmdStatus = 0;                 // 0 - awaiting command, 1 - command send - waiting for rec, 2 - receiving, 3 - received ok

    //QMOD
    char* CurrentMode;
    //QPIGS
    float GridVoltage;
    float GridFrequency;
    float ACOutputVoltage;
    float ACOutputFrequency;
    float ACOutputActivePower;
    float ACOutputLoadPercent;
    bool ACOutputOn;
    float BusVoltage;
    float BatteryVoltage;
    float BatteryChargingCurrent;
    float BatteryCapacityPercent;
    float InverterTemperature;
    float PVInputCurrent;
    float PVInputVoltage;
    float BatteryDischargeCurrent;
    float PVChargingPower;
    char* DeviceStatus1;
    char* DeviceStatus2;
    bool BatteryCharging;
    bool BatteryChargingSolar;
    bool BatteryChargingAC;
    bool BatteryChargingFloating;
    bool BatteryVoltageSteady;
} Easun;

void EasunInit(void) {
    Easun.active = false;
    Easun.cmdStatus = 0;
    //easunStatusCommandIndex = 0;
    easunCommandIndex = -1;
    easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;

    Easun.CurrentMode = strdup("?");
    Easun.DeviceStatus1 = strdup("00000000");
    Easun.DeviceStatus2 = strdup("000");
    Easun.GridVoltage = 0.0;
    Easun.GridFrequency = 0.0;
    Easun.ACOutputVoltage = 0.0;
    Easun.ACOutputFrequency = 0.0;
    Easun.ACOutputActivePower = 0.0;
    Easun.ACOutputLoadPercent = 0.0;
    Easun.ACOutputOn = false;
    Easun.BusVoltage = 0.0;
    Easun.BatteryVoltage = 0.0;
    Easun.BatteryChargingCurrent = 0.0;
    Easun.BatteryCapacityPercent = 0.0;
    Easun.InverterTemperature = 0.0;
    Easun.PVInputCurrent = 0.0;
    Easun.PVInputVoltage = 0.0;
    Easun.BatteryDischargeCurrent = 0.0;
    Easun.PVChargingPower = 0.0;
    Easun.BatteryCharging = false;
    Easun.BatteryChargingSolar = false;
    Easun.BatteryChargingAC = false;
    Easun.BatteryChargingFloating = false;
    Easun.BatteryVoltageSteady = false;

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
            easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;
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
                if (Easun.byteCounter > 2) {
                    Easun.buffer[Easun.byteCounter - 1] = 0x00;
                    Easun.buffer[Easun.byteCounter - 2] = 0x00;
                }

                if (easunCommandIndex > -1) {
                    Easun.cmdStatus = 3;

                    char *command = easunCommandBuffer[easunCommandIndex];
                    AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Command: %s, Response: %s, commandIndex: %d"), command, Easun.buffer, easunCommandIndex);

                    // ResponseClear();
                    //Response_P(PSTR("{\"Easun\":{\"Command\":\"%s\", \"Response\":\"%s\"}}"), command, Easun.buffer);
                    //MqttPublishPrefixTopicRulesProcess_P(RESULT_OR_STAT, PSTR("Easun"));

                    //parse
                    EasunParseReceivedData(command);

                    free(easunCommandBuffer[easunCommandIndex]);
                    easunCommandIndex--;
                } else {
                    AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Response: %s"), Easun.buffer);
                }

                Easun.cmdStatus = 0;
            }
            else {
                Easun.buffer[Easun.byteCounter++] = dataIn;
            }
        }
    }
}

void EasunEnqueueCommand(char *command)
{
    if (easunCommandIndex < 10) {
        easunCommandIndex++;
        int commandSize = strlen(command);
        easunCommandBuffer[easunCommandIndex] = new char[commandSize + 1];
        strcpy(easunCommandBuffer[easunCommandIndex], command);
        AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Enqueue command: %s, commandIndex: %d"), command, easunCommandIndex);
    }
}

void EasunSendCommand()
{
    const char *command = easunCommandBuffer[easunCommandIndex];
    int commandSize = strlen(command);
    ushort crc = EasunCRC16XMODEM((char*) command, strlen(command));
    AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Sending command: %s, commandIndex: %d"), command, easunCommandIndex);
    for (;commandSize > 0; commandSize--) {
        EasunSerial->write(*command++);
    }
    EasunSerial->write(crc >> 8);
    EasunSerial->write(crc & 0xFF);
    EasunSerial->write(0x0D); // <CR>

    Easun.cmdStatus = 1;
}

ushort EasunCRC16XMODEM(char *buf, int len)
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
    return crc;
}

void EasunParseReceivedData(char *command)
{
    if (Easun.buffer == nullptr)
        return;

    if (strcmp("NAK", Easun.buffer) == 0)
        return;

    if (strcmp("QPIGS", command) == 0) {
        if (Easun.byteCounter < 106)
            return;

        char *token = strtok(Easun.buffer, easunCommandSeparator);
        int index = 0;
        while (token != nullptr)
        {
            if (index == 0) Easun.GridVoltage = CharToFloat(token);
            if (index == 1) Easun.GridFrequency = CharToFloat(token);
            if (index == 2) Easun.ACOutputVoltage = CharToFloat(token);
            if (index == 3) Easun.ACOutputFrequency = CharToFloat(token);
            if (index == 5) Easun.ACOutputActivePower = CharToFloat(token);
            if (index == 6) Easun.ACOutputLoadPercent = CharToFloat(token);
            if (index == 7) Easun.BusVoltage = CharToFloat(token);
            if (index == 8) Easun.BatteryVoltage = CharToFloat(token);
            if (index == 9) Easun.BatteryChargingCurrent = CharToFloat(token);
            if (index == 10) Easun.BatteryCapacityPercent = CharToFloat(token);
            if (index == 11) Easun.InverterTemperature = CharToFloat(token);
            if (index == 12) Easun.PVInputCurrent = CharToFloat(token);
            if (index == 13) Easun.PVInputVoltage = CharToFloat(token);
            if (index == 15) Easun.BatteryDischargeCurrent = CharToFloat(token);
            if (index == 16) {
                strncpy(Easun.DeviceStatus1, token, 8);
                Easun.BatteryCharging = Easun.DeviceStatus1[5] == '1';
                Easun.BatteryChargingSolar = Easun.DeviceStatus1[6] == '1';
                Easun.BatteryChargingAC = Easun.DeviceStatus1[7] == '1';
                Easun.BatteryVoltageSteady = Easun.DeviceStatus1[4] == '1';
            }
            if (index == 20) {
                strncpy(Easun.DeviceStatus2, token, 3);
                Easun.BatteryChargingFloating = Easun.DeviceStatus2[0] == '1';
                Easun.ACOutputOn = Easun.DeviceStatus2[1] == '1';
            }
            if (index == 19) Easun.PVChargingPower = CharToFloat(token);

            token = strtok(nullptr, easunCommandSeparator);
            index++;
        }
        AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: QPIGS parsed OK"));
    }
    else if (strcmp("QMOD", command) == 0)
    {
        strncpy(Easun.CurrentMode, Easun.buffer, 1);
        AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: QMOD parsed OK"));
    }
}

void EasunSensorsShow(bool json)
{
    if (json) {
        ResponseAppend_P(PSTR(",\"Easun\":{"));
        ResponseAppend_P(PSTR("\"CurrentMode\": \"%s\""), Easun.CurrentMode);
        ResponseAppend_P(PSTR(",\"GridVoltage\": %.1f"), Easun.GridVoltage);
        ResponseAppend_P(PSTR(",\"GridFrequency\": %.1f"), Easun.GridFrequency);
        ResponseAppend_P(PSTR(",\"ACOutputVoltage\": %.1f"), Easun.ACOutputVoltage);
        ResponseAppend_P(PSTR(",\"ACOutputFrequency\": %.1f"), Easun.ACOutputFrequency);
        ResponseAppend_P(PSTR(",\"ACOutputActivePower\": %.0f"), Easun.ACOutputActivePower);
        ResponseAppend_P(PSTR(",\"ACOutputLoadPercent\": %.0f"), Easun.ACOutputLoadPercent);
        ResponseAppend_P(PSTR(",\"BusVoltage\": %.0f"), Easun.BusVoltage);
        ResponseAppend_P(PSTR(",\"BatteryVoltage\": %.2f"), Easun.BatteryVoltage);
        ResponseAppend_P(PSTR(",\"BatteryChargingCurrent\": %.0f"), Easun.BatteryChargingCurrent);
        ResponseAppend_P(PSTR(",\"BatteryDischargeCurrent\": %.0f"), Easun.BatteryDischargeCurrent);
        ResponseAppend_P(PSTR(",\"BatteryCapacityPercent\": %.0f"), Easun.BatteryCapacityPercent);
        ResponseAppend_P(PSTR(",\"InverterTemperature\": %.0f"), Easun.InverterTemperature);
        ResponseAppend_P(PSTR(",\"PVInputCurrent\": %.1f"), Easun.PVInputCurrent);
        ResponseAppend_P(PSTR(",\"PVInputVoltage\": %.1f"), Easun.PVInputVoltage);
        ResponseAppend_P(PSTR(",\"PVChargingPower\": %.0f"), Easun.PVChargingPower);
        ResponseAppend_P(PSTR(",\"DeviceStatus1\": \"%s\""), Easun.DeviceStatus1);
        ResponseAppend_P(PSTR(",\"DeviceStatus2\": \"%s\""), Easun.DeviceStatus2);
        ResponseAppend_P(PSTR(",\"ACOutputOn\": %d"), Easun.ACOutputOn);
        ResponseAppend_P(PSTR(",\"BatteryCharging\": %d"), Easun.BatteryCharging);
        ResponseAppend_P(PSTR(",\"BatteryChargingSolar\": %d"), Easun.BatteryChargingSolar);
        ResponseAppend_P(PSTR(",\"BatteryChargingAC\": %d"), Easun.BatteryChargingAC);
        ResponseAppend_P(PSTR(",\"BatteryChargingFloating\": %d"), Easun.BatteryChargingFloating);
        ResponseAppend_P(PSTR(",\"BatteryVoltageSteady\": %d"), Easun.BatteryVoltageSteady);

        ResponseJsonEnd();
    }
    #ifdef USE_WEBSERVER
    else {
        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}CurrentMode{m}%s{e}"), Easun.CurrentMode);
        WSContentSend_P(PSTR("{s}PVChargingPower{m}%.0f W{e}"), Easun.PVChargingPower);
        WSContentSend_P(PSTR("{s}ACOutputActivePower{m}%.0f W{e}"), Easun.ACOutputActivePower);
        WSContentSend_P(PSTR("{s}ACOutputLoadPercent{m}%.0f %%{e}"), Easun.ACOutputLoadPercent);

        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}GridVoltage{m}%.1f V{e}"), Easun.GridVoltage);
        WSContentSend_P(PSTR("{s}GridFrequency{m}%.1f Hz{e}"), Easun.GridFrequency);
        WSContentSend_P(PSTR("{s}ACOutputVoltage{m}%.1f V{e}"), Easun.ACOutputVoltage);
        WSContentSend_P(PSTR("{s}ACOutputFrequency{m}%.1f Hz{e}"), Easun.ACOutputFrequency);
        WSContentSend_P(PSTR("{s}ACOutputOn{m}%d{e}"), Easun.ACOutputOn);

        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}BatteryVoltage{m}%.1f V{e}"), Easun.BatteryVoltage);
        WSContentSend_P(PSTR("{s}BatteryChargingCurrent{m}%.0f A{e}"), Easun.BatteryChargingCurrent);
        WSContentSend_P(PSTR("{s}BatteryDischargeCurrent{m}%.0f A{e}"), Easun.BatteryDischargeCurrent);
        WSContentSend_P(PSTR("{s}BatteryCapacityPercent{m}%.0f %%{e}"), Easun.BatteryCapacityPercent);
        WSContentSend_P(PSTR("{s}BatteryCharging{m}%d{e}"), Easun.BatteryCharging);
        WSContentSend_P(PSTR("{s}BatteryChargingSolar{m}%d{e}"), Easun.BatteryChargingSolar);
        WSContentSend_P(PSTR("{s}BatteryChargingAC{m}%d{e}"), Easun.BatteryChargingAC);
        WSContentSend_P(PSTR("{s}BatteryChargingFloating{m}%d{e}"), Easun.BatteryChargingFloating);
        WSContentSend_P(PSTR("{s}BatteryVoltageSteady{m}%d{e}"), Easun.BatteryVoltageSteady);

        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}PVInputCurrent{m}%.1f A{e}"), Easun.PVInputCurrent);
        WSContentSend_P(PSTR("{s}PVInputVoltage{m}%.1f V{e}"), Easun.PVInputVoltage);

        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}BusVoltage{m}%.0f V{e}"), Easun.BusVoltage);
        WSContentSend_P(PSTR("{s}InverterTemperature{m}%.0f{e}"), Easun.InverterTemperature);
        WSContentSend_P(PSTR("{s}DeviceStatus1{m}%s{e}"), Easun.DeviceStatus1);
        WSContentSend_P(PSTR("{s}DeviceStatus2{m}%s{e}"), Easun.DeviceStatus2);
        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
    }
    #endif
}

void EasunProcessCommand(void) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: Command: %s, Data: %s"), XdrvMailbox.command, XdrvMailbox.data);
    if (strlen(XdrvMailbox.data) > 0) {
        EasunEnqueueCommand(XdrvMailbox.data);
        //Response_P(PSTR("{\"Easun\":{\"Command\":\"%s\"}}"), XdrvMailbox.data);
        ResponseCmndDone();
    }
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
    case FUNC_EVERY_SECOND:
        easunLoopSeconds--;
        if (EasunSerial && easunLoopSeconds <= 0) {
            if (Easun.cmdStatus != 0) {
                AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: No response received, resetting COM PORT"));
                EasunInit();
            }
            else {
                for (int i = easunStatusCommandSize - 1; i >= 0; i--) {
                    const char *command = easunStatusCommands[i];
                    EasunEnqueueCommand((char*) command);
                }
            }
            easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;
        }

        if (EasunSerial && Easun.cmdStatus == 0 && easunCommandIndex > -1) {
            EasunSendCommand();
        }
        break;
    case FUNC_COMMAND:
        result = DecodeCommand(kEasunCommand, EasunCommand);
        break;
    case FUNC_JSON_APPEND:
        EasunSensorsShow(1);
        break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
        EasunSensorsShow(0);
        break;
    case FUNC_WEB_GET_ARG:
        break;
#endif // USE_WEBSERVER
    }

    return result;
}

#endif // USE_EASUNINVERTER