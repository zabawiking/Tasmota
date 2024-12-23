#ifdef USE_EASUNINVERTER

#define XSNS_101 101
#define XSNS_101_COMMAND_PERIOD_SECONDS 10;

int easunLoopSeconds = 0;

#define EASUN_BUFFER_SIZE       256
#define EASUN_COMMMAND_SIZE     128

//QPIGS => rec 108
//int easunStatusCommandIndex = 0;
const char* easunStatusCommands[] PROGMEM = { "QPIGS", "QMOD", "QPIRI", "QPIWS" };
const int easunStatusCommandSize = sizeof(easunStatusCommands) / sizeof(char*);

char PROGMEM easunCommandBuffer[10][EASUN_COMMMAND_SIZE + 1];
int easunCommandIndex = -1;

#include <TasmotaSerial.h>
TasmotaSerial *EasunSerial = nullptr;

//commands
#define D_PRFX_EASUN "ea_"
const char kEasunCommand[] PROGMEM = D_PRFX_EASUN "|send";
void (*const EasunCommand[])(void) PROGMEM = {
    &EasunProcessCommand
};

PROGMEM struct EASUN {
    bool active;
    char buffer[EASUN_BUFFER_SIZE];        // Serial receive buffer
    int byteCounter = 0;                   // Index in serial receive buffer
    uint8_t cmdStatus = 0;                 // 0 - awaiting command, 1 - command send - waiting for rec, 2 - receiving, 3 - received ok

    //QMOD
    char CurrentMode[2];
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
    char DeviceStatus1[9];
    char DeviceStatus2[4];
    bool BatteryCharging;
    bool BatteryChargingSolar;
    bool BatteryChargingAC;
    bool BatteryChargingFloating;
    bool BatteryVoltageSteady;

    //QPIRI
    char SourcePriority[4];
    char ChargingSourcePriority[4];
} Easun;

void EasunInit(void) {

    if (!PinUsed(GPIO_EASUN_RX) || !PinUsed(GPIO_EASUN_TX))
        return;

    Easun.active = false;
    Easun.cmdStatus = 0;
    easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;

    strcpy(Easun.CurrentMode, "?");
    strcpy(Easun.DeviceStatus1, "00000000");
    strcpy(Easun.DeviceStatus2, "000");
    strcpy(Easun.SourcePriority, "???");
    strcpy(Easun.ChargingSourcePriority, "???");    
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

    easunCommandIndex = -1;

    if (EasunSerial != nullptr) {
        AddLog(LOG_LEVEL_INFO, PSTR("[EASUN]: Closing serial port"));
        EasunSerial->end();
    }

    if (EasunSerial == nullptr) {
        EasunSerial = new TasmotaSerial(Pin(GPIO_EASUN_RX), Pin(GPIO_EASUN_TX), 2);
    }

    int baudrate = 2400;
    if (Easun.buffer != nullptr) {
        if (EasunSerial->begin(baudrate)) {
            if (EasunSerial->hardwareSerial()) {
                ClaimSerial();
            }

            AddLog(LOG_LEVEL_INFO, PSTR("[EASUN]: Opened serial port"));
            easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;
            Easun.active = true;
            return;
        }
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

                    //parse
                    EasunParseReceivedData(command);

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
        strncpy(easunCommandBuffer[easunCommandIndex], command, EASUN_COMMMAND_SIZE);
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

    char crc_lo = crc & 0xFF;
    char crc_hi = crc >> 8;

    if (crc_lo == 0x28 || crc_lo == 0x0d || crc_lo == 0x0a)
      crc_lo++;
    if (crc_hi == 0x28 || crc_hi == 0x0d || crc_hi == 0x0a)
      crc_hi++;

    EasunSerial->write(crc_hi);
    EasunSerial->write(crc_lo);    
    EasunSerial->write(0x0D); // <CR>
    EasunSerial->flush();

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
    if (strcmp("NAK", Easun.buffer) == 0)
        return;

    if (strcmp("QPIGS", command) == 0) {
        if (Easun.byteCounter < 106)
            return;

        char *token = strtok(Easun.buffer, " ");
        int index = 0;
        while (token != nullptr)
        {
            switch(index) {
                case 0: Easun.GridVoltage = CharToFloat(token); break;
                case 1: Easun.GridFrequency = CharToFloat(token); break;
                case 2: Easun.ACOutputVoltage = CharToFloat(token); break;
                case 3: Easun.ACOutputFrequency = CharToFloat(token); break;
                case 5: Easun.ACOutputActivePower = CharToFloat(token); break;
                case 6: Easun.ACOutputLoadPercent = CharToFloat(token); break;
                case 7: Easun.BusVoltage = CharToFloat(token); break;
                case 8: Easun.BatteryVoltage = CharToFloat(token); break;
                case 9: Easun.BatteryChargingCurrent = CharToFloat(token); break;
                case 10: Easun.BatteryCapacityPercent = CharToFloat(token); break;
                case 11: Easun.InverterTemperature = CharToFloat(token); break;
                case 12: Easun.PVInputCurrent = CharToFloat(token); break;
                case 13: Easun.PVInputVoltage = CharToFloat(token); break;
                case 15: Easun.BatteryDischargeCurrent = CharToFloat(token); break;
                case 16:
                    strncpy(Easun.DeviceStatus1, token, 8);
                    Easun.BatteryCharging = Easun.DeviceStatus1[5] == '1';
                    Easun.BatteryChargingSolar = Easun.DeviceStatus1[6] == '1';
                    Easun.BatteryChargingAC = Easun.DeviceStatus1[7] == '1';
                    Easun.BatteryVoltageSteady = Easun.DeviceStatus1[4] == '1';
                break;
                case 20:
                    strncpy(Easun.DeviceStatus2, token, 3);
                    Easun.BatteryChargingFloating = Easun.DeviceStatus2[0] == '1';
                    Easun.ACOutputOn = Easun.DeviceStatus2[1] == '1';
                break;
                case 19: Easun.PVChargingPower = CharToFloat(token); break;
            }

            token = strtok(nullptr, " ");
            index++;
        }
        AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: QPIGS parsed OK"));
    }
    else if (strcmp("QPIRI", command) == 0) {
        if (Easun.byteCounter < 95)
            return;

        char *token = strtok(Easun.buffer, " ");
        int index = 0;
        while (token != nullptr)
        {
            switch(index) {
                case 16: 
                  if (TextToInt(token) == 0) strcpy(Easun.SourcePriority, "UTL");
                  else if (TextToInt(token) == 1) strcpy(Easun.SourcePriority, "SOL");
                  else if (TextToInt(token) == 2) strcpy(Easun.SourcePriority, "SBU");
                  else strcpy(Easun.SourcePriority, "???");
                break;
                case 17: 
                  if (TextToInt(token) == 0) strcpy(Easun.ChargingSourcePriority, "CUT");
                  else if (TextToInt(token) == 1) strcpy(Easun.ChargingSourcePriority, "CSO");
                  else if (TextToInt(token) == 2) strcpy(Easun.ChargingSourcePriority, "SUN");
                  else if (TextToInt(token) == 3) strcpy(Easun.ChargingSourcePriority, "OSO");
                  else strcpy(Easun.ChargingSourcePriority, "???");
                break;
            }

            token = strtok(nullptr, " ");
            index++;
        }
        AddLog(LOG_LEVEL_DEBUG, PSTR("[EASUN]: QPIRI parsed OK"));
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

        ResponseAppend_P(PSTR(",\"SourcePriority\": \"%s\""), Easun.SourcePriority);
        ResponseAppend_P(PSTR(",\"ChargingSourcePriority\": \"%s\""), Easun.ChargingSourcePriority);        

        ResponseJsonEnd();
    }
    #ifdef USE_WEBSERVER
    else {
        WSContentSend_P(PSTR("<tr><th colspan=\"2\"><hr/></th></tr>"));
        WSContentSend_P(PSTR("{s}CurrentMode{m}%s{e}"), Easun.CurrentMode);
        WSContentSend_P(PSTR("{s}SourcePriority{m}%s{e}"), Easun.SourcePriority);
        WSContentSend_P(PSTR("{s}ChargingSourcePriority{m}%s{e}"), Easun.ChargingSourcePriority);
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
                    EasunEnqueueCommand((char*) easunStatusCommands[i]);
                }
            }
            easunLoopSeconds = XSNS_101_COMMAND_PERIOD_SECONDS;
        }

        if (EasunSerial && Easun.cmdStatus == 0 && easunCommandIndex > -1) {
            EasunSendCommand();
        }
        break;
    case FUNC_COMMAND:
        // ea_send <command> eg: ea_send QPIGS
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


/*
PIPSOLAR_SENSOR(grid_voltage, QPIGS, float)
   PIPSOLAR_SENSOR(grid_frequency, QPIGS, float)
   PIPSOLAR_SENSOR(ac_output_voltage, QPIGS, float)
   PIPSOLAR_SENSOR(ac_output_frequency, QPIGS, float)
   PIPSOLAR_SENSOR(ac_output_apparent_power, QPIGS, int)
   PIPSOLAR_SENSOR(ac_output_active_power, QPIGS, int)
   PIPSOLAR_SENSOR(output_load_percent, QPIGS, int)
   PIPSOLAR_SENSOR(bus_voltage, QPIGS, int)
   PIPSOLAR_SENSOR(battery_voltage, QPIGS, float)
   PIPSOLAR_SENSOR(battery_charging_current, QPIGS, int)
   PIPSOLAR_SENSOR(battery_capacity_percent, QPIGS, int)
   PIPSOLAR_SENSOR(inverter_heat_sink_temperature, QPIGS, int)
   PIPSOLAR_SENSOR(pv_input_current_for_battery, QPIGS, float)
   PIPSOLAR_SENSOR(pv_input_voltage, QPIGS, float)
   PIPSOLAR_SENSOR(battery_voltage_scc, QPIGS, float)
   PIPSOLAR_SENSOR(battery_discharge_current, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(add_sbu_priority_version, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(configuration_status, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(scc_firmware_version, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(load_status, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(battery_voltage_to_steady_while_charging, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(charging_status, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(scc_charging_status, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(ac_charging_status, QPIGS, int)
   PIPSOLAR_SENSOR(battery_voltage_offset_for_fans_on, QPIGS, int)  //.1 scale
   PIPSOLAR_SENSOR(eeprom_version, QPIGS, int)
   PIPSOLAR_SENSOR(pv_charging_power, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(charging_to_floating_mode, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(switch_on, QPIGS, int)
   PIPSOLAR_BINARY_SENSOR(dustproof_installed, QPIGS, int)
 
   // QPIRI values
   PIPSOLAR_SENSOR(grid_rating_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(grid_rating_current, QPIRI, float)
   PIPSOLAR_SENSOR(ac_output_rating_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(ac_output_rating_frequency, QPIRI, float)
   PIPSOLAR_SENSOR(ac_output_rating_current, QPIRI, float)
   PIPSOLAR_SENSOR(ac_output_rating_apparent_power, QPIRI, int)
   PIPSOLAR_SENSOR(ac_output_rating_active_power, QPIRI, int)
   PIPSOLAR_SENSOR(battery_rating_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(battery_recharge_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(battery_under_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(battery_bulk_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(battery_float_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(battery_type, QPIRI, int)
   PIPSOLAR_SENSOR(current_max_ac_charging_current, QPIRI, int)
   PIPSOLAR_SENSOR(current_max_charging_current, QPIRI, int)
   PIPSOLAR_SENSOR(input_voltage_range, QPIRI, int)
   PIPSOLAR_SENSOR(output_source_priority, QPIRI, int)
   PIPSOLAR_SENSOR(charger_source_priority, QPIRI, int)
   PIPSOLAR_SENSOR(parallel_max_num, QPIRI, int)
   PIPSOLAR_SENSOR(machine_type, QPIRI, int)
   PIPSOLAR_SENSOR(topology, QPIRI, int)
   PIPSOLAR_SENSOR(output_mode, QPIRI, int)
   PIPSOLAR_SENSOR(battery_redischarge_voltage, QPIRI, float)
   PIPSOLAR_SENSOR(pv_ok_condition_for_parallel, QPIRI, int)
   PIPSOLAR_SENSOR(pv_power_balance, QPIRI, int)
 
   // QMOD values
   PIPSOLAR_VALUED_TEXT_SENSOR(device_mode, QMOD, char)
 
   // QFLAG values
   PIPSOLAR_BINARY_SENSOR(silence_buzzer_open_buzzer, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(overload_bypass_function, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(lcd_escape_to_default, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(overload_restart_function, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(over_temperature_restart_function, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(backlight_on, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(alarm_on_when_primary_source_interrupt, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(fault_code_record, QFLAG, int)
   PIPSOLAR_BINARY_SENSOR(power_saving, QFLAG, int)
 
   // QPIWS values
   PIPSOLAR_BINARY_SENSOR(warnings_present, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(faults_present, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_power_loss, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_inverter_fault, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_bus_over, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_bus_under, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_bus_soft_fail, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_line_fail, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_opvshort, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_inverter_voltage_too_low, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_inverter_voltage_too_high, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_over_temperature, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_fan_lock, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_voltage_high, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_low_alarm, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_under_shutdown, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_derating, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_over_load, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_eeprom_failed, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_inverter_over_current, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_inverter_soft_failed, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_self_test_failed, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_op_dc_voltage_over, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_battery_open, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_current_sensor_failed, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_battery_short, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_power_limit, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_pv_voltage_high, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_mppt_overload, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_mppt_overload, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_too_low_to_charge, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_dc_dc_over_current, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(fault_code, QPIWS, int)
   PIPSOLAR_BINARY_SENSOR(warnung_low_pv_energy, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_high_ac_input_during_bus_soft_start, QPIWS, bool)
   PIPSOLAR_BINARY_SENSOR(warning_battery_equalization, QPIWS, bool)
 
   PIPSOLAR_TEXT_SENSOR(last_qpigs, QPIGS)
   PIPSOLAR_TEXT_SENSOR(last_qpiri, QPIRI)
   PIPSOLAR_TEXT_SENSOR(last_qmod, QMOD)
   PIPSOLAR_TEXT_SENSOR(last_qflag, QFLAG)
   PIPSOLAR_TEXT_SENSOR(last_qpiws, QPIWS)
   PIPSOLAR_TEXT_SENSOR(last_qt, QT)
   PIPSOLAR_TEXT_SENSOR(last_qmn, QMN)
 
   PIPSOLAR_SWITCH(output_source_priority_utility_switch, QPIRI)
   PIPSOLAR_SWITCH(output_source_priority_solar_switch, QPIRI)
   PIPSOLAR_SWITCH(output_source_priority_battery_switch, QPIRI)
   PIPSOLAR_SWITCH(output_source_priority_hybrid_switch, QPIRI)
   PIPSOLAR_SWITCH(input_voltage_range_switch, QPIRI)
   PIPSOLAR_SWITCH(pv_ok_condition_for_parallel_switch, QPIRI)
   PIPSOLAR_SWITCH(pv_power_balance_switch, QPIRI)
*/