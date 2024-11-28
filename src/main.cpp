
#ifndef XPOWERS_NO_ERROR
#endif
// Defined using AXP2102
#define XPOWERS_CHIP_AXP2101

#include <Wire.h>
#include <Arduino.h>
#include "XPowersLib.h"

#ifndef CONFIG_PMU_SDA
#define CONFIG_PMU_SDA 21
#endif
#define I2C_SDA 22
#define I2C_SCL 23
#ifndef CONFIG_PMU_SCL
#define CONFIG_PMU_SCL 22
#endif

#ifndef CONFIG_PMU_IRQ
#define CONFIG_PMU_IRQ 35
#endif
#define boosten 12
#define PIN_DETECTION 15
bool  pmu_flag = true;
XPowersPMU power;

const uint8_t i2c_sda = I2C_SDA;
const uint8_t i2c_scl = I2C_SCL;
const uint8_t pmu_irq_pin = CONFIG_PMU_IRQ;

void setFlag(void)
{
    pmu_flag = true;
}


void setup() 
{
    delay(1000);
    pinMode(boosten, OUTPUT);
    digitalWrite(boosten, HIGH);
    delay(1500);
    Serial.begin(115200);
    bool result = power.begin(Wire, AXP2101_SLAVE_ADDRESS, i2c_sda, i2c_scl);
    if (result == false) {
        Serial.println("power is not online..."); while (1)delay(50);
    }
    Serial.printf("getID:0x%x\n", power.getChipID());
    pinMode(PIN_DETECTION, INPUT_PULLUP);

    power.enableBattDetection();
    power.enableVbusVoltageMeasure();
    power.enableBattVoltageMeasure();
    power.enableSystemVoltageMeasure();

    power.setVbusVoltageLimit(XPOWERS_AXP2101_VBUS_VOL_LIM_5V08);
    power.setVbusCurrentLimit(XPOWERS_AXP2101_VBUS_CUR_LIM_2000MA);
    power.setSysPowerDownVoltage(3000);
    power.setPrechargeCurr(XPOWERS_AXP2101_PRECHARGE_50MA);
    power.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_200MA);
    power.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    power.setDC1Voltage(3300);
    power.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    power.setPowerKeyPressOnTime(XPOWERS_POWERON_1S);
    power.disableTSPinMeasure();
    power.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    uint16_t voltage_bat_low = power.getSysPowerDownVoltage();
    Serial.printf("->  getSysPowerDownVoltage:%u\n", voltage_bat_low);
    uint16_t current_input_max = power.getVbusCurrentLimit();
    Serial.printf("->  VbusCurrentLimit:%u\n", current_input_max);
    // DC1 IMAX=2A
    // 1500~3400mV,100mV/step,20steps
    power.enableDC1();
    power.disableDC2();
    power.disableDC3();
    power.disableDC4();
    power.disableDC5();
    power.disableALDO1();
    power.disableALDO2();
    power.disableALDO3();
    power.disableALDO4();
    power.disableBLDO1();
    power.disableBLDO2();
    power.disableCPUSLDO();
    power.disableDLDO1();
    power.disableDLDO2();
    power.disablePwrOkPinPullLow();
    power.disablePwronShutPMIC();
    power.disableWatchdog();
    
    Serial.println("DCDC=======================================================================");
    Serial.printf("DC1  : %s   Voltage:%u mV \n",  power.isEnableDC1()  ? "+" : "-", power.getDC1Voltage());
    Serial.printf("DC2  : %s   Voltage:%u mV \n",  power.isEnableDC2()  ? "+" : "-", power.getDC2Voltage());
    Serial.printf("DC3  : %s   Voltage:%u mV \n",  power.isEnableDC3()  ? "+" : "-", power.getDC3Voltage());
    Serial.printf("DC4  : %s   Voltage:%u mV \n",  power.isEnableDC4()  ? "+" : "-", power.getDC4Voltage());
    Serial.printf("DC5  : %s   Voltage:%u mV \n",  power.isEnableDC5()  ? "+" : "-", power.getDC5Voltage());
    Serial.println("ALDO=======================================================================");
    Serial.printf("ALDO1: %s   Voltage:%u mV\n",  power.isEnableALDO1()  ? "+" : "-", power.getALDO1Voltage());
    Serial.printf("ALDO2: %s   Voltage:%u mV\n",  power.isEnableALDO2()  ? "+" : "-", power.getALDO2Voltage());
    Serial.printf("ALDO3: %s   Voltage:%u mV\n",  power.isEnableALDO3()  ? "+" : "-", power.getALDO3Voltage());
    Serial.printf("ALDO4: %s   Voltage:%u mV\n",  power.isEnableALDO4()  ? "+" : "-", power.getALDO4Voltage());
    Serial.println("BLDO=======================================================================");
    Serial.printf("BLDO1: %s   Voltage:%u mV\n",  power.isEnableBLDO1()  ? "+" : "-", power.getBLDO1Voltage());
    Serial.printf("BLDO2: %s   Voltage:%u mV\n",  power.isEnableBLDO2()  ? "+" : "-", power.getBLDO2Voltage());
    Serial.println("CPUSLDO====================================================================");
    Serial.printf("CPUSLDO: %s Voltage:%u mV\n",  power.isEnableCPUSLDO() ? "+" : "-", power.getCPUSLDOVoltage());
    Serial.println("DLDO=======================================================================");
    Serial.printf("DLDO1: %s   Voltage:%u mV\n",  power.isEnableDLDO1()  ? "+" : "-", power.getDLDO1Voltage());
    Serial.printf("DLDO2: %s   Voltage:%u mV\n",  power.isEnableDLDO2()  ? "+" : "-", power.getDLDO2Voltage());
    Serial.println("===========================================================================");
    
    bool en;
    power.setDC1LowVoltagePowerDowm(false);
    en = power.getDC1LowVoltagePowerDowmEn();
    Serial.print("getDC1LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");

    power.disableTSPinMeasure();
    Serial.print("getDC1LowVoltagePowerDowmEn:");
    Serial.println(en ? "ENABLE" : "DISABLE");
    power.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V1);
    power.disableWatchdog();
    power.enableButtonBatteryCharge();
    // Enable cell battery charge function
    // Print default setting current

}

void printPMU()
{
    Serial.print("isCharging:"); Serial.println(power.isCharging() ? "YES" : "NO");
    Serial.print("isDischarge:"); Serial.println(power.isDischarge() ? "YES" : "NO");
    // Serial.print("isStandby:"); Serial.println(power.isStandby() ? "YES" : "NO");
    Serial.print("isVbusIn:"); Serial.println(power.isVbusIn() ? "YES" : "NO");
    Serial.print("isVbusGood:"); Serial.println(power.isVbusGood() ? "YES" : "NO");
    Serial.print("getChargerStatus:");
    uint8_t charge_status = power.getChargerStatus();
    if (charge_status == XPOWERS_AXP2101_CHG_TRI_STATE) {
        Serial.println("tri_charge");
    } else if (charge_status == XPOWERS_AXP2101_CHG_PRE_STATE) {
        Serial.println("pre_charge");
    } else if (charge_status == XPOWERS_AXP2101_CHG_CC_STATE) {
        Serial.println("constant charge");
    } else if (charge_status == XPOWERS_AXP2101_CHG_CV_STATE) {
        Serial.println("constant voltage");
    } else if (charge_status == XPOWERS_AXP2101_CHG_DONE_STATE) {
        Serial.println("charge done");
    } else if (charge_status == XPOWERS_AXP2101_CHG_STOP_STATE) {
        Serial.println("not charge");
    }

    Serial.print("getBattVoltage:"); Serial.print(power.getBattVoltage()); Serial.println("mV");
    Serial.print("getVbusVoltage:"); Serial.print(power.getVbusVoltage()); Serial.println("mV");
    Serial.print("getSystemVoltage:"); Serial.print(power.getSystemVoltage()); Serial.println("mV");

    // The battery percentage may be inaccurate at first use, the PMU will automatically
    // learn the battery curve and will automatically calibrate the battery percentage
    // after a charge and discharge cycle
    if (power.isBatteryConnect()) {
        Serial.print("getBatteryPercent:"); Serial.print(power.getBatteryPercent()); Serial.println("%");
    }

    Serial.println();
}






void loop()
{

    printPMU();
    int pinState = digitalRead(PIN_DETECTION);
    if (pinState == LOW) {
      digitalWrite(boosten, LOW);
      delay(1000);
    } else if (pinState == HIGH) {
      digitalWrite(boosten, HIGH);
      delay(1000);
    }
    delay(15000);   
}

