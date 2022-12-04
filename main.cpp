#include <Arduino.h>
#include "esp_adc_cal.h"
#include <LiquidCrystal_I2C.h>
#include "BluetoothSerial.h"

//-- Defines --//
#define IN_BUTTON 15   // Acress Button
#define DC_BUTTON 4    // Decress Button
#define IN 14          // TAC in PIN
#define DAC 26         // DAC pin
#define sizeAverage 10 // Size Moving Average
//-------------//

//-- Objects --//
LiquidCrystal_I2C lcd(0x27, 16, 2);
esp_adc_cal_characteristics_t adc_chars;
hw_timer_t *timer0 = NULL;
static BluetoothSerial SerialBT;
//-------------//

//-- Global ---//
int movAverage[10];
byte mov_idx;
int valINV, counterAC, counterDC, counterDisplay, rpm, modeCT;
bool flag_IN, flag_DC, showdisplay, MODE = 0;
//-------------//

//-- PI -------//
float P, I, Kp, Ki = 0;
unsigned int ValPI = 0;
int erro, rpmSet = 0;
bool timeKi;
//-------------//

//-- Declaration Functions --//
void IRAM_ATTR interrupt();
void tacoRead(void *parameter);
int movingAverage(int value);
//---------------------------//

void setup()
{
  //-- Serial Begin --//
  Serial.begin(115200);
  //------------------//

  //-- Bluetooth --//
  SerialBT.begin("Projeto-ESP");
  //---------------//

  //-- LCD Begin --//
  lcd.init();
  lcd.backlight();
  //---------------//

  //-- Set I/O's --------//
  pinMode(IN_BUTTON, INPUT_PULLUP);
  pinMode(DC_BUTTON, INPUT_PULLUP);
  //---------------------//

  //-- Task -------------//
  xTaskCreate(tacoRead, "tacoRead", 1000, NULL, 0, NULL);
  //---------------------//

  //--- Timer Config -------------//
  timer0 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer0, &interrupt, true);
  timerAlarmWrite(timer0, 1000, true);
  timerAlarmEnable(timer0);
  //------------------------------//

  //-- ADC set attenuation -------//
  esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //------------------------------//
}

/*
 *  __  __      _        _
 * |  \/  |__ _(_)_ _   | |   ___  ___ _ __
 * | |\/| / _` | | ' \  | |__/ _ \/ _ \ '_ \
 * |_|  |_\__,_|_|_||_| |____\___/\___/ .__/
 *                                    |_|
 */

void loop()
{
  if (!MODE) // Open Loop
  {
    if (showdisplay) // True between 100ms
    {
      //-- LCD Print -------------------------------------//
      lcd.setCursor(0, 0);
      lcd.printf("Freq: %02d%c      ", valINV, char(37));
      lcd.setCursor(0, 1);
      lcd.printf("Speed:%04d RPM", rpm);
      //--------------------------------------------------//

      //-- Serial Print ----------------------------------//
      Serial.printf("Freq : %02d%c ", valINV, char(37));
      Serial.printf("Speed:%04d RPM\n", rpm);
      //--------------------------------------------------//

      //-- Bluetooth Print -------------------------------//
      SerialBT.printf("Freq : %02d%c    ", valINV, char(37));
      SerialBT.printf("Speed:%04d RPM \n", rpm);
      //--------------------------------------------------//

      showdisplay = false; // Reset variable
    }

    dacWrite(DAC, map(valINV, 0, 100, 0, 255)); // Write on DAC pin
  }
  else // Closed Loop
  {
    if (showdisplay) // True between 100ms
    {
      //-- LCD Print -------------------------------------//
      lcd.setCursor(0, 0);
      lcd.printf("SetPoint: %04d ", rpmSet);
      lcd.setCursor(0, 1);
      lcd.printf("Speed:%04d RPM", rpm);
      //---------------------------------------------------//

      //-- Serial Print -----------------------------------//
      Serial.printf("DAC: %03d Speed:%04d RPM \n", ValPI, rpm);
      //---------------------------------------------------//

      //-- Bluetooth Print --------------------------------//
      SerialBT.printf("SetPoint: %04d  ", rpmSet);
      SerialBT.printf("Speed:%04d RPM \n", rpm);
      //---------------------------------------------------//

      showdisplay = false; // Reset variable
    }

    //-- Integrative Proportional Controller --//
    Kp = 0.19; // Constant Proportional
    Ki = 0.30; // Constant Integrative

    //-- Erro ----------//
    erro = rpmSet - rpm;
    //------------------//

    //-- Proportional --//
    P = erro * Kp;
    //------------------//

    //-- Integrative ---//
    if (timeKi && ValPI < 255)
    {
      I += (erro * Ki) * 0.001;
      timeKi = 0;
    }
    //------------------//

    ValPI = P + I;                           // sum of values
    dacWrite(DAC, constrain(ValPI, 0, 255)); // Write on DAC pin
    //-----------------------------------------//
  }
}

/*  ___             _   _
 * | __|  _ _ _  __| |_(_)___ _ _  ___
 * | _| || | ' \/ _|  _| / _ \ ' \(_-<
 * |_| \_,_|_||_\__|\__|_\___/_||_/__/
 */

void tacoRead(void *parameter)
{
  while (true) // Loop
  {
    unsigned long int buffer = 0;
    for (byte i = 0; i < 100; i++) // 100 Readings
    {
      buffer += esp_adc_cal_raw_to_voltage(analogRead(IN), &adc_chars); // Save on buffer
      ets_delay_us(100);
    }

    int tacoMV = movingAverage(buffer / 100); // Returns the moving average result
    rpm = map(tacoMV, 128, 2180, 0, 1943);    // RPM conversion

    vTaskDelay(1); // Delay for FreeRTOS manage tasks
  }
}

int movingAverage(int value)
{
  int bufferAverage = 0;       // Local Variable
  movAverage[mov_idx] = value; // Get current value
  mov_idx++;                   // Plus idx

  if (mov_idx >= sizeAverage)
  {
    for (byte i = 0; i < sizeAverage; i++)
    {
      bufferAverage += movAverage[i]; // Get all values

      if (i < sizeAverage - 1)
        movAverage[i] = movAverage[i + 1]; // Shift Left
      else
        movAverage[i] = 0; // Shift Left Sucess
    }
    if (mov_idx)
      mov_idx--;

    bufferAverage = bufferAverage / sizeAverage; // Moving average result
  }
  else // if the moving average is not full
  {
    for (byte i = 0; i < mov_idx; i++)
      bufferAverage += movAverage[i];

    bufferAverage = bufferAverage / mov_idx; // Moving average result
  }

  return bufferAverage; // Returns moving average result
}

void IRAM_ATTR interrupt()
{
  //-- Button Read --------------------------//
  if (!digitalRead(IN_BUTTON) && !flag_IN)
    flag_IN = true;
  if (digitalRead(IN_BUTTON) && flag_IN)
  {
    if (MODE)
      rpmSet += 50; // Increase
    else
      valINV++; // Increase

    flag_IN = 0; // Reset flag
  }

  if (!digitalRead(DC_BUTTON) && !flag_DC)
    flag_DC = true;
  if (digitalRead(DC_BUTTON) && flag_DC)
  {
    if (MODE)
      rpmSet -= 50; // Decrease
    else
      valINV--; // Decrease

    flag_DC = 0; // Reset flag
  }

  if (!digitalRead(IN_BUTTON) && !digitalRead(DC_BUTTON))
  {
    modeCT++;
    if (modeCT == 200)
    {
      MODE = MODE ^ 1; // Toogle Mode
      valINV = 0;      // Reset
      rpmSet = 0;      // Reset
    }
  }
  else
    modeCT = 0;
  //------------------------------------------//

  //-- Increment Logic -----------------------//
  if (!digitalRead(IN_BUTTON))
    counterAC++;
  else
    counterAC = 0;
  if (!digitalRead(DC_BUTTON))
    counterDC++;
  else
    counterDC = 0;

  if (counterAC >= 200)
  {
    valINV++;
    counterAC = 0;
  }

  if (counterDC >= 200)
  {
    valINV--;
    counterDC = 0;
  }

  // Set Max/Min interval
  valINV = constrain(valINV, 0, 100);
  rpmSet = constrain(rpmSet, 0, 1950);
  //------------------------------------------//

  //-- Show display delay ---//
  counterDisplay++;
  if (counterDisplay >= 10)
  {
    showdisplay = true;
    counterDisplay = 0;
  }
  //-------------------------//

  timeKi = true; // Time to increase integral;
}
