/*****************************************************
 * Test setup for the CellComms library.
 * 
 * The CellComms library needs two calls to be made periodically
 * 1. A call to sendMillivolts every sample period (5s or so)
 * 2. A call to readCells shortly afterwards.
*****************************************************/


// Options
#define debug             (1)       // enable this to have debug messages sent on serial0
#define LCD_DISPLAY       (1)       // 0 = No LCD display, 1 = LCD display fitted
#define READ_CURRENT      (1)       // enable this to sample a current sensor on an ADC pin. The pin in set in the assignments section
#define ACTIVE_BALANCING  (1)       // enable this to balance the cells all the time, otherwise top balancing only.


#include <CellComms.h>
#if (0 != LCD_DISPLAY)
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
#endif



// Settings
#define DEBUG_BAUD        (9600)    // baudrate for debug comms to the console
#define CELL_LOOP_MS      (6000)    // interval between cell data reads
#define CELL_READ_MS      (400)     // delay after send for the data to shift in
#define AMPS_READ_LOOP_MS (100)     // interval in ms between reading the current sensor.
#define SOC_UPDATE_MS     (1000)    // interval in ms between updating the State-of-Charge (SoC).
#define SCREEN_REFRESH_MS (1000)    // how often the display is updated
#define AMPS_OFFSET       (512)     // ~1/2 of the max ADC count as the current sensor is biased so it can read charge and dis-charge currents.
#define AMPS_SCALAR       (97)      // scalar to convert the ADC count into the actual milliamps.
#define SOC_MAX           (10000)   // 100.00, 100% with 2 d.p.

// Pin Assignments
#define ALARM_LED_PIN     (4)
#define FAULT_LED_PIN     (5)
#define RELAY_PIN         (6)
#define HEART_LED_PIN     (13)
#define AMPS_AN_PIN       (0)       // the ADC pin connected to the current sensor

#if (0 != LCD_DISPLAY)
#define I2C_ADDR          0x3F      // Define I2C Address where the PCF8574A is
#define Rs_pin            0
#define Rw_pin            1
#define En_pin            2
#define BACKLIGHT_PIN     3
#define D4_pin            4
#define D5_pin            5
#define D6_pin            6
#define D7_pin            7
#endif


unsigned int cellMeanMillivolt  = 3600;   // initiate with a high mean value so the loads don't turn on.
unsigned long currentMillis;
unsigned long sendMillis;
unsigned long readMillis;
unsigned long readAmpsMillis;
unsigned long updateSoCMillis;
unsigned long ScreenRefreshMillis;
long milliAmps;
long aveMilliAmps;
long milliAmpHours;
unsigned int SoC;                         // State of Charge

CellComms cells;                          // constructor for the CellComms library

#if (0 != LCD_DISPLAY)
LiquidCrystal_I2C       lcd(I2C_ADDR, En_pin, Rw_pin, Rs_pin, D4_pin, D5_pin, D6_pin, D7_pin);
#endif

void setup() {
  Serial.begin(DEBUG_BAUD);

#ifdef debug
  Serial.println("Starting...");
#endif
  
  pinMode(HEART_LED_PIN, OUTPUT);       // Enable Heart LED
  pinMode(FAULT_LED_PIN, OUTPUT);       // Enable Fault LED
  pinMode(ALARM_LED_PIN, OUTPUT);       // Enable Alarm LED
  pinMode(RELAY_PIN, OUTPUT);           // Enable Relay

#if (0 != LCD_DISPLAY)
  // NEW LCD code using I2C
  lcd.begin (20, 4, LCD_5x8DOTS);
  lcd.setBacklightPin(BACKLIGHT_PIN, POSITIVE); // init the backlight
#endif
  
  digitalWrite(HEART_LED_PIN, HIGH);
  digitalWrite(FAULT_LED_PIN, HIGH);
  delay(200); 
  digitalWrite(ALARM_LED_PIN, HIGH);
  delay(20); 
  digitalWrite(FAULT_LED_PIN, LOW);
  digitalWrite(ALARM_LED_PIN, LOW);
  
#if (0 != LCD_DISPLAY)
  // Print start-up message to the LCD.
  lcd.setBacklight(HIGH);     // Backlight on
  lcd.clear(); // clear display, set cursor position to zero

  lcd.print("   Eco-Ants OSBMS   ");    // NB must be 20 characters or it garbles
  lcd.setCursor(0, 1);
  lcd.print("     BMS master     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 2);
  lcd.print("    16 Jul 2017     ");   // NB must be 20 characters or it garbles
  lcd.setCursor(0, 3);
  lcd.print("Ants, Duncan & Dave ");   // NB must be 20 characters or it garbles

  // delay so we can read initial banner
  delay(2000);
#endif // LCD_DISPLAY

  aveMilliAmps        = 0;
  milliAmpHours       = 35000;          // 35Ah
  SoC                 = 5000;           // 50.00%
  currentMillis       = millis();
  sendMillis          = (currentMillis + 100);
  readAmpsMillis      = (currentMillis + 50);
  updateSoCMillis     = (currentMillis + 1025);
  ScreenRefreshMillis = (currentMillis + 1033);
}

void loop() {
  // this is essentially just a scheduler to call functions at different intervals.
  
  currentMillis       = millis();

  if (currentMillis == sendMillis) {
    startCellComms(CELL_READ_MS);
  } // end of sendMillis
  
  if (currentMillis == readMillis) {
    readCellComms();
  } // end of readMillis
  
#if (0 != READ_CURRENT)
  if (currentMillis == readAmpsMillis) {
    readAmps();
  } // end of readMillis
#endif  // READ_CURRENT

  if (currentMillis == updateSoCMillis)
  {
    updateSoC();
  }

  if (currentMillis == ScreenRefreshMillis)
  {
    // TODO:
  }
} // end of loop


/******************************************
  initiate cell comms by sending the mean cell voltage.
  interval between sends can be varied.
******************************************/
void startCellComms(uint16_t interval) {
    digitalWrite(HEART_LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(FAULT_LED_PIN, LOW);

    // write 6 bytes to the BMS to initiate data transfer.
#if (0 != ACTIVE_BALANCING)
    cells.sendMillivolts(cellMeanMillivolt);
//    cells.sendMillivolts(3600);
#else
    cells.sendMillivolts(0);
#endif

    // wait for the cell data to arrive
    readMillis        = (sendMillis + interval);
    sendMillis        += CELL_LOOP_MS;    // TODO: make the interval a function of the battery current
 } // end of startCellComms --------------------------


/*****************************************************
 * read the cell data once it has arrived.
 * 
 * if all cells read then do stuff with the data.
******************************************************/
 void readCellComms(void) {
  int cellsRead         = cells.readCells();
  
#ifdef debug
  Serial.print("Read ");
  Serial.println(cellsRead);
#endif
    
//  if (cellsRead == NUM_CELLS) {
  if (cellsRead > 0) {
    cellMeanMillivolt     = cells.getCellsAveV();
    int cellMinMillivolt  = cells.getCellsMinV();
    int cellMaxMillivolt  = cells.getCellsMaxV();
    int cellMinTemp       = cells.getCellsMinT();
    int cellMaxTemp       = cells.getCellsMaxT();
    int cellsUndervolt    = cells.getCellsUnderVolt();
    int cellsOvervolt     = cells.getCellsOverVolt();
    int cellsOvertemp     = cells.getCellsOverTemp();
    int cellsBalancing    = cells.getCellsBalancing();
    int cellMinVnum       = cells.getMinVCell();
    int cellMaxVnum       = cells.getMaxVCell();
    int cellMinTnum       = cells.getMinTCell();
    int cellMaxTnum       = cells.getMaxTCell();
  
#ifdef debug
    Serial.print("mean ");
    Serial.println(cellMeanMillivolt);
    
    Serial.print("min ");
    Serial.print(cellMinMillivolt);
    Serial.print(", max ");
    Serial.print(cellMaxMillivolt);
    Serial.print(", delta ");
    Serial.println(cellMaxMillivolt - cellMinMillivolt);
    
    Serial.print("UV ");
    Serial.print(cellsUndervolt);
    Serial.print(", OV ");
    Serial.print(cellsOvervolt);
    Serial.print(", HT ");
    Serial.print(cellsOvertemp);
    Serial.print(", Load ");
    Serial.println(cellsBalancing);
#endif

    // Check conditions to turn the charger ON
    // if any cells are under-voltage then turn the charger on
    if (cellsUndervolt > 0) {
      digitalWrite(RELAY_PIN, HIGH);   // turn charger on
    }
    // enable charger if SoC < 80% ?
    if (cellMaxMillivolt < 3500)
    {
      digitalWrite(RELAY_PIN, HIGH);
    }
    // TODO: may have a manual CHARGE button as well.


    // Check conditions to turn charger OFF
    // if all the cells are balancing then turn the charger off
    if (cellsBalancing == NUM_CELLS) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }
    // if any cells are overvoltage then turn the charger off
    if (cellsOvervolt > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }
    // if any cells are over-temperature then turn the charger off
    if (cellsOvertemp > 0) {
      digitalWrite(RELAY_PIN, LOW);   // turn charger off
    }


    // TODO: do other stuff because the read was good.

    
#if (0 != LCD_DISPLAY)
    lcd.clear(); // clear display, set cursor position to zero
    lcd.print(cellsRead);
    lcd.print(" read, u");
    lcd.print(cellsUndervolt);
    lcd.print(" o");
    lcd.print(cellsOvervolt);
    lcd.print(" t");
    lcd.print(cellsOvertemp);
    
    lcd.setCursor(0, 1);
    lcd.print("Max ");
    lcd.print(cellMaxMillivolt);
    lcd.print(":");
    lcd.print(cellMaxVnum);
    lcd.print(", ");
    lcd.print(cellMaxTemp / 10);
    lcd.print(".");
    lcd.print(cellMaxTemp % 10);
    lcd.print(":");
    lcd.print(cellMaxTnum);
    
    lcd.setCursor(0, 2);
    lcd.print("Min ");
    lcd.print(cellMinMillivolt);
    lcd.print(":");
    lcd.print(cellMinVnum);
    lcd.print(", ");
    lcd.print(cellMinTemp / 10);
    lcd.print(".");
    lcd.print(cellMinTemp % 10);
    lcd.print(":");
    lcd.print(cellMinTnum);
    
    lcd.setCursor(0, 3);
    if (cellsBalancing != 0) {
      lcd.print(cellsBalancing);
      lcd.print(" cells balancing");
    }
    else {
      lcd.print("Mean ");
      lcd.print(cellMeanMillivolt);
      lcd.print(", Delta ");
      lcd.print(cellMaxMillivolt - cellMinMillivolt);
    }
#endif // LCD_DISPLAY
  } // end of all cells read successfully
  else {
    digitalWrite(FAULT_LED_PIN, HIGH);

#if (0 != LCD_DISPLAY)
    lcd.clear();
    lcd.print("My arms are flailing");
    lcd.setCursor(0, 1);
    lcd.print("  Error  !! ");
    lcd.setCursor(0, 2);
    lcd.print(cellsRead); lcd.print(" out of "); lcd.print(NUM_CELLS);
    lcd.setCursor(0, 3);
    lcd.print("Break before # "); lcd.print((NUM_CELLS - cellsRead));
#endif // LCD_DISPLAY
  } // not all cells read

  readMillis            += CELL_LOOP_MS;  // move the trigger value on so it doesn't run 100 times
  
  digitalWrite(HEART_LED_PIN, LOW);   // turn the LED off to show we're done
 } // end of readCellComms ---------------------------


#if (0 != READ_CURRENT)
/*****************************************************
 * reads the current sensor
 ****************************************************/
 void   readAmps(void)
 {
  // sample the adc channel connected to the current sensor
  int val               = analogRead(AMPS_AN_PIN);
  
  // scale the adc reading into milliamps
  milliAmps             = ( (long) val - AMPS_OFFSET) * AMPS_SCALAR;

  // update a filtered average current as well
  aveMilliAmps          = ((aveMilliAmps * 9) + milliAmps) / 10;

  readAmpsMillis        += AMPS_READ_LOOP_MS;
 } // end of readAmps --------------------------------
#endif  // READ_CURRENT


 /****************************************************
  * updates the State of Charge estimate
  * called every 1000ms
  ***************************************************/
  void  updateSoC(void)
  {
    static unsigned int relaxedTime    = 0;
    static long tmpMAs    = 0;
    long        SoCMinimum;
    
    // if ave current is above noise floor, 1% of full scale
    if ( (aveMilliAmps > 1000) || (aveMilliAmps < -1000) )
    {
      // count coulombs or rather totalise the mA seconds
      if (aveMilliAmps > 0)
      {
        // assume 5% loss during charging
        tmpMAs            += (aveMilliAmps - (aveMilliAmps / 20));
      }
      else
      {
        tmpMAs            += (aveMilliAmps);
      }

      // establish the threshold for a change in the SoC
      SoCMinimum          = (milliAmpHours / SOC_MAX);

      // if we have more than 0.01% SoC
      while (tmpMAs > SoCMinimum)
      {
        tmpMAs            -= SoCMinimum;
        if (SoC < SOC_MAX)
        {
          ++SoC;
        } // end of if SoC < 100%
      }
      // if we have more than -0.01% SoC
      while (tmpMAs < (0 - SoCMinimum))
      {
        tmpMAs            += SoCMinimum;
        if (SoC > 0)
        {
          --SoC;
        } // end of if SoC > 0
      }

      relaxedTime         = 0;
    } // end of if enough current to use
    else  if ( (aveMilliAmps < 100) && (aveMilliAmps > -100) )
    {
      ++relaxedTime;
      if (relaxedTime > (60 * 5))   // 5 minutes
      {
        // read Voc
        
        // TODO: need to add a compensation for temperature
  
        // TODO: need an external library to convert open-circuit voltage (compensated for current and temperature into SoC

        relaxedTime       = 0;
      }
    } // end of else low current

    updateSoCMillis       += SOC_UPDATE_MS;
  } // end of updateSoC ------------------------------


/*****************************************************
 * Decodes a value, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 * 
 * the 'balancing' parameter is an output.
 ****************************************************/
String  decodeVoltage(byte payload0, byte payload1, byte decoded0, byte decoded1) {

  float devisor = 1000.0;
  int precision = 3;
  int value =     ((decoded0 & 0x7f) << 8) | decoded1;//Mask off MSB
  float ret_val = 0.00f;
    
  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    return String(( (float) value / devisor), precision);   
  }  
}


/*******************************************************
 * Decodes Temperature, using fec to determine validity.
 * Returns value as a string repesentation of a float, or a string containing 'NULL' if invalid.
 *******************************************************/
String  decodeTemperature(byte payload0, byte payload1, byte decoded0, byte decoded1) {

  float devisor = 10.0;
  int precision = 2;
  int value =     ((decoded0 & 0x3f) << 8) | decoded1;
  float ret_val = 0.00f;

  if (payload0 != decoded0 || payload1 != decoded1) {
    return "NULL";
  } else {
    return String(( (float) value / devisor),precision);   
  }
}


String int2String(int number)
{
  char buf[6];
  byte bufIndex   = 0;

  if (number > 10000)
  {
    buf[bufIndex] = (number / 10000) + '0';
    number        = (number % 10000);
    ++bufIndex;
  }
  if (number > 1000)
  {
    buf[bufIndex] = (number / 1000) + '0';
    number        = (number % 1000);
    ++bufIndex;
  }
  if (number > 100)
  {
    buf[bufIndex] = (number / 100) + '0';
    number        = (number % 100);
    ++bufIndex;
  }
  if (number > 10)
  {
    buf[bufIndex] = (number / 10) + '0';
    number        = (number % 10);
    ++bufIndex;
  }
  if (number > 0)
  {
    buf[bufIndex] = (number) + '0';
    ++bufIndex;
  }
  else
  {
    buf[bufIndex] = '0';
    ++bufIndex;
  }

  // append a string terminator
    buf[bufIndex] = 0;
  
  return String(buf);
}



