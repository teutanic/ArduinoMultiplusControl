#include <SPI.h>                        // needed for Ethernet shield
#include <Ethernet.h>                   // Standard Ethernet library
#include <Timer.h>                      // Timer Library from Jack Christensen
#include <genieArduino.h>               // 4D systems display 

// #define DEBUG                       // only used when on Arduino MEGA or without display 
#define DISPLAY_PRESENT                // 4d Screen available - comment out when on UNO with DEBUG

/* **************************************************************************************************************************
                                                4D DISPLAY DEFINITIONS
*/
#if defined(DISPLAY_PRESENT)
Genie genie;
#define RESETLINE 4                    // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
#endif

char sleepMessage[]       = {"Going to sleep in 10 seconds"};
char writeTimeout[]       = {"Write Timeout"};
char writeOkMessage[]     = {"Write successful"};
char writeFailedMessage[] = {"Write Failed"};
char writeLimitMessage[]  = {"Writing Limit"};

#define AC_ON_LED 2               // Led showing AC on
#define CHARGER_BUTTON 1          // Switching charger on/off
#define INVERTER_BUTTON 0         // Switching inverter on/off
#define CHARGER_LED 1             // Showing charger on when green
#define INVERTER_LED 0            // Showing inverter on when green
#define LED_DIGITS 0              // Id for the LED digits
#define SLIDER 0                  // Id for the slider
#define SLEEP_FORM 1              // Form Id for the sleep form 
#define FORM_ACTIVE 1             // Status for form active
#define CONTRAST_OFF 0            // Status flag for contrast off
#define CONTRAST_ON 1             // Status flag for contrast on
#define MAIN_FORM 0               // Form Id for main form
#define WAKE_UP_BUTTON 2          // Id for wake up button
#define LIMIT_0_BUTTON 2          // IDs for fixed limit buttons
#define LIMIT_10_BUTTON 3
#define LIMIT_15_BUTTON 4
#define LIMIT_28_BUTTON 5
#define LIMIT_SET_BUTTON 6        // Id of set limit button
/* **************************************************************************************************************************
                                                MODBUS DEFINITIONS
*/
#define LIMIT_REGISTER        22                      // Multi field for active current limit
#define MAIN_SWITCH_REGISTER  33                      // Multi field for power switch
#define AC_IN_REGISTER        3                       // multi field for AC volateg in phase 1
#define POWER_ON 3                                    // Status All on for power switch Multi
#define POWER_OFF 4                                   // Status all off for power switch Multi
#define CHARGE_ON 1                                   // Only charger on of Multi
#define INVERTER_ON 2                                 // Only inverter on of Multi
#define MULTI_UID 246                                 // Multi slaveId on the CCGX
#define FC_REGISTER_WRITE 6                           // Function code write single register
#define FC_REGISTER_READ 4                            // Function code read single register
#define MB_PORT 502                                   // Standard Modbus Port 
uint8_t modbusRequest[26];                            // send and recieve buffer
int transactionCount = 1;                             // Identifier for Modbus Transaction
byte registerList[] = { LIMIT_REGISTER,               // field for limit
                        MAIN_SWITCH_REGISTER,         // field for main switch 
                        AC_IN_REGISTER                // field for AC in
                      };                              // the 3 registers we read from the multi
int numRegistersToRead = 3;

/* **************************************************************************************************************************
                                                ETHERNET DEFINITIONS
*/
EthernetServer ModbusSlave(MB_PORT);                              // This is the CCGX
EthernetClient ModbusMaster;                                      // This is us - this Arduino
// Ethernet settings (depending on MAC and Local network)
byte mac[] = {0x90, 0xA2, 0xEA, 0x0E, 0x54, 0xB5 };               // needs to be unique on the network
IPAddress ip(192, 168, 0, 171);                                   // fixed ip
IPAddress slaveIp(192, 168, 0, 8);                                // fixed ip for CCGX


/* ****************************************************************************************************************
                                                   GLOBALS
*/
boolean chargerOn = false;              // Status flag - charger On
boolean inverterOn = false;             // Status flag - inverter On
boolean limitAdjusting = false;         // Current Limit is currently being adjusted
boolean needsUpdate = false;            // Display update required
boolean displayIsOff = false;           // Display is switched dark
boolean goToSleep = false;              // Sleeptime reached
boolean updateSleepCounter = false;     // Timer triggers update of the sleep counter
boolean modbusWait = false;             // Currently waiting for the Modbus to answer
boolean modbusTimeOut = false;          // Modbus connection timed out
boolean readInProgress = false;         // Currently reading from Modbus Slave
boolean writeInProgress = false;        // Currently writing to Modbus slave
boolean startSampling = false;          // Indicates request to start with sampling (reading registers)
boolean acOn = false;                   // Status flag - AC iput is active
boolean activateMainForm = false;       // True when required to switch to Form 0
boolean messageDisplayed = false;       // true when a message is displayed in the message string
int sampleId = 0;                       // Index of the sampling process into the register list
int limit = 0;                          // The current limit on the display
int clearCount = 0;                     // counting to leave the message visible

Timer sleepTimer;                       // A timer to switch the display to dark
int sleepCount = 0;                     // counting to 60 every second
int sleepTimerId;                       // Id of the active timer

Timer sampleTimer;                      // Set to activate sampling
int sampleTimerId;                      // Id for the current sampling timer

Timer timeoutModbusTimer;               // Starts when reading or writing to monitor timeout
int timeoutModbusTimerId;               // Id for the current modbus timer


#define BASIC_SLEEP_TIME 1000           // timer for 1 second for sleep count
#define TIMEOUT_TIME 5000               // Timeout is 5 seconds for modbus transmission 

/* ****************************************************************************************************************
                                                   SETUP
*/
void setup() {
  // serial setup
#if defined(DEBUG)                       // Debug must be off when on UNO and display is running
  Serial.begin(115200);                  // Serial Monitor
#endif

  // ------------------------------------  Init display -----------------------------------------------------------------------
  // change Serial0 to Serial1 when testing on MEGA and connect display via jumper cables, remove jumper 3 and 4 on the shield
#if defined(DISPLAY_PRESENT)
  Serial1.begin(200000);                                  // 4D system display
  delay (500);                                           //let the display start up after the reset
  //Serial.println("Initialise Display");
  genie.Begin(Serial1);                                   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  genie.AttachEventHandler(myGenieEventHandler);         // Attach the user function Event Handler for processing events
  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE, OUTPUT);                             // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 1);                             // Reset the Display via D4
  delay(200);
  digitalWrite(RESETLINE, 0);                             // unReset the Display via D4
  delay (6000);                                           //let the display start up after the reset (This is important 3500 is too short)
#endif  // -------------  end display init 

  // ------------------------------------  Init Ethernet Shield -------------------------
  // initialize the ethernet device
  Ethernet.begin(mac, ip);
  delay(1000);                                            // let it connect

#if defined(DISPLAY_PRESENT)
  sleepTimerId = sleepTimer.after(BASIC_SLEEP_TIME, countSleepTime);  // The display goes to sleep after 60 seconds
#endif

#if defined(DEBUG)
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
#endif

}

/* ****************************************************************************************************************
                                                   MAIN LOOP
*/
void loop() {
#if defined(DISPLAY_PRESENT)

  if (activateMainForm) {                                   // When the display comes back from sleep
    genie.WriteContrast(1);                                 // switch backlight on
    genie.WriteObject(GENIE_OBJ_FORM, 2, 1);                // switch to Form 2 (the flag page)
    genie.WriteObject(GENIE_OBJ_4DBUTTON, 7, 0);            // Main button to off
    activateMainForm = false;                               // make sure we do not do it again next round
  }

  updateDisplay();                                          // check if we need to update the display and do so
  genie.DoEvents();                                         // check if we have any touch events on the display
#endif

  if (startSampling) {                                      // Timer has elapsed and flag was set for sampling
    clearMessage() ;                                        // Clear the message line
    if (!readInProgress) {                                  // Only start a new read of none in progress
      if (sampleId < numRegistersToRead) {                  // read if we are not at the end of the list
        // true if connected, false if not
        readInProgress = accessRegister(4,                  // function code read single register
                                        transactionCount++, // transaction Id
                                        246,                // fixed unitId for multi on the CCGX
                                        registerList[sampleId], // the current register number
                                        0);                 // not needed for read, it is the data for write
      }
    }
    if (readInProgress) {                                   // in case we started a read we no w need to wait
      if (waitForResponse()) {
        // the wait is essential
        // process the result
        int value = readResponse();                         // get the Modbus response
        needsUpdate = true;                                 // update the display anyway - no matter what

        if (registerList[sampleId] == AC_IN_REGISTER) {     // we were reading AC IN register
          if (value > 200) {                                // over 200V is considered AC on
            acOn = true;
            // AC input connected - Inverter had to be on for this to read
          } else {
            acOn = false;
          }
#if defined(DEBUG)
          //             Serial.print("AC ON " ); Serial.println( acOn);
#endif
        } else if (registerList[sampleId] == MAIN_SWITCH_REGISTER) { // Inverter switch status
          switch (value) {
            case 1:
              chargerOn = true;
              inverterOn = false;
              break;
            case 2:
              chargerOn = false;
              inverterOn = true;
              break;
            case 3:
              chargerOn = true;
              inverterOn = true;
              break;
            case 4:
              chargerOn = false;
              inverterOn = false;
              break;
          }

        } else if (registerList[sampleId] == LIMIT_REGISTER) {    // Current Active Limit
          limit = value;
          limitAdjusting = true;                                  // make sure we update everything
#if defined(DEBUG)
          Serial.print("Limit " );  Serial.println( value);
#endif
        }
        sampleId++;                                               // read the next register
        readInProgress = false;                                   // no read currently - last one is finished
        if (sampleId >= numRegistersToRead) {                     // wrap around to 0 at the end of the list
          startSampling = false;                                  // but stop sampling
          sampleTimerId = sampleTimer.after(10000, updateStatus); // until the timer elapses again
        }
      } else {
        // check here for timeout
        // not yet handled, when Arduino starts without Ethernet we might become hanging
        if (modbusTimeOut) {
          readInProgress = false;
          sampleId = 0;
          modbusTimeOut = false;
          startSampling = false;                                  // but stop sampling
          sampleTimerId = sampleTimer.after(10000, updateStatus); // until the timer elapses again
        }
      }
    }
  }

#if defined(DISPLAY_PRESENT)
  sleepTimer.update();
  checkSleepTimer();
#endif
  sampleTimer.update();
  timeoutModbusTimer.update();
}

/* ****************************************************************************************************************
                                                   HELPER FUNCTIONS
*/

#if defined(DISPLAY_PRESENT)
void countSleepTime() {
  sleepTimer.stop(sleepTimerId);
  updateSleepCounter = true;
}
#endif

void updateStatus() {
  sampleTimer.stop(sampleTimerId);
  if (!displayIsOff)
    startSampling = true;
  sampleId = 0;
}

/* ****************************************************************************************************************
                                                   DISPLAY FUNCTIONS
*/
#if defined(DISPLAY_PRESENT)

void checkSleepTimer() {
  if (updateSleepCounter) {
    sleepCount ++;
    updateSleepCounter = false;
    if (sleepCount > 50) {
      displayMessage(sleepMessage);
    }

    if (sleepCount > 60) {                 // switch off display after 1 minute
      goToSleep = true;
      clearMessage();
    } else {
      sleepTimerId = sleepTimer.after(BASIC_SLEEP_TIME, countSleepTime);
    }
  }
  if (goToSleep) {
    displayOff();
  }
}

void displayMessage(char* message) {
  genie.WriteStr(0, message);               // show the message
  messageDisplayed = true;                  // indicate message is displayed
  clearCount = 0;                           // make sure message stays visible for a moment 
}

void clearMessage() {
  if (messageDisplayed) {                   // only relevant when message is displayed
    clearCount++;                           // leave it visible for a moment
    if (clearCount == 10) {                  // when time is come to clear
      char empty[] = {""};                  // write an empty string
      genie.WriteStr(0, empty);             // to the message line
      clearCount = 0;                       // start time count again
    }
  }
}


void updateDisplay() {
  if (needsUpdate) {

    genie.WriteObject(GENIE_OBJ_LED, AC_ON_LED, !acOn);    // AC IN led
    genie.WriteObject(GENIE_OBJ_4DBUTTON, CHARGER_BUTTON, chargerOn);
    genie.WriteObject(GENIE_OBJ_LED, CHARGER_LED, !chargerOn);       // 0 will be green
    genie.WriteObject(GENIE_OBJ_4DBUTTON,INVERTER_BUTTON, inverterOn);
    genie.WriteObject(GENIE_OBJ_LED, INVERTER_LED, !inverterOn);      // 0 will be green
    if (limitAdjusting) {
      genie.WriteObject(GENIE_OBJ_LED_DIGITS, LED_DIGITS, limit);
      genie.WriteObject(GENIE_OBJ_SLIDER, SLIDER, limit);
    }
  }
}

void sleepFormOn() {
  genie.WriteObject(GENIE_OBJ_FORM, SLEEP_FORM, FORM_ACTIVE);
}

void displayOff() {
  displayIsOff = true;
  sleepFormOn();
  genie.WriteContrast(CONTRAST_OFF);
}

void displayOn() {
  displayIsOff = false;
  genie.WriteContrast(CONTRAST_ON);
  activateMainForm = true;
  goToSleep = false;
  sleepTimerId = sleepTimer.after(BASIC_SLEEP_TIME, countSleepTime);
  sleepCount = 0;
}


void myGenieEventHandler(void)
{
  genieFrame Event;
  genie.DequeueEvent(&Event);
  //  Serial.println("Check Event");
  //If the cmd received is from a Reported Event (Events triggered from the Events tab of Workshop4 objects)
  if (Event.reportObject.cmd == GENIE_REPORT_EVENT) {
    sleepCount = 0;
    clearMessage();
    int index;
    int key = 0;
    int slider_val = 0;
    switch (Event.reportObject.object) {
      case GENIE_OBJ_4DBUTTON:                                      // If the Reported Message was from a 4D Button
        handle4Dbutton(Event);
        break;
      case GENIE_OBJ_FORM:
        index = Event.reportObject.index;
        if (index == MAIN_FORM)  {                                   // page 0 is actice - start sampling
          startSampling = true;
        }
        if (index == WAKE_UP_BUTTON) {
          if (displayIsOff)
            displayOn();
        }
        break;
      case GENIE_OBJ_SLIDER:
        limit = genie.GetEventData(&Event);                           // Receive the event data from the Slider0
        limitAdjusting = true;
        break;
      case GENIE_OBJ_USERBUTTON:
        displayOn();
        break;
    }
  }
}

void updatePowerSwitch() {
  int powerStatus = 0;
  if (chargerOn && inverterOn) powerStatus = POWER_ON;
  if (!chargerOn && !inverterOn) powerStatus = POWER_OFF;
  if (chargerOn && !inverterOn) powerStatus = CHARGE_ON;
  if (!chargerOn && inverterOn) powerStatus = INVERTER_ON;

  writeInProgress = accessRegister(FC_REGISTER_WRITE, transactionCount++, MULTI_UID, MAIN_SWITCH_REGISTER, powerStatus);
  if (writeInProgress) {
    while (!waitForResponse()) {
      // the wait is essential
      if (modbusTimeOut) {
        writeInProgress = false;
        displayMessage(writeTimeout);
        startSampling = true;
        modbusTimeOut = false;
        return;
      }
    }
    int value = readResponse();
    writeInProgress = false;
    if (value == FC_REGISTER_WRITE) {
      displayMessage(writeOkMessage);
    }
  } else {
    displayMessage(writeFailedMessage);
    writeInProgress = false;
  }
  startSampling = true;
}

// Button pressed on display - check which one and process it
//
void handle4Dbutton(genieFrame Event) {
  int sw_val = 0;
  switch (Event.reportObject.index) {
    case CHARGER_BUTTON:   // charger button
      chargerOn = genie.GetEventData(&Event);              // Receive the event data from the charge on button
      needsUpdate = true;
      updatePowerSwitch();
      break;
    case INVERTER_BUTTON:   // inverter button
      inverterOn = genie.GetEventData(&Event);              // Receive the event data from the inverter on button
      needsUpdate = true;
      updatePowerSwitch();
      break;
    case LIMIT_0_BUTTON:
      limit = 0;
      needsUpdate = limitAdjusting = true;
      break;
    case LIMIT_10_BUTTON:
      limit = 100;
      needsUpdate = limitAdjusting = true;
      break;
    case LIMIT_15_BUTTON:
      limit = 150;
      needsUpdate = limitAdjusting = true;
      break;
    case LIMIT_28_BUTTON:
      limit = 280;
      needsUpdate = limitAdjusting = true;
      break;
    case LIMIT_SET_BUTTON:
      setLimitTo(limit);
      displayMessage(writeLimitMessage);
      break;
  }
}

// Set button pressed - write the new limit
//
void setLimitTo(int value) {
  writeInProgress = accessRegister(FC_REGISTER_WRITE, transactionCount++, MULTI_UID, LIMIT_REGISTER, limit);
  if (writeInProgress) {
    while (!waitForResponse()) {
      // the wait is essential
      if (modbusTimeOut) {
        writeInProgress = false;
        displayMessage(writeTimeout);
        startSampling = true;
        modbusTimeOut = false;
        return;
      }
    }
    int value = readResponse();
    writeInProgress = false;
    if (value == FC_REGISTER_WRITE) {
      displayMessage(writeOkMessage);
    }
  } else {
    displayMessage(writeFailedMessage);
    writeInProgress = false;
  }
  startSampling = true;
}
#endif

/* ****************************************************************************************************************
                                                   MODBUS FUNCTIONS
*/

// ------------ Read the register  from Uinit Id and Register
// 0 - 6  MBAP Header 7-11 Modbus PDU
boolean  accessRegister(int fc, word tId, byte uId, word registerNumber, word data) {
  modbusRequest[0] = highByte(tId);                     // Transaction ID high byte
  modbusRequest[1] = lowByte(tId);                      // Transaction ID low byte
  modbusRequest[2] = 0;                                 // protocol high byte FIXED
  modbusRequest[3] = 0;                                 // protocol low byte
  modbusRequest[4] = 0;                                 // Lenght high byte
  modbusRequest[5] = 6;                                 // Lenght low byte FIXED
  modbusRequest[6] = uId;                               // unit ID
  modbusRequest[7] = fc;                                // function code FIXED
  modbusRequest[8] = highByte(registerNumber);          // Register to read
  modbusRequest[9] = lowByte(registerNumber);
  modbusRequest[10] = 0;
  modbusRequest[11] = 1;                                // Read count FIXED

  if (fc == FC_REGISTER_WRITE) {                                        // it is a write
    modbusRequest[10] = highByte(data);                  // data to write
    modbusRequest[11] = lowByte(data);
  }
  if (ModbusMaster.connect(slaveIp, MB_PORT)) {
    // single char write did not work with the CCGX, worked with everything else
    //
    ModbusMaster.write(modbusRequest, 12);                               // send the request
    int byteCounter = 0;
    modbusRequest[7] = 0;                                                 // response function code
    return true;
  } else {
#ifdef DEBUG
    Serial.println("connection with modbus slave failed");
#endif
    ModbusMaster.stop();
  }
  return false;
}


boolean waitForResponse() {
  // the wait is essential
  if (!ModbusMaster.available()) {
    if (!modbusWait) {
      timeoutModbusTimerId = timeoutModbusTimer.after(TIMEOUT_TIME, modbusTimeout);
      modbusWait = true;
    }
  } else {
    timeoutModbusTimer.stop(timeoutModbusTimerId);
    modbusWait = false;
    return true;
  }
  return false;
}

void modbusTimeout() {
  modbusWait = false;
  timeoutModbusTimer.stop(timeoutModbusTimerId);
  modbusTimeOut = true;
}

word readResponse() {
  int byteCounter = 0;
  modbusRequest[7] = 0;                                                 // response function code
  while (ModbusMaster.available()) {                                    // if data available
    modbusRequest[byteCounter] = ModbusMaster.read();                   // read the response
    byteCounter++;
    if (byteCounter > 10)  {                                            // We expect 10 bytes
      ModbusMaster.stop();
      if (writeInProgress) {
        return modbusRequest[7];
      }
      if (modbusRequest[7] == FC_REGISTER_READ) {                                      // Errors have function codes >= 0x80
        word result = (modbusRequest[9] * 0x100) + modbusRequest[10];   // Get the result vlue
        return result;
      } else
        return -1;                                                      // needs better error handling
    }
  }
  return -1;
}



