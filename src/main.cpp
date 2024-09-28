#include <Arduino.h>
#include "m5StickCPlus2.h"

#define ESP32_CAN_TX_PIN GPIO_NUM_26
#define ESP32_CAN_RX_PIN GPIO_NUM_36

#include <NMEA2000.h>
#include <NMEA2000_esp32.h> 
#include <N2kMessages.h>
// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={127506L,127508L,127513L,0};

// ---  Example of using PROGMEM to hold Product ID.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const tNMEA2000::tProductInformation BatteryMonitorProductInformation PROGMEM={
                                       2100,                        // N2kVersion
                                       100,                         // Manufacturer's product code
                                       "Simple battery monitor",    // Manufacturer's Model ID
                                       "1.2.0.16 (2022-10-01)",     // Manufacturer's Software version code
                                       "1.2.0.0 (2022-10-01)",      // Manufacturer's Model version
                                       "00000001",                  // Manufacturer's Model serial code
                                       0,                           // SertificationLevel
                                       1                            // LoadEquivalency
                                      };                                      

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char BatteryMonitorManufacturerInformation [] PROGMEM = "John Doe, john.doe@unknown.com"; 
const char BatteryMonitorInstallationDescription1 [] PROGMEM = "Just for sample"; 
const char BatteryMonitorInstallationDescription2 [] PROGMEM = "No real information send to bus"; 

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler DCBatStatusScheduler(false,1500,500);
tN2kSyncScheduler DCStatusScheduler(false,1500,510);
tN2kSyncScheduler BatConfScheduler(false,5000,520); // Non periodic

tNMEA2000_esp32 NMEA2000;


void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

void setup() {

    // M5 Setup screen 

    auto cfg = M5.config();
    StickCP2.begin(cfg);
    StickCP2.Display.setRotation(1);
    StickCP2.Display.setTextColor(GREEN);
    StickCP2.Display.setTextDatum(middle_center);
    StickCP2.Display.setTextFont(&fonts::Orbitron_Light_24);
    StickCP2.Display.setTextSize(1);
    // NMEA2000 Config



                                
  // Uncomment 3 rows below to see, what device will send to bus                           
  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.
 // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  // NMEA2000.EnableForward(false);                      // Disable all msg forwarding to USB (=Serial)
  
//  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

}

int delta = 0;
void loop() {
    NMEA2000.ParseMessages();
    delta++;
    if(delta >= 10000){  
      Serial.print("*");
      delta = 0;
  }
}
  
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  unsigned char instance;
  double voltage;
  
  
  
    double current;
  double temperature;


  if (N2kMsg.PGN == 127508){
    if(ParseN2kDCBatStatus(N2kMsg, instance, voltage, current, temperature, SID)){
      StickCP2.Display.clear();
        StickCP2.Display.setCursor(10, 30);
        int vol = floor(voltage * 1000);
        StickCP2.Display.printf("BAT: %dmv", vol);
    }else{
      Serial.println("Malformed message");
    }
  }else{
    Serial.print("Rebut PGN "); Serial.println(N2kMsg.PGN);
  }
}