#include <Arduino.h>
#include "M5StickCPlus2.h"

#define ESP32_CAN_TX_PIN GPIO_NUM_26
#define ESP32_CAN_RX_PIN GPIO_NUM_36

#define EVO_ADDRESS 204

#include <NMEA2000.h>
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include "N2kDeviceList.h"
#include "pgnsToString.h"

// ---  Example of using PROGMEM to hold Configuration information.  However, doing this will prevent any updating of
//      these details outside of recompiling the program.
const char BatteryMonitorManufacturerInformation[] PROGMEM = "John Doe, john.doe@unknown.com";
const char BatteryMonitorInstallationDescription1[] PROGMEM = "Just for sample";
const char BatteryMonitorInstallationDescription2[] PROGMEM = "No real information send to bus";

// Define schedulers for messages. Define schedulers here disabled. Schedulers will be enabled
// on OnN2kOpen so they will be synchronized with system.
// We use own scheduler for each message so that each can have different offset and period.
// Setup periods according PGN definition (see comments on IsDefaultSingleFrameMessage and
// IsDefaultFastPacketMessage) and message first start offsets. Use a bit different offset for
// each message so they will not be sent at same time.
tN2kSyncScheduler DCBatStatusScheduler(false, 1500, 500);
tN2kSyncScheduler DCStatusScheduler(false, 1500, 510);
tN2kSyncScheduler BatConfScheduler(false, 5000, 520); // Non periodic

tNMEA2000_esp32 NMEA2000;

tN2kDeviceList *pN2kDeviceList;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

//*****************************************************************************
void PrintUlongList(const char *prefix, const unsigned long *List)
{
  uint8_t i;
  if (List != 0)
  {
    Serial.print(prefix);
    for (i = 0; List[i] != 0; i++)
    {
      if (i > 0)
        Serial.print(", ");
      Serial.print(List[i]);
    }
    Serial.println();
  }
}

//*****************************************************************************
void PrintText(const char *Text, bool AddLineFeed = true)
{
  if (Text != 0)
    Serial.print(Text);
  if (AddLineFeed)
    Serial.println();
}

//*****************************************************************************
void PrintDevice(const tNMEA2000::tDevice *pDevice)
{
  if (pDevice == 0)
    return;

  Serial.println("----------------------------------------------------------------------");
  Serial.println(pDevice->GetModelID());
  Serial.print("  Source: ");
  Serial.println(pDevice->GetSource());
  Serial.print("  Manufacturer code:        ");
  Serial.println(pDevice->GetManufacturerCode());
  Serial.print("  Unique number:            ");
  Serial.println(pDevice->GetUniqueNumber());
  Serial.print("  Software version:         ");
  Serial.println(pDevice->GetSwCode());
  Serial.print("  Model version:            ");
  Serial.println(pDevice->GetModelVersion());
  Serial.print("  Manufacturer Information: ");
  PrintText(pDevice->GetManufacturerInformation());
  Serial.print("  Installation description1: ");
  PrintText(pDevice->GetInstallationDescription1());
  Serial.print("  Installation description2: ");
  PrintText(pDevice->GetInstallationDescription2());
  PrintUlongList("  Transmit PGNs :", pDevice->GetTransmitPGNs());
  PrintUlongList("  Receive PGNs  :", pDevice->GetReceivePGNs());
  Serial.println();
}

#define START_DELAY_IN_S 8
//*****************************************************************************
void ListDevices(bool force = false)
{
  static bool StartDelayDone = false;
  static int StartDelayCount = 0;
  static unsigned long NextStartDelay = 0;
  if (!StartDelayDone)
  { // We let system first collect data to avoid printing all changes
    if (millis() > NextStartDelay)
    {
      if (StartDelayCount == 0)
      {
        Serial.print("Reading device information from bus ");
        NextStartDelay = millis();
      }
      Serial.print(".");
      NextStartDelay += 1000;
      StartDelayCount++;
      if (StartDelayCount > START_DELAY_IN_S)
      {
        StartDelayDone = true;
        Serial.println();
      }
    }
    return;
  }
  if (!force && !pN2kDeviceList->ReadResetIsListUpdated())
    return;

  Serial.println();
  Serial.print("Max devices ");
  Serial.println(N2kMaxBusDevices);
  Serial.println("**********************************************************************");
  for (uint8_t i = 0; i < N2kMaxBusDevices; i++)
  {

    PrintDevice(pN2kDeviceList->FindDeviceBySource(i));
  }
}
void sendHeadingMessage(){
  tN2kMsg N2kMsg =  tN2kMsg();
  tN2kOnOff RudderLimitExceeded = tN2kOnOff::N2kOnOff_Off;
  tN2kOnOff OffHeadingLimitExceeded = tN2kOnOff::N2kOnOff_Off;
  tN2kOnOff OffTrackLimitExceeded=  tN2kOnOff::N2kOnOff_Off;
  tN2kOnOff Override = tN2kOnOff::N2kOnOff_Off;
  tN2kSteeringMode SteeringMode = tN2kSteeringMode::N2kSM_HeadingControl;
  tN2kTurnMode TurnMode = tN2kTurnMode::N2kTM_RudderLimitControlled;
  tN2kHeadingReference HeadingReference = tN2kHeadingReference::N2khr_true;
  tN2kRudderDirectionOrder CommandedRudderDirection = tN2kRudderDirectionOrder::N2kRDO_Unavailable;
  double CommandedRudderAngle = 0.0;
  double HeadingToSteerCourse = PI / 2.0;
  double Track = 0.0;
  double RudderLimit = 30.0;
  double OffHeadingLimit = 10.0;
  double RadiusOfTurnOrder = 0.0;
  double RateOfTurnOrder = 0.0;
  double OffTrackLimit = 0.0;
  double VesselHeading = PI / 4.0 ;

  unsigned char SID = 1;


  SetN2kHeadingTrackControl(N2kMsg, RudderLimitExceeded,OffHeadingLimitExceeded,OffTrackLimitExceeded,Override,SteeringMode,TurnMode,
    HeadingReference,CommandedRudderDirection,CommandedRudderAngle,HeadingToSteerCourse,Track,RudderLimit,OffHeadingLimit,
    RadiusOfTurnOrder,RateOfTurnOrder,OffTrackLimit,VesselHeading);

    NMEA2000.SendMsg(N2kMsg);

}
void sendAutopilotMessage()
{
  tN2kMsg N2kMsg;

    N2kMsg.Destination = EVO_ADDRESS;
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=8;
    N2kMsg.AddByte(0x01); // Command
    N2kMsg.Add3ByteInt(65379L); // PGN commanded
    N2kMsg.AddByte(0xf8); // Priority + Reserved
    N2kMsg.AddByte(0x04); // 4 Params
    N2kMsg.AddByte(0x01); // Param 1 of 65379. Manufacturer
    N2kMsg.Add2ByteUInt(1851L); // Raymarine
    N2kMsg.AddByte(0x03); // Param 3 of 65379. Industry Code
    N2kMsg.AddByte(0x04); // Marine Industry
    N2kMsg.AddByte(0x04); // Param 4 of 65379. Mode
    N2kMsg.Add2ByteUInt(0x256); // wind
    N2kMsg.AddByte(0x05);
    N2kMsg.Add2ByteUInt(0x00); // Submode
    NMEA2000.SendMsg(N2kMsg);

    N2kMsg.Destination = EVO_ADDRESS;
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=8;
    N2kMsg.AddByte(0x01); // Command
    N2kMsg.Add3ByteInt(65360L); // PGN commanded
    N2kMsg.AddByte(0xf8); // Priority + Reserved
    N2kMsg.AddByte(0x05); // 4 Params
    N2kMsg.AddByte(0x01); // Param 1 of 65379. Manufacturer
    N2kMsg.Add2ByteUInt(1851L); // Raymarine
    N2kMsg.AddByte(0x03); // Param 3 of 65379. Industry Code
    N2kMsg.AddByte(0x04); // Marine Industry
    N2kMsg.AddByte(0x04); // Param 4 of 65379. SID
    N2kMsg.AddByte(1); // SID
    N2kMsg.AddByte(0x05); // Param 5 Target heading True
    N2kMsg.Add2ByteDouble(3.141592, 0.0001); // Target headingTrue
    N2kMsg.AddByte(0x06); // Param 5 Target heading True
    N2kMsg.Add2ByteDouble(3.12, 0.0001); // Target headingTrue

    N2kMsg.Add2ByteUInt(0x00); // Submode
    NMEA2000.SendMsg(N2kMsg);
}

void sendAutopilotRequest(){
  tN2kMsg N2kMsg;
  
  N2kMsg.Destination = EVO_ADDRESS;
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=8;
    N2kMsg.AddByte(0x00); // Request
    N2kMsg.Add3ByteInt(65379L); // PGN commanded
    N2kMsg.Add4ByteUInt(0L);  // Transmission interval
    N2kMsg.Add2ByteUInt(0L);  // Transmission Interval Offset
    N2kMsg.AddByte(0x02); // 2 Params
    N2kMsg.AddByte(0x01); // Param 1 of 65379. Manufacturer
    N2kMsg.Add2ByteUInt(1851L); // Raymarine
    N2kMsg.AddByte(0x03); // Param 3 of 65379. Industry Code
    N2kMsg.AddByte(0x04); // Marine Industry
    NMEA2000.SendMsg(N2kMsg);

N2kMsg.Destination = EVO_ADDRESS;
    N2kMsg.SetPGN(126208L);
    N2kMsg.Priority=8;
    N2kMsg.AddByte(0x00); // Request
    N2kMsg.Add3ByteInt(65360L); // PGN commanded
    N2kMsg.Add4ByteUInt(0L);  // Transmission interval
    N2kMsg.Add2ByteUInt(0L);  // Transmission Interval OffsetThe idea is NOT 
    N2kMsg.AddByte(0x02); // 2 Params
    N2kMsg.AddByte(0x01); // Param 1 of 65379. Manufacturer
    N2kMsg.Add2ByteUInt(1851L); // Raymarine
    N2kMsg.AddByte(0x04); // Marine Industry
    NMEA2000.SendMsg(N2kMsg);


  unsigned char SID = 1;
  double DistanceToWaypoint = 1.0;
  tN2kHeadingReference BearingReference = N2khr_magnetic;
  bool PerpendicularCrossed = false;
  bool ArrivalCircleEntered = false;
  tN2kDistanceCalculationType CalculationType = N2kdct_RhumbLine;
  double ETATime = 10000.0;
  int16_t ETADate = 100;
  double BearingOriginToDestinationWaypoint = 180.0;
  double BearingPositionToDestinationWaypoint = 170.0;
  uint32_t OriginWaypointNumber = 1;
  uint32_t DestinationWaypointNumber = 2;
  double DestinationLatitude = 40.11300458661758;
  double DestinationLongitude = 4.064127428738578;
  double WaypointClosingVelocity = 1.0;

  SetN2kNavigationInfo(N2kMsg, SID, DistanceToWaypoint, BearingReference,
                       PerpendicularCrossed, ArrivalCircleEntered, CalculationType,
                       ETATime, ETADate, BearingOriginToDestinationWaypoint, BearingPositionToDestinationWaypoint,
                       OriginWaypointNumber, DestinationWaypointNumber,
                       DestinationLatitude, DestinationLongitude, WaypointClosingVelocity);
  NMEA2000.SendMsg(N2kMsg);

  tN2kXTEMode XTEMode = N2kxtem_Autonomous;
  bool NavigationTerminated = false;
  double XTE = 1.0;

  SetN2kXTE(N2kMsg, SID, XTEMode, NavigationTerminated, XTE);
  NMEA2000.SendMsg(N2kMsg);
}


void setup()
{

  // M5 Setup screen

  auto cfg = M5.config();
  StickCP2.begin(cfg);
  StickCP2.Display.setRotation(1);
  StickCP2.Display.setTextColor(GREEN);
  StickCP2.Display.setTextDatum(middle_center);
  StickCP2.Display.setTextFont(&fonts::Orbitron_Light_24);
  StickCP2.Display.setTextSize(1);
  // NMEA2000 Config

  NMEA2000.SetProductInformation("00000003",                // Manufacturer's Model serial code
                                 100,                       // Manufacturer's product code
                                 "N2k bus device analyzer", // Manufacturer's Model ID
                                 "1.0.0.10 (2017-07-29)",   // Manufacturer's Software version code
                                 "1.0.0.0 (2017-07-12)"     // Manufacturer's Model version
  );

  NMEA2000.SetDeviceInformation(2,   // Unique number. Use e.g. Serial number.
                                130, // Device function=Display. See codes on https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                120, // Device class=Display. See codes on  https://web.archive.org/web/20190531120557/https://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on https://web.archive.org/web/20190529161431/http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );

  // Uncomment 3 rows below to see, what device will send to bus
  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);     // Show in clear text. Leave uncommented for default Actisense format.
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below

  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
                                                 // NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense);
  NMEA2000.EnableForward(false);                 // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, 50);

  //  NMEA2000.SetN2kCANMsgBufSize(2);                    // For this simple example, limit buffer size to 2, since we are only sending data
  // Define OnOpen call back. This will be called, when CAN is open and system starts address claiming.
  pN2kDeviceList = new tN2kDeviceList(&NMEA2000);
  // NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();
  Serial.println("NMEA2000 analyzer ready");
  NMEA2000.EnableForward(true);
  StickCP2.Display.clear();
  StickCP2.Display.setCursor(10, 30);

  StickCP2.Display.printf("Ready");
}

int delta = 0;
void loop()
{
  NMEA2000.ParseMessages();
  ListDevices();

  if (Serial.available() > 0)
  {
    int command = Serial.read();
    Serial.print("Received command : ");
    Serial.println(command);

    switch (command)
    {
    case 'L':
    case 'l':

      ListDevices(true);
      break;

    case 'T':
    case 't':

      NMEA2000.EnableForward(true);
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);

      StickCP2.Display.printf("Tracking");
      break;

    case 'S':
      sendAutopilotRequest();
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);
      StickCP2.Display.printf("Sent autopilot request");
      Serial.println("Sent autopilot request");
      break;

      case 'h':
      sendHeadingMessage();
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);
      StickCP2.Display.printf("Sent Heading Message");
      Serial.println("Sent Heading Message");
      break;

    case 's':
      sendAutopilotMessage();
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);
      StickCP2.Display.printf("Sent autopilot");
      Serial.println("Sent autopilot message");
      break;

    case 27:  // Escape

      NMEA2000.EnableForward(false);
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);

      StickCP2.Display.printf("NOT Tracking");
      break;
    }
  }
}

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  unsigned char SID;
  unsigned char instance;
  double voltage;

  double current;
  double temperature;
  Serial.print(".");
  if (N2kMsg.PGN == 127508 && false)
  {
    if (ParseN2kDCBatStatus(N2kMsg, instance, voltage, current, temperature, SID))
    {
      StickCP2.Display.clear();
      StickCP2.Display.setCursor(10, 30);
      int vol = floor(voltage * 1000);
      StickCP2.Display.printf("BAT: %dmv", vol);
    }
  }
  else
  {
    Serial.print("Received PGN ");
    Serial.print(N2kMsg.PGN);
    Serial.print(" ");
    Serial.println(toStringPgn(N2kMsg.PGN));
    StickCP2.Display.setCursor(10, 50);
    StickCP2.Display.printf("%d\n", N2kMsg.PGN);
    StickCP2.Display.printf("%s", toStringPgn(N2kMsg.PGN));
  }
}
