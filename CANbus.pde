/*CANbus PINs
update defaults.h in CANbus library to these values:

#define	P_MOSI	J,1
#define	P_MISO	J,0
#define	P_SCK	J,2
#define	MCP2515_CS	H,2 
#define	MCP2515_INT	J,6

Unknown:
#define LED2_HIGH			B,0
#define LED2_LOW			B,0

see 
http://arduino.cc/forum/index.php/topic,8730.0.html# 
http://tucrrc.utulsa.edu/Publications/Arduino/arduino.html
for more info and perhaps a different library
*/

// 65203 address - .05 L/hour trip fuel rate

#include <Canbus.h>
boolean InitCAN(){
  if(Canbus.init(CANSPEED_500)){
    Serial.println("CAN Init ok");
    return true;
  } else {
      Serial.println("Can't init CAN");
      return false;
  } 
}

void logCANbus(){
  char buffer[512];
  if(Canbus.ecu_req(ENGINE_RPM,buffer) == 1){         /* Request for engine RPM */
    Serial.println(buffer);                         /* Display data on LCD */
  } 
  if(Canbus.ecu_req(VEHICLE_SPEED,buffer) == 1){
    Serial.println(buffer);
  }
  if(Canbus.ecu_req(ENGINE_COOLANT_TEMP,buffer) == 1){
    Serial.println(buffer);   
  }
  if(Canbus.ecu_req(THROTTLE,buffer) == 1){
    Serial.println(buffer);
  }  
  if(Canbus.ecu_req(O2_VOLTAGE,buffer) == 1){
    Serial.println(buffer);
  }   
}

//Method Two using 
