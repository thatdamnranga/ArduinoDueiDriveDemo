/*
 * BMW iDrive Implementation for KCAN2 (KCAN-2) controller 9286699-02
 * 
 * For use with an Arduino Due and a CAN shield of some description attached to Can0
 * 
 * This is purely a demo/test sketch for a future project. 
 * I publish this because people like 'Mikrocontroller Projekte' 
 * on YouTube annoy me by producing pointless demo videos that help nobody!
 * 
 * 
 * It is not meant to be a complete implementation. The link below will assist you in decoding the button presses (and rotary encoder)
 * 
 * http://www.loopybunny.co.uk/CarPC/k_can.html
 * 
 * 
 * Requires CircularBuffer, DueTimer and due_can libraries.
 * All I/O is via the Native USB device.
 * 
 */

#include <due_can.h>
#include <CircularBuffer.h>
#include <DueTimer.h>

// Initialise our three important outbound frames. The controller won't work properly without them.
CAN_FRAME busAliveFrame, rotaryInitFrame, illumiStatusFrame;
uint8_t targetIllumiLevel;
bool busAliveStatus, encoderActiveStatus;
#define BUS_ALIVE_TX_INTERVAL 1000000 //Send once per second.
#define CAN_TX_INTERVAL 5000 //Maximum CAN TX frequency 200hz/5ms.

#define Serial SerialUSB // Use the native port not the progreamming port.

#define CIRCULAR_BUFFER_XS
#define CIRCULAR_BUFFER_INT_SAFE  //Given this is interrupt driven, we need our buffer to be interrupt safe 
CircularBuffer<CAN_FRAME,16> CAN_rxBuffer, CAN_txBuffer; //Two 16 message buffers to handle inbound and outbound frames.

//Define static pins
#define CANLED0 14

#define HeartbeatTimer Timer0
#define CANtxTimer Timer1
#define BusAliveTimer Timer2


// Start LED triggers
void HeartbeatLED()
{
  digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));
}

void CANactivity(int port){
  switch (port) {
    case 0:
      digitalWrite(CANLED0, !(digitalRead(CANLED0)));
      break;
    default:
      break;
  }
}
// END LED Triggers

void printFrame(int port, CAN_FRAME *frame) {
   Serial.print("Port: ");
   Serial.print(port);
   Serial.print(" ID: 0x");
   Serial.print(frame->id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame->length);
   Serial.print(" Data: 0x ");
   for (int count = 0; count < frame->length; count++) {
       Serial.print(frame->data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\n");
}

void checkFrame(CAN_FRAME frame){
  switch (frame.id){
    case 0x5E7:
      busAliveStatus = true;
      if (encoderActiveStatus == false){
        ActivateEncoder();
      }
      break;
    default:
      break;
  }
}

void BusAlive(){
  CAN_txBuffer.push(busAliveFrame);
}



void SetBacklight(uint8_t BacklightLevel){
  illumiStatusFrame.data.byte[0] = BacklightLevel;
  CAN_txBuffer.push(illumiStatusFrame);
  Serial.print("Set backlight level");
}

void ActivateEncoder(){
  CAN_txBuffer.push(rotaryInitFrame);
  encoderActiveStatus = true;
}




void CANtxFrame()
{
  if(!CAN_txBuffer.isEmpty()){
    CANactivity(0);
    while(!CAN_txBuffer.isEmpty()){
      CAN_FRAME frame = CAN_txBuffer.shift();
      Can0.sendFrame(frame);
      printFrame(0,&frame);
    }
    CANactivity(0);
  }
}

void CANrxFrame(CAN_FRAME *frame) 
{
  CANactivity(0);
  CAN_rxBuffer.push(*frame);
  CANactivity(0);
}


void setup(){
  // Define bus alive frame, 0x560 (0x0560)
  busAliveFrame.id = 0x560;
  busAliveFrame.extended = 0;
  busAliveFrame.length = 8;
  busAliveFrame.data.low = 0x00000000;
  busAliveFrame.data.high = 0x60002f57;

  // Define rotary initialisation frame, 0x273 (0x0273)
  rotaryInitFrame.id = 0x273;
  rotaryInitFrame.extended = 0;
  rotaryInitFrame.length = 8;
  rotaryInitFrame.data.low = 0xf000e11d;
  rotaryInitFrame.data.high = 0x04de7fff;

  // Set up the illumination status frame
  illumiStatusFrame.id = 0x202;
  illumiStatusFrame.extended = 0;
  illumiStatusFrame.length = 2;
  illumiStatusFrame.data.byte[0] = 0xfd;
  illumiStatusFrame.data.byte[1] = 0x0;

  busAliveStatus, encoderActiveStatus = false;
  pinMode(CANLED0, OUTPUT);
  digitalWrite(CANLED0, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(57600); //Start up serial interface at 1Mbps
  
  // Initialize CAN0 at 500K for KCAN2
  if(Can0.begin(CAN_BPS_500K)){
    digitalWrite(CANLED0, LOW);
  }

  Can0.setNumTXBoxes(4);
  Can0.watchFor();
  Can0.setGeneralCallback(CANrxFrame);

  //Initialise controller.


  HeartbeatTimer.attachInterrupt(HeartbeatLED).start(125000);
  CANtxTimer.attachInterrupt(CANtxFrame).start(CAN_TX_INTERVAL);
  BusAliveTimer.attachInterrupt(BusAlive).start(BUS_ALIVE_TX_INTERVAL);
  SetBacklight(0xfd);
}

void loop(){ 
  //Run through receive buffers and print output to serial port
  if(!CAN_rxBuffer.isEmpty()){
    while(!CAN_rxBuffer.isEmpty()){  
      CAN_FRAME frame = CAN_rxBuffer.shift();
      checkFrame(frame);
      printFrame(0,&frame);
    }
  } 
}
