//Settings for this module:
byte address[] = {0x01, 0x02, 0x03};
int pins[] = {3,4,5,6,7,8,9,10};
byte magicBeginning = 0x03;

//General Setting & Application Data:
boolean isMaster = false;
volatile boolean isListening = false; 
boolean isSending = false;
boolean hearedIdOnBus = false;
boolean isBroadcastPacket = false;
boolean isSerialEnabled = false;

boolean systemError = false;
boolean setupDone = false;

int offset = 50; // Delay the execution around this value in ms

int clkPin = A0; //Clock Pin. Output.
int incomingCLKPin = 2; // Clock Pin IN. Triggers an Interrupt.
int masterPin = A1; //Pull this HIGH in order to enable "MASTER" Mode. Only one MASTER is allowed on a Bus
int listeningPin = A2; //Listening? Outputs a Voltage if device is in listening mode.
int idOnBusPin = A3; //Heared its ID on Bus? Outputs a Voltage if a device heared its ID as a "TargetID" on the Bus
int broadcastPin = A4; // Lights up if the Message is a broadcast on the system bus
int rgbRedPin = 11;
int rgbGreenPin = 12;
int rgbBluePin = 13;

//Currently delivered Package Information (RECEIVE)

boolean messageStarted = false; // Tells if a message has been started to listen to
volatile int messagePosition = 0; // Counts the messagePosition of the current byte
int messageLength = 0; //Data Length of Package
volatile byte receivedBytes[256]; // Store receiving bytes

int senderAddressPosition = 0; //Counter for SenderAdressBytes
byte senderAddress[3]; //Sender-Address of an incoming package
int receiverAddressPosition = 0; //Counter for ReceiverAdressBytes
byte receiverAddress[3]; //Receiver-Address of an incoming package

byte packageType; //Type of Received Package

int priority = 0; //Priority of the package
boolean addressIsPhysical = false; //Type of the Address. If TRUE its physical, FALSE if group-address.
boolean expectedAnswer = false; //Is a answer expected?

byte sendBuffer[256];

byte buffer[250]; // Buffer for Serial Communication
int bufferLength = 0;

void setup() {
  Serial.begin(9600); //DEBUG PURPOSES
  //Setting up the PORTB & PORTD Registers:
  for(int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
  }
  DDRB = B0110100; //Setting D8 and D9 to IN, since there is nothing attached to. D10 is OUT (CLK), D11 is IN (Master?), D12 is OUT (Listening LED), D13 is OUT (ID heared on BUS)
  isMaster = digitalRead(masterPin); // Reads if this device is set as Master or not.
  pinMode(clkPin, OUTPUT);
  pinMode(masterPin, INPUT);
  pinMode(listeningPin, OUTPUT);
  pinMode(idOnBusPin, OUTPUT);
  pinMode(incomingCLKPin, INPUT_PULLUP);
  pinMode(broadcastPin, OUTPUT);
  pinMode(rgbRedPin, OUTPUT);
  pinMode(rgbGreenPin, OUTPUT);
  pinMode(rgbBluePin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(incomingCLKPin), receivePacket, RISING); // Attaching a Interrupt to the incoming CLOCK Pin
  setRgbLed();
  setupDone = true;
}



void stopListening() {
    isListening = false;
    hearedIdOnBus = false;
    messageStarted = false;
    messagePosition = 0;
    messageLength = 0;
}




//This function decodes the current byte instruction and sets variables if needed.
void decodeBytes() {
  byte lastByte  = receivedBytes[messagePosition];
  Serial.print("Position :"); Serial.println(messagePosition);
  if(messagePosition == 1) {
    if(lastByte != magicBeginning) {
      stopListening();
      return;
    }
  } if(messagePosition == 2 || messagePosition == 3 || messagePosition == 4) { //Bytes of sender Address
    senderAddress[senderAddressPosition] = lastByte;
    senderAddressPosition++; 
  } else if(messagePosition == 5 || messagePosition == 6 || messagePosition == 7) { //Bytes of receiver Address
    receiverAddress[receiverAddressPosition] = lastByte;
    receiverAddressPosition++;
    //DEBUG:
    if(receiverAddressPosition == 3) {
      if(receiverAddress[0] == address[0] && receiverAddress[1] == address[1] && receiverAddress[2] == address[2]) {
        hearedIdOnBus = true;
      }
    }
  } else if(messagePosition == 8) { //PackageType of Package
    packageType = lastByte; 
  } else if(messagePosition == 9) { // Length of Message
    messageLength = lastByte;
  } else if(messagePosition >= 10) { // Flags
    //TODO: TEST IF THIS WORKS
    if(messagePosition == 10) {
      addressIsPhysical = bitRead(lastByte, 4); // Read Bit 4 from the Byte.
      expectedAnswer = bitRead(lastByte, 5); // Read Bit 5 from the Byte.
      priority = (bitRead(lastByte,0)+bitRead(lastByte,1)+bitRead(lastByte,2)); //Sets the priority based on Bits 1-3 of the lastByte
      debugPackage();
    }
    if((messageLength+10) == messagePosition) { // This has to run if the package has been finished receiving
        stopListening();
    }
  }
}

void receivePacket() {
  isListening = true;
  if(!setupDone) {
    isListening = false;
    return;
  }
  byte data; 
  messagePosition++;
  for(int i = 0; i < 8; i++) {
    digitalWrite(pins[i], 0);
  }
  
  for(int i = 0; i < 8; i++) {
    pinMode(pins[i], INPUT);
  }
  
  for(int i = 0; i < 8; i++) {
    bitWrite(data, i, digitalRead(pins[i]));
  }
  
  receivedBytes[messagePosition] = data;
  decodeBytes();
}



void loop() {
  if(isListening) {
    analogWrite(listeningPin, 255);
  } else {
    analogWrite(listeningPin, 0);
  }
  if(isBroadcastPacket) {
    analogWrite(broadcastPin, 255);
  } else {
    analogWrite(broadcastPin, 0);
  }
  if(hearedIdOnBus) {
    analogWrite(idOnBusPin, 255);
  } else {
    analogWrite(idOnBusPin, 0);
  }
  if(Serial.available() > 0) {
    digitalWrite(rgbGreenPin, HIGH);
    byte incomingByte = 0;
    incomingByte = Serial.read();
    if(incomingByte != -1) {
     buffer[bufferLength] = incomingByte;
     bufferLength++;
     if(bufferLength == 2) {
      handleSerialCommand(buffer);
      bufferLength = 0; 
      }
    }
  }
}

void handleSerialCommand(byte data[]) {
  if(!isSerialEnabled) {
    if(data[0] == 0xC1 && data[1] == 0x00) {
      isSerialEnabled = true;
    } else {
      return;
    }
  }
  if(data[0] == 0xAE && data[1] == 0x00) { // Requesting Address from device
    Serial.write(address, 3);
  } else if(data[0] == 0xF1) { // Setting Address[0]
    address[0] = data[1];
  } else if(data[0] == 0xF2) { // Setting Address[1]
    address[1] = data[1];
  } else if(data[0] == 0xF3) { // Setting Address[2]
    address[2] = data[1];
  } else if(data[0] == 0xF4) { // Setting delay (offset) for execution
    offset = data[1];
    Serial.write(0x01);
  } else if(data[0] == 0xA1) { // Send a Predefined Command:
    if(data[1] == 0x01) {
      sendMasterWakeup(); // Send Master Wakeup on 0xA1 0x01
      Serial.write(0x01);
    }
  } else if(data[0] == 0xA2) { // Send a Value given from Console
    sendByte(data[1]);
    Serial.write(0x01);
  } else if(data[0] == 0xC1) { // Serial Communication Controlling:
    if(data[1] == 0x00) {
      isSerialEnabled = true;
        
    } else if(data[1] == 0x01) {
      isSerialEnabled = false;
    }
  }
}





void sendByte(byte b) {
  if(isListening || systemError) { 
    return; 
  }
  isSending = true;
  for(int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
  }
  for(int i = 0; i < 8; i++) {
    digitalWrite(pins[i],bitRead(b, i));
  }
  //delayMicroseconds(10+offset); //Delaying for 10µs
  delay(1+offset);
  analogWrite(clkPin, 255);
  //delayMicroseconds(100+offset); //Delaying for 100 µs
  delay(1+offset);
  analogWrite(clkPin, 0); //Disabling Clock.
  for(int i = 0; i < 8; i++) {
    digitalWrite(pins[i], 0);
  }
  for(int i = 0; i < 8; i++) {
    pinMode(pins[i], INPUT);
  }
  delay(1+offset);
}

//Ignores the restriction of "systemError" Boolean
void sendErrorByte(byte b) {
  systemError = true;
  for(int i = 0; i < 8; i++) {
    pinMode(pins[i], OUTPUT);
  }
  for(int i = 0; i < 8; i++) {
    digitalWrite(pins[i],bitRead(b, i));
  }
}

void displayError(byte error) {
  systemError = true;
  sendErrorByte(error);
  while(true) {
   digitalWrite(rgbBluePin, LOW);
   digitalWrite(rgbRedPin,  HIGH);
   delay(700);
   digitalWrite(rgbRedPin,  LOW);
   digitalWrite(rgbBluePin, HIGH);
   delay(700);
  }
}

void setRgbLed() {
  if(isMaster) {
    digitalWrite(rgbBluePin, HIGH); // Displays MasterMode
    //sendMasterWakeup();
  } else {
    digitalWrite(rgbGreenPin, HIGH); // Displays SlaveMode
  }
}

//
//
//   PREPARED MESSAGES - READY TO SEND:
//
//


void sendKeepAlive() {
  if(isListening) {
    return;
  }
    byte data[] = {magicBeginning, address[0], address[1], address[2], 0x11,0x12,0x13, 0x14, 0x00, 0x00};
    int size = sizeof(data)/sizeof(byte);
    for(int i = 0; i < size; i++) {
      sendByte(data[i]);
    }
}

void sendMasterWakeup() {
    byte data[] = {0xFF, address[0], address[1], address[2], 0xFF,0xFF,0xFF, 0x15, 0x00, 0x00};
    int size = sizeof(data)/sizeof(byte);
    for(int i = 0; i < size; i++) {
      sendByte(data[i]);
    }
}

void sendSerialDisabled() {
    byte data[] = {0xFF, address[0], address[1], address[2], 0xFF,0xFF,0xFF, 0xA2, 0x00, 0x00};
    int size = sizeof(data)/sizeof(byte);
    for(int i = 0; i < size; i++) {
      sendByte(data[i]);
    }
}

void sendSerialEnabled() {
    byte data[] = {0xFF, address[0], address[1], address[2], 0xFF,0xFF,0xFF, 0xA1, 0x00, 0x00};
    int size = sizeof(data)/sizeof(byte);
    for(int i = 0; i < size; i++) {
      sendByte(data[i]);
    }
}

//
//
// DEBUG
//
//
//

void debugPackage() {
    Serial.println("### PACKAGE INFO ###");
    Serial.print("SEN-ADDR: ");
    Serial.print(senderAddress[0], HEX);
    Serial.print(senderAddress[1], HEX);
    Serial.print(senderAddress[2], HEX);
    Serial.println("");
    Serial.print("REC-ADDR: ");
    Serial.print(receiverAddress[0], HEX);
    Serial.print(receiverAddress[1], HEX);
    Serial.print(receiverAddress[2], HEX);
    Serial.println("");
    Serial.print("PKG-TYPE: ");
    Serial.print(packageType, HEX);
    Serial.println("");
    Serial.print("MSG-LENGTH: ");
    Serial.print(messageLength, HEX);
    Serial.println("");
    Serial.print("ADDR-PHY: "); Serial.print(addressIsPhysical, HEX); Serial.print(" | ");
    Serial.print("EXPCT-ANS: "); Serial.print(expectedAnswer, HEX); Serial.print(" | ");
    Serial.print("PRIORITY: "); Serial.print(priority, HEX); Serial.println();
}
