//Settings for this module:
byte address[] = {0x01, 0x01, 0x01};


//General Setting & Application Data:
boolean isMaster = false;
boolean isListening = false; 
boolean isSending = false;
boolean hearedIdOnBus = false;


int clkPin = 10; //Clock Pin. Output.
int incomingCLKPin = 18; // Clock Pin IN. Triggers an Interrupt.
int masterPin = 11; //Pull this HIGH in order to enable "MASTER" Mode. Only one MASTER is allowed on a Bus
int listeningPin = 12; //Listening? Outputs a Voltage if device is in listening mode.
int idOnBusPin = 13; //Heared its ID on Bus? Outputs a Voltage if a device heared its ID as a "TargetID" on the Bus

boolean messageStarted = false; // Tells if a message has been started to listen to
int messagePosition = 0; // Counts the messagePosition of the current byte
int defaultMessageLength = 10; //Default value for a message Length is 10 
int messageLength = defaultMessageLength; //Default is 10.
byte receivedBytes[256]; // Store receiving bytes

int senderAddressPosition = 0; //Counter for SenderAdressBytes
byte senderAddress[3]; //Sender-Address of an incoming package
int receiverAddressPosition = 0; //Counter for ReceiverAdressBytes
byte receiverAddress[3]; //Receiver-Address of an incoming package

int packageType = 0; //Type of Received Package

int priority = 0; //Priority of the package
boolean addressIsPhysical = false; //Type of the Address. If TRUE its physical, FALSE if group-address.
boolean expectedAnswer = false; //Is a answer expected?

void setup() {
  Serial.begin(9600); //DEBUG PURPOSES
  //Setting up the PORTB & PORTD Registers:
  DDRD = B11111111; //Setting all Ports on Register D to Input
  DDRB = B0110100; //Setting D8 and D9 to IN, since there is nothing attached to. D10 is OUT (CLK), D11 is IN (Master?), D12 is OUT (Listening LED), D13 is OUT (ID heared on BUS)
  isMaster = digitalRead(masterPin); // Reads if this device is set as Master or not.
  attachInterrupt(digitalPinToInterrupt(incomingCLKPin), receivePacket, RISING); // Attaching a Interrupt to the incoming CLOCK Pin
}

void loop() {
  if(isListening) {
    digitalWrite(listeningPin, HIGH);
  } else {
    digitalWrite(listeningPin, LOW);
  }
  if(hearedIdOnBus) {
    digitalWrite(idOnBusPin, HIGH);
  }
  delayMicroseconds(10);
}

void receivePacket() {
   if(isSending) {
    return;
   }
   if(!messageStarted) {
    isListening = true;
    messageStarted = true;
    messageLength = defaultMessageLength;
    messagePosition = 0;
    
   }
   DDRD = B00000000; //TODO: May remove.
   receivedBytes[messagePosition] = PORTD; // Reading the Data from PORTD
   messagePosition++;
   decodeBytes();
}

//This function decodes the current byte instruction and sets variables if needed.
void decodeBytes() {
  int position = messageLength-1;
  byte lastByte  = receivedBytes[position];
  if(position == 2 || position == 3 || position == 4) { //Bytes of sender Address
    senderAddress[senderAddressPosition];
    senderAddressPosition++; 
  } else if(position == 5 || position == 6 || position == 7) { //Bytes of receiver Address
    receiverAddress[receiverAddressPosition];
    receiverAddressPosition++;

    //DEBUG:
    if(receiverAddressPosition == 3) {
      if(receiverAddress[0] == address[0] && receiverAddress[1] == address[1] && receiverAddress[2] == address[2]) {
        hearedIdOnBus = true;
      }
    }
    
  } else if(position == 8) { //PackageType of Package
    packageType == lastByte; 
  } else if(position == 9) { // Length of Message
    messageLength = lastByte;
  } else if(position == 10) { // Flags
    //TODO: TEST IF THIS WORKS
    addressIsPhysical = getBit(lastByte, 4); // Read Bit 4 from the Byte.
    expectedAnswer = getBit(lastByte, 5); // Read Bit 5 from the Byte.
    priority = (getBit(lastByte,0)+getBit(lastByte,1)+getBit(lastByte,2)); //Sets the priority based on Bits 1-3 of the lastByte
  } else {
    if(messageLength == messagePosition) {
      isListening = false;
      messageStarted = false;
      
      decodePackage();
      //Resetting important values
      messagePosition = 0;
      messageLength = 0;
    }
  }
}


//Decodes the package and its instructions
void decodePackage() {
  
}

void sendPacket(byte b) {
  if(isListening) { 
    return; 
  }
  isSending = true;
  DDRD = B11111111; // Setting PORTD to OUT Mode
  PORTD = b;
  delayMicroseconds(10); //Delaying for 10 µs
  digitalWrite(clkPin, HIGH);
  delayMicroseconds(100); //Delaying for 10 µs
  DDRD = B00000000; //Clearing OUTPUT
  digitalWrite(clkPin, LOW); //Disabling Clock.
}

//Extracts a bit p from a number 
int getBit(int number, int p) {
    return (((1 << 1) - 1) & (number >> (p - 1)));
}
