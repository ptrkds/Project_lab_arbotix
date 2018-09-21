
#include <Commander.h>
#include <ax12.h>
#include <BioloidController.h>

void setup() {

  //open serial port
   Serial.begin(9600);
   delay (500);   
   Serial.println("###########################");    
   Serial.println("Serial Communication Established.");  
  
  // set everything to center
    Serial.println("###########################");
    Serial.println("Set all servo motor to center (512).");
    for (int i = 1; i <= 5; i++){
      SetPosition(i, 512);
      delay(100);
    } 

    Serial.println("###########################");
    Serial.println("Relax all servo motor.");  
    for (int i = 1; i <= 5; i++){
      Relax(i); 
    }
    
}

void loop() {
  
  // read the servo motor number
    Serial.println("###########################");
    Serial.println("Which servo motor?"); 
    
    while(!Serial.available()){} // wait til the serial input
    char servoInByte = Serial.read();
    int servoNumber = servoInByte - '0';
    
    unsigned int positionValue=0;  // Max value is 65535
    char incomingByte;
    
  // read position value
    Serial.println("###########################");
    Serial.println("Which position?");
    
  
    while(!Serial.available()){}
    delay(100);
    if (Serial.available() > 0) {   // something came across serial
      positionValue = 0;         // throw away previous integerValue
      while(Serial.available() > 0) {
        incomingByte = Serial.read();
        
        // Serial.println(incomingByte);  // just for test
    
        positionValue *= 10;  // shift left 1 decimal place
        
        // convert ASCII to integer, add, and shift left 1 decimal place
        positionValue = ( (incomingByte - '0') + positionValue);
      }
      
      SetPosition(servoNumber,positionValue);
      Relax(servoNumber); 
      
      Serial.print("Servo");
      Serial.print(servoNumber);
      Serial.print(" in Position: ");
      Serial.println(positionValue);
    }
}
