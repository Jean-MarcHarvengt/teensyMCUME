
#include <Wire.h>
//#include <i2c_t3.h>

void setup() {
  Serial.begin(115200);
  
  // put your setup code here, to run once:
  //Wire.setSCL(3);//19);
  //Wire.setSDA(4);//18);
  Wire.begin();

  //Wire2.setDefaultTimeout(200000); // 200ms  
  int status = Wire.requestFrom(8, 5);
  Serial.println(status);
  if(status == 5) {
    //i2cKeyboardPresent = true;
    Serial.println("i2C keyboard found");    
  } 
}

void loop() {
  // put your main code here, to run repeatedly:
    byte msg[5];
    Wire.requestFrom(8, 5);    // request 5 bytes from slave device #8 
    int i = 0;
    int hitindex=-1;
    while (Wire.available() && (i<5) ) { // slave may send less than requested
      byte b = Wire.read(); // receive a byte
      if (b != 0xff) hitindex=i; 
      msg[i++] = b;        
    }
    
    if (hitindex >=0 ) {
      
      Serial.println(msg[0], BIN);
      Serial.println(msg[1], BIN);
      Serial.println(msg[2], BIN);
      Serial.println(msg[3], BIN);
      Serial.println(msg[4], BIN);
      Serial.println("");
      Serial.println("");
      Serial.println("");
     }
     delay(20); 
}
