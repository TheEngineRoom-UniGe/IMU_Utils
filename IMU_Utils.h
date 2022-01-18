#ifndef IMU_Utils_h
#define IMU_Utils_h

const uint8_t ADDRESSES[2] = {0x68 , 0x69};
const int MUX_ADDRESS = 0x6A;

struct s_module{
  MPU6050 sensor;
  bool dmpReady;

  Quaternion q;
  VectorInt16 accel;
  VectorInt16 gyro;
  VectorInt16 mag;

  uint8_t fifoBuffer[48];
  uint16_t packetSize;

  uint8_t address;
  uint8_t mux;
  uint8_t channel;
  uint8_t id;
};

typedef union {
 float floatingPoint[4];
 byte binary[16];
} binary4Float;

typedef union{
  int8_t integerPoint;
  byte binary;
} binaryInt;
  
typedef union{
  int16_t integerPoint[3];
  byte binary[6];
} binary3Int;


void tcaselect(uint8_t i,uint8_t mux_address) {
  if (i > 7) return;
  Wire.beginTransmission(mux_address);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void sendData(s_module imu, udp_data ROS_data){
  binaryInt ID;
  ID.integerPoint = imu.id;
  WiFiUDP Udp;
  binary4Float quaternion;
  quaternion.floatingPoint[0] = imu.q.x;
  quaternion.floatingPoint[1] = imu.q.y;
  quaternion.floatingPoint[2] = imu.q.z;
  quaternion.floatingPoint[3] = imu.q.w;

  binary3Int accelerometer;
  accelerometer.integerPoint[0] = imu.accel.x;
  accelerometer.integerPoint[1] = imu.accel.y;
  accelerometer.integerPoint[2] = imu.accel.z;

  binary3Int gyroscope;
  gyroscope.integerPoint[0] = imu.gyro.x;
  gyroscope.integerPoint[1] = imu.gyro.y;
  gyroscope.integerPoint[2] = imu.gyro.z;

  binary3Int magnetometer;
  magnetometer.integerPoint[0] = imu.mag.x;
  magnetometer.integerPoint[1] = imu.mag.y;
  magnetometer.integerPoint[2] = imu.mag.z;

  byte packetBuffer[29+6];
  memset(packetBuffer, 0, 29+6);

  packetBuffer[28+6] = ID.binary;
  
  for(int i=0; i<16; i++){
    packetBuffer[i] = quaternion.binary[i];
  }
  
  for(int i=16; i<22; i++){
    packetBuffer[i] = accelerometer.binary[i-16];
  }

  for(int i=22; i<28; i++){
    packetBuffer[i] = gyroscope.binary[i-22];
  }

  for(int i=28; i<34; i++){
    packetBuffer[i] = magnetometer.binary[i-28];
  }  
  
  int begin = Udp.beginPacket(ROS_data.server, ROS_data.localPort);
  int write = Udp.write(packetBuffer, 29+6);
  int end = Udp.endPacket();  
}

bool checkAddress(uint8_t address){
  delay(1000); 
  Wire.beginTransmission(address);
  byte error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("I2C device found at address 0x");
    if (address<16) {
      Serial.print("0");
    }
    Serial.println(address,HEX);
    return true;
  }else if (error==4) {
    Serial.print("Unknow error at address 0x");
    if (address<16) {
      Serial.print("0");
    }
    Serial.println(address,HEX);
    return false;
  }    
  return false;
}

int countIMUs(){
  int counter=0;
  for (int j = 0; j <2; j++){
    for (int i =0 ; i <4 ; i ++){
      Serial.print("MUX ");  Serial.print(j); Serial.print(" Channel ");  Serial.println(i); 
      tcaselect (i,MUX_ADDRESS+j);
      for(int k=0; k<2;k++){
        if(checkAddress(ADDRESSES[k])){
          counter++;
        }
      }
      tcaselect (5,MUX_ADDRESS+j); // Since two Mux are on the same i2c, you should disable the first mux, before enabling the other
    }
  }
  return counter;
}


void findIMUs(s_module s_arrays[]){
  int counter = 0, devStatus=-1;
  s_arrays[0].packetSize = 0;
  for (int j = 0; j <2; j++){
    for (int i =0 ; i <4 ; i ++){
      tcaselect (i,MUX_ADDRESS+j);
      for(int k=0; k<2;k++){
        if(checkAddress(ADDRESSES[k])){
          s_arrays[counter].address = ADDRESSES[k];
          s_arrays[counter].mux = MUX_ADDRESS+j;
          s_arrays[counter].channel = i;      
          s_arrays[counter].id = j*100+i*10+k;    

          s_arrays[counter].sensor = MPU6050(ADDRESSES[k]);
          s_arrays[counter].sensor.initialize();
          bool cnctd = s_arrays[counter].sensor.testConnection();
  
          Serial.print("connecting to address " );Serial.print(s_arrays[counter].address);
          Serial.print(" MUX " ); Serial.print (s_arrays[counter].mux);
          Serial.print(" Channel " ); Serial.println (s_arrays[counter].channel);
          Serial.println(cnctd ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
          
          if (cnctd){
            s_arrays[counter].sensor.resetFIFO();
          }

          devStatus= s_arrays[counter].sensor.dmpInitialize();
          Serial.println ("initializing MPU");
          Serial.println(devStatus);
          if (devStatus == 0 ){
            Serial.println ("MPU initialized");
            s_arrays[counter].sensor.PrintActiveOffsets();
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            s_arrays[counter].sensor.setDMPEnabled(true);
         
            //set our DMP Ready flag so the main loop() function knows it's okay to use it
            Serial.println(F("DMP ready!"));
            s_arrays[counter].dmpReady = true;
            s_arrays[counter].packetSize = s_arrays[counter].sensor.dmpGetFIFOPacketSize();
            uint16_t fifoCount = s_arrays[counter].sensor.getFIFOCount();
            Serial.print ("fifocount"); Serial.println(fifoCount);
            Serial.print("PacketSize");Serial.println(s_arrays[counter].packetSize); 
          }else{
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
          }    
          counter++;
        }
      }
      tcaselect (5,MUX_ADDRESS+j); // Since two Mux are on the same i2c, you should disable the first mux, before enabling the other
    }
  }
}
#endif