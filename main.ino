// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Math.h>
#include <TheThingsNetwork.h>
#include <MPU6050.h>

Adafruit_MPU6050 mpu;
double roulis=0;
double tangage=0; 
double pivot=0;

/*Activer ou non
  * 0=sleep
  * 1=desactive alarme
  * 3=arme
  */
int sleep=3;


int buzzPin =  6;    //Connect Buzzer on Digital Pin3

// Set your AppEUI and AppKey
const char *appEui = "70B3D57ED00431B6";
const char *appKey = "5420EFF9042C5DB4FDC9B093504DCE8F";


#define loraSerial Serial1
#define freqPlan TTN_FP_EU868

TheThingsNetwork ttn(loraSerial, Serial, freqPlan);



//convertisseur m/s²
float ConvertData(float data){

  data = abs(data);     
  data *= 9.80665; // m/s
  data *= 3.6;  // km/h
  return data;
}



//Procedure convertion
void ComputeAngle(int gyrox, int gyroy, int gyroz,  double roulis, double tangage, double pivot)
{
  double x = gyrox;
  double y = gyroy;
  double z = gyroz;

  roulis = atan(x/sqrt((y*y) + (z*z))); //  pitch 
  tangage = atan(y/sqrt((x*x) + (z*z))); // roll
  pivot = atan(z/sqrt((x*x) + (y*y))); // pitch
  
  //Conversion Radian en degrée
  roulis = roulis * (180.0/M_PI);
  tangage = tangage * (180.0/M_PI) ;
  pivot = pivot * (180.0/M_PI) ;
}


void setup(void) {
  
  Serial.begin(9600);
  loraSerial.begin(57600);

  // Wait a maximum of 10s for Serial Monitor
  while (!Serial && millis() < 10000)
    ;

  // Set callback for incoming messages
  ttn.onMessage(message);
  
  /* Active ou non le mpu6050 */
  mpu.enableSleep(sleep);
  mpu.enableCycle(sleep);

  ttn.setClass(CLASS_C);
  
  Serial.println("-- STATUS");
  ttn.showStatus();

  Serial.println("-- JOIN");
  ttn.join(appEui, appKey);


  Serial.println("Antivol connecte");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find accelerometer MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("accelerometer MPU6050 Found!");



  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }



  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }


  //Bande passante gyroscope
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  //Init buzzer
  pinMode(buzzPin, OUTPUT); 

  Serial.println("");
  delay(1000);
}






void loop() {

  Serial.println("-- LOOP");

  Serial.println(sleep);

  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ComputeAngle(g.gyro.x, g.gyro.y, g.gyro.z,  roulis, tangage, pivot);

  float vitesse = ConvertData((float)g.gyro.x);

  
   float inclinaison = a.acceleration.x+10;






  //Activation desactivation des appareils
  if(sleep==3) //Arméééé
  {
     mpu.enableSleep(false);
     mpu.enableCycle(false);

     
    if((inclinaison<4) || (inclinaison>16) || (vitesse>8))
    {
      digitalWrite(buzzPin, HIGH);
      delay(100);
      digitalWrite(buzzPin, LOW);
    }
    else if((inclinaison>4) && (inclinaison<16))
    {
      digitalWrite(buzzPin, LOW);
    }

    // Prepare payload
    byte payload[2];
    payload[0] = inclinaison;
    payload[1] = vitesse;
  
    // Send it off
    ttn.sendBytes(payload, sizeof(payload));

    ttn.onMessage(message);

  }
  else if(sleep==1) //Uniquement le buzzer desactive pour voir vitesse en utilisant velo (option)
  {
     
     mpu.enableSleep(false);
     mpu.enableCycle(false);

     digitalWrite(buzzPin, LOW);
    

    // Prepare payload
    byte payload[2];
    payload[0] = 00;
    payload[1] = vitesse;
  
    // Send it off
    ttn.sendBytes(payload, sizeof(payload));

    ttn.onMessage(message);
  }
  else if (sleep==0) //Sommeil
  {
    mpu.enableSleep(true);
    mpu.enableCycle(true);
    digitalWrite(buzzPin, LOW);

    // Prepare payload
    byte payload[2];
    payload[0] = 00;
    payload[1] = 00;
  
    // Send it off
    ttn.sendBytes(payload, sizeof(payload));

    ttn.onMessage(message);
  }






  
  
  /* Print out the values */
  Serial.print("Gyroscope: ");
  Serial.print(inclinaison);
  Serial.println(" ( 0-10 a gauche 10-20 a droite )");
  Serial.print("Acceleration: ");
  Serial.print(vitesse);
  Serial.println(" km/h");
  
  Serial.println("");

  //Duty cycle
  delay(36000);
}

void message(const uint8_t *payload, size_t size, port_t port)
{
  Serial.println("-- MESSAGE");
  Serial.print("Received " + String(size) + " bytes on port " + String(port) + ":");

  String recu="";
  
  /*
   * AA== egale 0
   * AQ== egale 1
   * Aw== egale 3
   */
  for (int i = 0; i < size; i++)
  {
    Serial.print(" " + String(payload[i]));
    recu+=String(payload[i]);
  }

  if(recu=="0")
  {
    sleep=0;
  }
  else if(recu=="3")
  {
    sleep=3;
  }
  else if(recu=="1")
  {
    sleep=1;
  }
  
  Serial.println();
}
