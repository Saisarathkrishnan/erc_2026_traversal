#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void displayCalStatus() {
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  Serial.printf("1calibrate → SYS:%d G:%d A:%d M:%d\n", sys, gyro, accel, mag);
}

void setup() {
  
  Serial.begin(115200);
  delay(1000);
  Wire.begin(21, 22);

  if (!bno.begin()) {
    Serial.println("BNO055 not connected idiot");
    while (1)
      ;
  }

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_NDOF);


  Serial.println("\n1_360 no scope with sensor to calibrate");
}


  bool calib = false;



void loop() {



  uint8_t sys, gyro, accel, mag;
  if(!calib){
      displayCalStatus();
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  //Serial.print("hi");
  }

  if (gyro == 3 && mag == 3 && !calib) {
    Serial.println("\n1calibrated");

    adafruit_bno055_offsets_t offsets;
    bno.getSensorOffsets(offsets);

    //Serial.println("Saving calibration to flash...");
    bno.setSensorOffsets(offsets);

    Serial.println("1Calibration saved");
    calib = true;
  }

  if (calib) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Serial.print("2 ");
    Serial.print(euler.x(), 2);
    Serial.print("_");
    Serial.print(euler.y(), 2);
    Serial.print("_");
    Serial.print(euler.z(),2);
    Serial.println(";");
  }

  delay(10);
}
