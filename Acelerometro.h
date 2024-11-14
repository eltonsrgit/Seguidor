#ifndef Acelerometro_H
#define Acelerometro_H

MPU6050 mpu6050(Wire);

long timer = 0;

void dadosMPU(){

  mpu6050.update();

  if(millis() - timer > 1000){
    
    BT.println("=======================================");
    //BT.print("temp : "); BT.println(mpu6050.getTemp());
    BT.print("Ax : "); BT.print(mpu6050.getAccX());
    BT.print(" / Ay : "); BT.print(mpu6050.getAccY());
    BT.print(" / Az : "); BT.println(mpu6050.getAccZ());
  
    BT.print("Gx : "); BT.print(mpu6050.getGyroX());
    BT.print(" / Gy : "); BT.print(mpu6050.getGyroY());
    BT.print(" / Gz : "); BT.println(mpu6050.getGyroZ());
  
    BT.print("AAX: "); BT.print(mpu6050.getAccAngleX());
    BT.print(" / AAY : "); BT.println(mpu6050.getAccAngleY());
  
    BT.print("GAngleX : "); BT.print(mpu6050.getGyroAngleX());
    BT.print(" / GAngleY : "); BT.print(mpu6050.getGyroAngleY());
    BT.print(" / GAngleZ : "); BT.println(mpu6050.getGyroAngleZ());
    
    /*BT.print("angleX : "); BT.print(mpu6050.getAngleX());
    BT.print("\tangleY : "); BT.print(mpu6050.getAngleY());
    BT.print("\tangleZ : "); BT.println(mpu6050.getAngleZ());*/
    BT.println("=======================================\n");

    timer = millis();
    
  }
}

#endif