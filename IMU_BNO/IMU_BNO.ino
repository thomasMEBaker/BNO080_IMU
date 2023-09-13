
#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" 
BNO080 myIMU;

float quatI;
float quatJ; 
float quatK;
float quatReal;
float quatRadianAccuracy;
float x;
float y;
float z;

//Kalman Filter
float x_est_last = 0;
float P_last = 0;
//the noise in the system
float qKal = 0.004;
float rkal = 0.617;
float K;
float P;
float P_temp;
float x_temp_est;
float x_est;
float z_measured; //the 'noisy' value we measured
float z_real = 0.5; //the ideal value we wish to measure
float sum_error_kalman = 0;
float sum_error_measure = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Setup");

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    while (1);
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.calibrateAll(); //Turn on cal for Accel, Gyro, and Mag
  myIMU.enableGameRotationVector(100); //Send data update every 100ms
  myIMU.enableAccelerometer(100); //Send data update every 50ms
}

void loop()
{
  //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
      quatI = myIMU.getQuatI();
      quatJ = myIMU.getQuatJ();
      quatK = myIMU.getQuatK();
      quatReal = myIMU.getQuatReal();
      quatRadianAccuracy = myIMU.getQuatRadianAccuracy();

      x = myIMU.getAccelX();
      y = myIMU.getAccelY();
      z = myIMU.getAccelZ();

      plotAccelerometer();
  }
}

void printRotationVector(){
    Serial.print(quatI, 2);
    Serial.print(F(","));
    Serial.print(quatJ, 2);
    Serial.print(F(","));
    Serial.print(quatK, 2);
    Serial.print(F(","));
    Serial.print(quatReal, 2);
    Serial.print(F(","));
    Serial.print(quatRadianAccuracy, 2);
    Serial.print(F(","));
}

void printAccelerometer(){
    Serial.print(F(","));
    Serial.print(y, 2);
    Serial.print(F(","));
    Serial.print(z, 2);
    Serial.print(F(","));
}

void plotAccelerometer(){
  int upperLim = 20;
  int lowLim = -20;
  double kalmanFiltered = KalmanFilter(x);

  Serial.print(lowLim);
  Serial.print(F(","));
  Serial.print(upperLim);
  Serial.print(F(","));
  Serial.print(x, 2);
  Serial.print(F(","));
  Serial.println(kalmanFiltered);
}

double KalmanFilter(double inputVal){
		z_real = inputVal;
		//do a prediction
		x_temp_est = x_est_last;
		P_temp = P_last + qKal;
		//calculate the Kalman gain
		K = (P_temp * (1.0 / (P_temp + rkal)));
		//measure
    int randomNumber = random(101);
    float randomFloat = random(1000) / 1000.0;
    z_measured = z_real + randomFloat * 0.09;
		//correct
		x_est = x_temp_est + K * (z_measured - x_temp_est);
		P = (1 - K) * P_temp;
		sum_error_measure += fabs(z_real - z_measured);
		//update our last's
		P_last = P;
		x_est_last = x_est;

		return x_est;
}

