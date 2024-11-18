#include <Wire.h>
#include <MPU6050.h>

// MPU6050 object
MPU6050 mpu;

// Sensor data (using int16_t for raw values)
int16_t AccX_raw, AccY_raw, AccZ_raw;
int16_t GyroX_raw, GyroY_raw, GyroZ_raw;

// Floating point variables for angles
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float pitch, roll, yaw;
float yaw_rate = 0; // Yaw rate based on gyroZ

// Complementary filter alpha value
#define alpha 0.98

// Variables for gyro bias (drift correction)
float gyroBiasX = 0.0, gyroBiasY = 0.0, gyroBiasZ = 0.0;
int biasSamples = 1000;

unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Structure to hold the sensor data and angles
struct MPUData {
  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float roll, pitch, yaw;
};

void setup() {
  Serial.begin(115200);

  Wire.begin();
  mpu.initialize();

  // Check if MPU6050 is connected properly
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initial setup for sensor data
  lastTime = millis();

  // Calibrate gyro by averaging the first few readings to estimate the bias
  for (int i = 0; i < biasSamples; i++) {
    mpu.getRotation(&GyroX_raw, &GyroY_raw, &GyroZ_raw);
    gyroBiasX += GyroX_raw;
    gyroBiasY += GyroY_raw;
    gyroBiasZ += GyroZ_raw;
    delay(10); // Small delay between readings
  }

  // Calculate average bias values for the gyroscope
  gyroBiasX /= biasSamples;
  gyroBiasY /= biasSamples;
  gyroBiasZ /= biasSamples;
}

void loop() {
  // Call the function to get MPU6050 values
  MPUData data = readMPU6050();

  // Print the values to the serial monitor
  Serial.print("Roll: ");
  Serial.print(data.roll);
  Serial.print(" | Pitch: ");
  Serial.print(data.pitch);
  Serial.print(" | Yaw: ");
  Serial.println(data.yaw);

  delay(50); // Delay for readability
}

// Function to get sensor data (Accelerometer and Gyroscope)
MPUData readMPU6050() {
  MPUData data;

  // Get raw data from the MPU6050
  mpu.getAcceleration(&AccX_raw, &AccY_raw, &AccZ_raw);
  mpu.getRotation(&GyroX_raw, &GyroY_raw, &GyroZ_raw);

  // Convert raw accelerometer data to g's and gyroscope data to degrees/s
  data.AccX = AccX_raw / 16384.0;
  data.AccY = AccY_raw / 16384.0;
  data.AccZ = AccZ_raw / 16384.0;

  data.GyroX = (GyroX_raw - gyroBiasX) / 131.0;  // Subtract the gyro bias
  data.GyroY = (GyroY_raw - gyroBiasY) / 131.0;
  data.GyroZ = (GyroZ_raw - gyroBiasZ) / 131.0;

  // Calculate the angles (Pitch, Roll, Yaw)
  // Acceleration-based angles (Roll and Pitch)
  accAngleX = atan2(data.AccY, sqrt(data.AccX * data.AccX + data.AccZ * data.AccZ)) * 180.0 / PI;
  accAngleY = atan2(-data.AccX, data.AccZ) * 180.0 / PI;

  // Get the current time
  currentTime = millis();

  // Calculate the change in time (dt)
  float dt = (currentTime - lastTime) / 1000.0;

  // Integrate gyroscope data to get angle (pitch and roll)
  gyroAngleX += data.GyroX * dt;  // Gyroscope measures angular velocity, so multiply by dt to get the angle
  gyroAngleY += data.GyroY * dt;
  gyroAngleZ += data.GyroZ * dt;  // This will be used for yaw

  // Apply complementary filter to combine accelerometer and gyroscope data for pitch and roll
  data.pitch = alpha * (gyroAngleX) + (1 - alpha) * (accAngleX);
  data.roll = alpha * (gyroAngleY) + (1 - alpha) * (accAngleY);

  // For yaw, we only use the gyroscope (integration of GyroZ) as it's more accurate for rotation around Z-axis
  data.yaw = gyroAngleZ;

  // Optional: Normalize the yaw value to stay within -180 to 180 range for better usability
  if (data.yaw > 180) {
    data.yaw -= 360;
  } else if (data.yaw < -180) {
    data.yaw += 360;
  }

  // Update lastTime for the next loop
  lastTime = currentTime;

  return data;
}
