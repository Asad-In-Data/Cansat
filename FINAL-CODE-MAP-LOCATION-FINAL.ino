#include <NMEAGPS.h>
#include <HardwareSerial.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Sensor objects
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
float sealevel;
// GPS objects
NMEAGPS gps;
gps_fix fix;
HardwareSerial gpsSerial(2); // UART2: RX=16, TX=17

// SD card chip select (use correct pin for your module)
#define SD_CS 5
File kmlFile;
unsigned long lastSaveTime = 0;
int fileIndex = 0; // counter for filenames

/*----------------------------------WIFI Starts-------------------------------------------*/
uint8_t broadcastAddress[] = {0x6C, 0xC8, 0x40, 0x44, 0x94, 0xB8}; //6C:C8:40:44:94:B8
//struct for RF Data
typedef struct struct_message {
  float Temperature;
  float Altitude;
  float Pressure;
  float Heading;
  float Xacc;
  float Yacc;
  float Zacc;
  float Angaccx;
  float Angaccy;
  float Angaccz;
  float Magx;
  float Magy;
  float Magz;
  float Sat;
  float Lat;
  float Long;
  float GPSAlt;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

wifi_interface_t current_wifi_interface;
/*----------------------------------WIFI Ends-------------------------------------------*/

/*--------------------------------SD CARD Functions-----------------------------------------*/
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, String message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    // Success message commented out to reduce serial clutter
    // Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

/*--------------------------------HMC5883L Functions-----------------------------------------*/
void displaySensorDetails(void) {
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print ("Sensor: "); Serial.println(sensor.name);
  Serial.print ("Driver Ver: "); Serial.println(sensor.version);
  Serial.print ("Unique ID: "); Serial.println(sensor.sensor_id);
  Serial.print ("Max Value: "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print ("Min Value: "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print ("Resolution: "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/*--------------------------------KML File Functions-----------------------------------------*/
// Function to start a new KML file
void startNewKML() {
  if (kmlFile) {
    // close previous file with footer
    kmlFile.println("</coordinates>");
    kmlFile.println("</LineString>");
    kmlFile.println("</Placemark>");
    kmlFile.println("</Document>");
    kmlFile.println("</kml>");
    kmlFile.flush();
    kmlFile.close();
  }

  // create a new file with unique name
  String filename = "/gps_" + String(fileIndex++) + ".kml";
  kmlFile = SD.open(filename.c_str(), FILE_WRITE);
  
  if (kmlFile) {
    Serial.print("New file created: ");
    Serial.println(filename);
    
    // Write KML header
    kmlFile.println("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
    kmlFile.println("<kml xmlns=\"http://www.opengis.net/kml/2.2\">");
    kmlFile.println("<Document>");
    kmlFile.println("<name>GPS Path</name>");
    kmlFile.println("<Placemark>");
    kmlFile.println("<LineString>");
    kmlFile.println("<tessellate>1</tessellate>");
    kmlFile.println("<coordinates>");
    kmlFile.flush();
  } else {
    Serial.println("Failed to create new KML file!");
  }
}

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // Initialize GPS serial

  // BMP180 configuration
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  sealevel = bmp.readPressure();
  Serial.println("Sea level pressure: ");
  Serial.println(sealevel);

  // SD card configuration
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  
  writeFile(SD, "/sensor_data.txt", "Xacc,Yacc,Zacc,Angaccx,Angaccy,Angaccz,Magx,Magy,Magz,Temperature,Altitude,Heading,Sat,Lat,Long,GPSAlt\n");
  
  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

  // MPU6050 configuration
  Serial.println("Adafruit MPU6050 test!");
  
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
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
  
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
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

  // HMC5883 configuration
  /* Initialise the sensor */
  if (!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  Serial.println("");

  // Wifi configuration
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_protocol(current_wifi_interface, WIFI_PROTOCOL_LR) != ESP_OK) {
    Serial.println("Error initializing WIFI LR");
    return;
  }
  
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for Send CB to get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Buzzer configuration
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(100);

  // Start first KML file for GPS data
  startNewKML();
  lastSaveTime = millis();
}

void loop() {
  // mpu6050 gets data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  myData.Xacc = a.acceleration.x;
  myData.Yacc = a.acceleration.y;
  myData.Zacc = a.acceleration.z;
  myData.Angaccx = g.gyro.x;
  myData.Angaccy = g.gyro.y;
  myData.Angaccz = g.gyro.z;
  delay(100);

  // bmp gets data
  myData.Temperature = bmp.readTemperature();
  myData.Pressure = bmp.readPressure();
  myData.Altitude = bmp.readAltitude(sealevel);

  // HMC5883 works
  sensors_event_t event;
  mag.getEvent(&event);
  myData.Magx = event.magnetic.x;
  myData.Magy = event.magnetic.y;
  myData.Magz = event.magnetic.z;

  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0;
  heading += declinationAngle;
  
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  
  myData.Heading = heading * 180 / M_PI;
  delay(100);

  // GPS get data
  bool newGPSData = false;
  while (gps.available(gpsSerial)) {
    fix = gps.read();
    newGPSData = true;
    
    if (fix.valid.location) {
      myData.Lat = fix.latitude();
      myData.Long = fix.longitude();
    }
    
    if (fix.valid.altitude) {
      myData.GPSAlt = fix.altitude();
    }
    
    if (fix.valid.satellites) {
      myData.Sat = fix.satellites;
    }
    
    // Save GPS data to KML file
    if (kmlFile && fix.valid.location) {
      // Save coordinates: longitude,latitude,altitude
      kmlFile.print(fix.longitude(), 6);
      kmlFile.print(",");
      kmlFile.print(fix.latitude(), 6);
      kmlFile.print(",");
      
      if (fix.valid.altitude)
        kmlFile.println(fix.altitude());
      else
        kmlFile.println("0");
        
      kmlFile.flush();
    }
  }

  // Writes sensor data to SD Card
  String sd_text = String(myData.Xacc) + "," + 
                  String(myData.Yacc) + "," + 
                  String(myData.Zacc) + "," + 
                  String(myData.Angaccx) + "," + 
                  String(myData.Angaccy) + "," + 
                  String(myData.Angaccz) + "," + 
                  String(myData.Magx) + "," + 
                  String(myData.Magy) + "," + 
                  String(myData.Magz) + "," + 
                  String(myData.Temperature) + "," + 
                  String(myData.Altitude) + "," + 
                  String(myData.Heading) + "," + 
                  String(myData.Sat) + "," + 
                  String(myData.Lat, 6) + "," + 
                  String(myData.Long, 6) + "," + 
                  String(myData.GPSAlt) + "\n";
                  
  appendFile(SD, "/sensor_data.txt", sd_text);

  // RF Send
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  // Display on Serial Monitor
  Serial.println("");
  Serial.print("Acceleration X: ");
  Serial.print(myData.Xacc);
  Serial.print(", Y: ");
  Serial.print(myData.Yacc);
  Serial.print(", Z: ");
  Serial.print(myData.Zacc);
  Serial.println(" m/s^2");
  
  Serial.print("Rotation X: ");
  Serial.print(myData.Angaccx);
  Serial.print(", Y: ");
  Serial.print(myData.Angaccy);
  Serial.print(", Z: ");
  Serial.print(myData.Angaccz);
  Serial.println(" rad/s");
  
  Serial.print("Temperature = ");
  Serial.print(myData.Temperature);
  Serial.println(" *C");
  
  Serial.print("Pressure = ");
  Serial.print(myData.Pressure);
  Serial.println(" Pa");
  
  Serial.print("Altitude = ");
  Serial.print(myData.Altitude);
  Serial.println(" meters");
  
  Serial.print("Magnetic X: ");
  Serial.print(myData.Magx);
  Serial.print(" ");
  Serial.print("Magnetic Y: ");
  Serial.print(myData.Magy);
  Serial.print(" ");
  Serial.print("Magnetic Z: ");
  Serial.print(myData.Magz);
  Serial.print(" ");
  Serial.println("uT");
  
  Serial.print("Heading: ");
  Serial.print(myData.Heading);
  Serial.println(" degrees");
  
  Serial.print("Sat No: ");
  Serial.print(myData.Sat);
  Serial.print(" Latitude : ");
  Serial.print(myData.Lat, 6);
  Serial.print(" Longitude: ");
  Serial.print(myData.Long, 6);
  Serial.print(" GPS Altitude: ");
  Serial.println(myData.GPSAlt);
  Serial.println(" ");

  // Buzzer decision
  if (myData.Altitude <= 1) // THIS SHOULD BE CHANGED BASED ON THE LOCATION
  {
    digitalWrite(4, LOW);
    Serial.println("Buzzer On");
  } else {
    digitalWrite(4, HIGH);
    Serial.println("Buzzer Off");
  }

  // Check if 2 minutes have passed to create a new KML file
  if (millis() - lastSaveTime >= 120000) { // 120000 ms = 2 minutes
    startNewKML();
    lastSaveTime = millis();
  }

  delay(1000); // Delay between readings
}