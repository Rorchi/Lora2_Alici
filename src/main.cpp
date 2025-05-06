#include <Arduino.h>
#include <LoRa_E220.h>
#include <Adafruit_GFX.h>

#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <math.h>
// Define the pins for the LoRa E220 module
#define M0 18
#define M1 19
#define AUX 32
#define CHANNEL 23


#define TFT_CS   15
#define TFT_DC   2
#define TFT_RST  4
#define TFT_LED  33

// 3D Çizim Değişkenleri
// Initialize the display
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// Initialize the LoRa module
LoRa_E220 e220ttl(&Serial2, AUX, M0, M1); // RX, TX, AUX, M0, M1

struct Data
{
  float temperature;
  float Accelx;
  float Accely;
  float Accelz;
  //x,y,z ekseni gyro
  float Gyrox;  
  float Gyroy;
  float Gyroz;
  
  // yükseklik ve basınç
  float altitude;
  float pressure;
};

typedef struct {
  float Gyrox, Gyroy, Gyroz;
} GyroData;
GyroData receivedData = {0, 0, 0};

// Vertex yapısı (3D nokta)
typedef struct {
  float x, y, z;
} Vec3;

// 2D projeksiyon için
typedef struct {
  int x, y;
} Vec2;

// Basit uçak modelinin noktaları (gövde + kanatlar + kuyruk)
#define NUM_VERTICES 100
Vec3 vertices[NUM_VERTICES] = {
  // Gövde
  
  {0, -10, 40}, {5, -10, 40}, {5, -15, 40}, {0, -15, 40},
  {25, 20, -20}, {30, 20, -20}, {30, 15, -20}, {25, 15, -20},
  
  //Uçağın Kuyruğu
  {15, 20, -20}, {40, 20, -20}, {35, 15, -15}, {10, 15, -15},
  {15, 21, -20}, {40, 21, -20}, {35, 16, -15}, {10, 16, -15},

   //Dikey Stabilazör
  {26, 29, -20}, {26, 21, -20}, {23, 16, -15},
  {28, 29, -20}, {28, 21,-20}, {25, 16,-15},

  //kanatlar
  //22,23,24,25
  {-10, 0, 0}, {28, 0, 0}, {35, 6, -10}, {-5, 6, -10}, 
  {-10, 1, 0}, {28, 1, 0}, {35, 7, -10}, {-5, 7, -20}
  //26,27,28,29
  

};

// Bağlantılar (line çizeceğiz)
int edges[][2] = {
  {0, 1}, {1, 2}, {2, 3}, {3, 0},
  {0, 4}, {1, 5}, {2, 6}, {3, 7},
  {4, 5}, {5, 6}, {6, 7}, {7, 4}, 


  {8, 9}, {9, 10}, {10, 11}, {11, 8},
  {8, 12}, {9, 13}, {10, 14}, {11, 15},
  {12, 13}, {13, 14}, {14, 15}, {15, 12},

  {16, 17}, {17, 18}, {18, 16}, 
  {19,20}, {20,21}, {21,19},
  {16,19}, {17,20}, {18,21},

{22, 23}, {23, 24}, {24,25},{25,22},
{25, 26}, {26, 27},{27, 28},{28, 29},
{26, 22}, {27, 23},{28, 24},{29, 25},

 
 
};

#define NUM_EDGES (sizeof(edges)/sizeof(edges[0]))

// Dönme fonksiyonları (Euler)
Vec3 rotate(Vec3 v, float roll, float pitch, float yaw) {
  float x = v.x, y = v.y, z = v.z;

  // Yaw (Z)
  float cosa = cos(yaw), sina = sin(yaw);
  float x1 = x * cosa - y * sina;
  float y1 = x * sina + y * cosa;
  x = x1; y = y1;

  // Pitch (X)
  cosa = cos(pitch); sina = sin(pitch);
  float y2 = y * cosa - z * sina;
  float z1 = y * sina + z * cosa;
  y = y2; z = z1;

  // Roll (Y)
  cosa = cos(roll); sina = sin(roll);
  float x2 = x * cosa + z * sina;
  float z2 = -x * sina + z * cosa;
  x = x2; z = z2;

  return {x, y, z};
}

// 3D -> 2D projeksiyon
Vec2 project(Vec3 v) {
  float scale = 3.0;
  int cx = 160, cy = 120; // ekran ortası
  return {(int)(cx + v.x * scale), (int)(cy - v.y * scale)};
}

void drawPlane(float roll, float pitch, float yaw) {
  Vec2 projected[NUM_VERTICES];

  for (int i = 0; i < NUM_VERTICES; i++) {
    Vec3 rotated = rotate(vertices[i], roll, pitch, yaw);
    projected[i] = project(rotated);
  }

  for (int i = 0; i < NUM_EDGES; i++) {
    Vec2 p1 = projected[edges[i][0]];
    Vec2 p2 = projected[edges[i][1]];
    tft.drawLine(p1.x, p1.y, p2.x, p2.y,ILI9341_WHITE);
    tft.fillRect(p1.x, p1.y, 2, 2, ILI9341_RED); // nokta çizimi
    
  }
}





void setup()
{
  Serial.begin(9600);
  delay(500);
  e220ttl.begin();
  tft.begin();

  pinMode(TFT_LED, OUTPUT);
  digitalWrite(TFT_LED, HIGH);
  tft.setRotation(1);

  // Print module information
  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  // Set the configuration for the receiver
  configuration.ADDH = 0; // Receiver address (high byte)
  configuration.ADDL = 2; // Receiver address (low byte)
  configuration.CHAN = CHANNEL;
  // Channel (must match the transmitter)
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_200_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_22;
  //
  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  // Save the configuration
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  c.close();

}

void loop()
{
  if (e220ttl.available() > 0)
  {
    ResponseStructContainer rc = e220ttl.receiveMessageRSSI(sizeof(Data));
    Data receivedData;
    memcpy(&receivedData, rc.data, sizeof(Data));
    Serial.println( "Sicaklik" + String(receivedData.temperature)+ "degC, Accelx: " + String(receivedData.Accelx)+ "m/s^2,  Accely: " + String(receivedData.Accely)+ "m/s^2,  Accelz: " + String(receivedData.Accelz) + "m/s^2 \n"+
    "Gyrox: " + String(receivedData.Gyrox) + "rad,  Gyroy: " + String(receivedData.Gyroy) +"rad,  Gyroz: " + String(receivedData.Gyroz)+"rad\n"
    "Basinç: "+ String(receivedData.pressure) +"Pa,  Yukseklik: " + String(receivedData.altitude)+"m\n");
    delay(500);

    

    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_WHITE);
    tft.setRotation(1);
    tft.setCursor(10, 10);
    tft.setTextSize(1);
    tft.println("Sicaklik:" +String(receivedData.temperature) + "C");

    tft.setTextColor(ILI9341_BLUE);
    tft.setRotation(1);
    tft.setCursor(10, 20);
    tft.setTextSize(1);
    tft.println("X ekseni ivme:" +String(receivedData.Accelx) + "m/s^2 ");
    

    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(10, 30);
    tft.setTextSize(1);
    tft.println("Y ekseni ivme:" +String(receivedData.Accely) + "m/s^2 ");
    

    tft.setTextColor(ILI9341_DARKGREY);
    tft.setCursor(10, 40);
    tft.setTextSize(1);
    tft.println("Z ekseni ivme:" +String(receivedData.Accelz) + "m/s^2 ");
  

    tft.setTextColor(ILI9341_RED);
    tft.setCursor(10, 50);
    tft.setTextSize(1);
    tft.println("X ekseni aci:" +String(receivedData.Gyrox) + "m/s^2 ");
   
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(10, 60);
    tft.setTextSize(1);
    tft.println("Y ekseni aci:" +String(receivedData.Gyroy) + "m/s^2 ");
    

    tft.setTextColor(ILI9341_ORANGE);
    tft.setCursor(10, 70);
    tft.setTextSize(1);
    tft.println("Z ekseni aci:" +String(receivedData.Gyroz) + "m/s^2 ");
   

    tft.setTextColor(ILI9341_MAGENTA);
    tft.setCursor(10, 80);
    tft.setTextSize(1);
    tft.println("Basinc:" +String(receivedData.pressure/1000) + "kPa");
    

    tft.setTextColor(ILI9341_CYAN);
    tft.setCursor(10, 90);
    tft.setTextSize(1);
    tft.println("Yukseklik:" + String(receivedData.altitude/1000) + "km");
    
    drawPlane(receivedData.Gyrox, receivedData.Gyroy, receivedData.Gyroz);
    delay(100);
 
  }
  else{
    Serial.println("Veri alinamadi");
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextColor(ILI9341_RED);
    tft.setRotation(1);
    tft.setCursor(10, 10);
    tft.setTextSize(2);
    tft.println("Veri \n Alinamadi");
    delay(500);
  }
  }
