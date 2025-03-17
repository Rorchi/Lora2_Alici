#include <Arduino.h>
#include <LoRa_E220.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <TFT_eSPI.h> // ILI9341 için TFT kütüphanesi

// Define the display parameters
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Define the pins for the LoRa E220 module
#define M0 18
#define M1 19
#define AUX 15

#define CHANNEL 23

// TFT nesnesi oluştur
TFT_eSPI tft = TFT_eSPI();

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

// Uçak pozisyon ve açı değişkenleri
/*float planeX, planeY, planeAngle;
unsigned long lastUpdateTime = 0;

// Diğer tanımlamalar (LoRa pinleri vs.)...
void drawPlane(int x, int y, float angle) {
  int size = 15; // Uçak boyutu
  float cosA = cos(angle);
  float sinA = sin(angle);

  // Uçak noktaları (üçgen)
  int16_t points[3][2] = {
    {0, -size},    // Burun
    {-size, size},  // Sol kanat
    {size, size}    // Sağ kanat
  };

  // Noktaları döndür ve ekrana çiz
  for (int i = 0; i < 3; i++) {
    int16_t xRot = points[i][0] * cosA - points[i][1] * sinA;
    int16_t yRot = points[i][0] * sinA + points[i][1] * cosA;
    points[i][0] = x + xRot;
    points[i][1] = y + yRot;
  }

  // Üçgen çiz
  tft.drawLine(points[0][0], points[0][1], points[1][0], points[1][1], TFT_WHITE);
  tft.drawLine(points[1][0], points[1][1], points[2][0], points[2][1], TFT_WHITE);
  tft.drawLine(points[2][0], points[2][1], points[0][0], points[0][1], TFT_WHITE);
}
void updatePlane(Data data) {
  // Zaman farkını hesapla
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Saniye cinsinden
  lastUpdateTime = currentTime;

  // Açısal hızı açıya çevir (Gyro Z ekseni)
  planeAngle += data.Gyroz * deltaTime;

  // İvme verilerini pozisyona çevir (Accel X ve Y)
  float speed = 50.0; // Hız çarpanı
  planeX += data.Accelx * speed * deltaTime;
  planeY += data.Accely * speed * deltaTime;

  // Ekran sınırları
  planeX = constrain(planeX, 0, tft.width());
  planeY = constrain(planeY, 0, tft.height());

  // Ekranı temizle ve uçağı çiz
  tft.fillScreen(TFT_BLACK);
  drawPlane(planeX, planeY, planeAngle);
}*/

// Initialize the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



// Initialize the LoRa module
LoRa_E220 e220ttl(&Serial2, AUX, M0, M1); // RX, TX, AUX, M0, M1

void setup()
{
  Serial.begin(9600);
  delay(500);

  // Start the LoRa module
  e220ttl.begin();
  // Initialize the OLED display
  while(!e220ttl.begin()){
    Serial.println("LoRa Modul Baslatilamadi");
    delay(1000);
  }
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3C for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Halt if display initialization fails
  }

  /*tft.init();
  tft.setRotation(3); // Ekran yönünü ayarla
  tft.fillScreen(TFT_BLACK);
  planeX = tft.width() / 2;
  planeY = tft.height() / 2;

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Lora Alici\nBaslatildi");
  display.display();
  delay(3000);*/
 

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

      // Uçağı güncelle
     // updatePlane(receivedData);

    display.clearDisplay();
    display.setCursor(10, 10);
    display.setTextSize(2);
    display.println("Sicaklik: " + String(receivedData.temperature) + "C");
    display.display();
    delay(2000);


    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("X ekseni\nivme\n\n" +  String(receivedData.Accelx) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Y ekseni\nivme\n\n" +  String(receivedData.Accely) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Z ekseni\nivme\n\n" +  String(receivedData.Accelz) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("X ekseni\naci\n\n" +  String(receivedData.Gyrox) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Y ekseni\naci\n\n" +  String(receivedData.Gyroy) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Z ekseni\naci\n\n" +  String(receivedData.Gyroz) + "m/s^2 ");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Basinc:\n " +  String(receivedData.pressure/1000) + "kPa");
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.println("Yukseklik \n" +  String(receivedData.altitude/1000) + "km");
    display.display();
    delay(2000);
  }
  else{
    Serial.println("Veri alinamadi");
    display.clearDisplay();
    display.setCursor(10, 10);
    display.setTextSize(2);
    display.println("Veri \n Alinamadi");
    display.display();
    delay(500);
  }
  }

  
