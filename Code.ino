typedef struct {
  const uint16_t* data;
  size_t length;
  const char* action;
} IRCommand;

QueueHandle_t irQueue;

#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <IRremote.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include <time.h>

HardwareSerial mySerial(2); // UART2
PZEM004Tv30 pzem(mySerial, 4, 16); // RX, TX

#define WIFI_SSID "xuong TD"
#define WIFI_PASSWORD "p.hshop10102015"

#define FIREBASE_PROJECT_ID "kltndh21td"
#define API_KEY "AIzaSyB6bFOl0QniLAedLjKuorSoXCHiCtn-sW8"
#define DATABASE_URL "https://kltndh21td-default-rtdb.firebaseio.com/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

#define IR_LED_PIN 27

#define DS18B20_PIN 19
#define DHT_PIN 18
#define DHT_TYPE DHT22

#define BUTTON_ON       23  // Nút bật
#define BUTTON_OFF      14  // Nút tắt
#define BUTTON_DEFAULT  26  // Nút reset

#define RELAY4_PIN 17 // PS
#define RELAY2_PIN 32 // ĐÈN VÀNG
#define RELAY3_PIN 25 // ĐÈN XANH
#define RELAY1_PIN 33 // ĐÈN ĐỎ

uint16_t raw_2A46B51B[] =   {4450, 4450, 500, 1600, 500, 600, 500, 1600, 500, 1600, 500, 600, 500, 600, 500, 1600, 500, 600, 500, 550, 500, 1650, 500, 550, 500, 600, 500, 1600, 500, 1650, 500, 550, 500, 1650, 500, 600, 500, 1600, 500, 1600, 500, 1600, 500, 1650, 500, 550, 500, 1650, 550, 1550, 500, 1600, 550, 550, 550, 550, 500, 550, 550, 550, 550, 1600, 500, 550, 550, 550, 500, 1600, 550, 1600, 500, 1600, 500, 600, 500, 550, 550, 550, 500, 600, 550, 500, 550, 550, 500, 600, 500, 550, 550, 1600, 500, 1600, 500, 1600, 550, 1600, 550, 1550, 550, 5650, 4450, 4400, 550, 1600, 550, 500, 550, 1600, 500, 1600, 550, 550, 500, 600, 500, 1600, 500, 600, 500, 550, 550, 1600, 550, 500, 550, 550, 500, 1600, 550, 1600, 550, 500, 550, 1600, 500, 550, 550, 1600, 500, 1600, 500, 1600, 550, 1600, 500, 550, 550, 1600, 500, 1600, 500, 1600, 550, 550, 550, 550, 500, 550, 550, 550, 500, 1600, 550, 550, 500, 600, 500, 1600, 500, 1600, 550, 1600, 500, 550, 550, 550, 550, 550, 500, 550, 550, 550, 550, 550, 500, 600, 500, 550, 550, 1550, 550, 1600, 500, 1600, 550, 1550, 550, 1600, 500};
uint16_t raw_84D3A633[] = {4550, 4400, 600, 1550, 600, 500, 600, 1550, 550, 1550, 600, 500, 600, 500, 600, 1550, 600, 500, 550, 550, 550, 1550, 600, 500, 600, 500, 600, 1550, 600, 1550, 550, 500, 600, 1550, 500, 1650, 600, 500, 550, 1550, 600, 1550, 600, 1550, 600, 1550, 550, 1550, 600, 1550, 600, 500, 600, 1550, 550, 550, 550, 500, 600, 500, 600, 500, 600, 500, 600, 500, 500, 600, 500, 600, 500, 1600, 600, 500, 600, 500, 600, 500, 600, 500, 500, 600, 600, 1550, 550, 1550, 600, 500, 600, 1550, 600, 1550, 550, 1550, 600, 1550, 600, 1550, 550, 5700, 4450, 4500, 550, 1600, 500, 600, 500, 1600, 550, 1600, 550, 550, 550, 550, 500, 1650, 550, 500, 550, 550, 500, 1650, 550, 550, 500, 600, 500, 1650, 500, 1600, 500, 600, 500, 1650, 500, 1650, 500, 550, 500, 1650, 500, 1650, 500, 1650, 500, 1600, 500, 1650, 550, 1600, 500, 600, 500, 1600, 550, 550, 500, 600, 500, 600, 550, 550, 500, 600, 500, 600, 500, 550, 500, 600, 500, 1650, 550, 550, 500, 600, 500, 600, 500, 600, 500, 600, 500, 1600, 550, 1600, 500, 600, 500, 1600, 550, 1600, 550, 1600, 500, 1650, 500, 1600, 550};

bool lastOnState = HIGH;
bool lastOffState = HIGH;
bool lastDefaultState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;
unsigned long lastLCDUpdate = 0;
const unsigned long lcdUpdateInterval = 2000;
float lastTempDS = -100, lastTempDHT = -100, lastHumidDHT = -1;
bool isRelay4On = false;

float voltage = 0, current = 0, power = 0, energy = 0, frequency = 0, pf = 0;

String lastCmd = "";

OneWire oneWire(DS18B20_PIN);
DallasTemperature ds18b20(&oneWire);
DHT dht(DHT_PIN, DHT_TYPE);
LiquidCrystal_I2C lcd(0x27, 20, 4);

float tempDS, tempDHT, humidDHT;

void initTime() {
  configTime(7 * 3600, 0, "1.vn.pool.ntp.org", "time.nist.gov", "time.windows.com");  // GMT+7
  Serial.print("⏳ Đồng bộ thời gian...");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("✅ Đã có thời gian thực!");
}

void connectWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ Đã kết nối WiFi");
}

void taskSendFirebase(void* pvParameters) {
  for (;;) {
    if (Firebase.ready() && WiFi.status() == WL_CONNECTED) {
      // Gửi dữ liệu Realtime Database
      Firebase.RTDB.setFloat(&fbdo, "/sensor/tempDS", tempDS);
      Firebase.RTDB.setFloat(&fbdo, "/sensor/tempDHT", tempDHT);
      Firebase.RTDB.setFloat(&fbdo, "/sensor/humidDHT", humidDHT);

      // Gửi lên Firestore
      FirebaseJson json;
      json.set("fields/tempDS/doubleValue", tempDS);
      json.set("fields/tempDHT/doubleValue", tempDHT);
      json.set("fields/humidDHT/doubleValue", humidDHT);

      time_t now = time(nullptr);
      struct tm* timeinfo = localtime(&now);
      char timeStr[30];
      strftime(timeStr, sizeof(timeStr), "%Y%m%d_%H%M%S", timeinfo);  // ISO 8601
      String docId = String(timeStr);
      String path = "realtime/" + docId;
      json.set("fields/timestamp/stringValue", String(timeStr));

      Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", path.c_str(), json.raw());

      Serial.println("Đã gửi dữ liệu Firebase");
    }
    vTaskDelay(pdMS_TO_TICKS(10000));  // Mỗi 5 giây
  }
}

//void taskNhanLenhRTDB(void* pvParameters) {
//  String lastCommand = "";
//
//  for (;;) {
//    if (Firebase.ready() && WiFi.status() == WL_CONNECTED) {
//      String cmd;
//      if (Firebase.RTDB.getString(&fbdo, "/command", &cmd)) {
//        cmd.trim();
//        if (cmd.length() > 0 && cmd != lastCommand) {
//          Serial.printf("📥 Nhận lệnh RTDB: %s\n", cmd.c_str());
//          lastCommand = cmd;
//
//          if (cmd == "T1") digitalWrite(RELAY4_PIN, HIGH);
//          else if (cmd == "T2") digitalWrite(RELAY4_PIN, LOW);
//          else if (cmd == "S1") {
//            IRCommand irCmd = { raw_2A46B51B, 199, "BẬT" };
//            xQueueSend(irQueue, &irCmd, portMAX_DELAY);
//            digitalWrite(RELAY3_PIN, HIGH);
//            digitalWrite(RELAY1_PIN, LOW);
//            digitalWrite(RELAY2_PIN, LOW);
//          } else if (cmd == "S2") {
//            IRCommand irCmd = { raw_84D3A633, 199, "TẮT" };
//            xQueueSend(irQueue, &irCmd, portMAX_DELAY);
//            digitalWrite(RELAY1_PIN, HIGH);
//            digitalWrite(RELAY2_PIN, LOW);
//            digitalWrite(RELAY3_PIN, LOW);
//          }
//
//          // Xoá lệnh sau khi xử lý
//          Firebase.RTDB.deleteNode(&fbdo, "/command");
//        }
//      }
//    }
//
//    vTaskDelay(pdMS_TO_TICKS(3000));
//  }
//}

void taskIRSender(void* pvParameters) {
  IRCommand cmd;
  for (;;) {
    if (xQueueReceive(irQueue, &cmd, portMAX_DELAY) == pdTRUE) {
      Serial.printf("📡 IR: Gửi lệnh %s\n", cmd.action);
      IrSender.sendRaw(cmd.data, cmd.length, 38);
    }
  }
}

void doccb() {
  ds18b20.requestTemperatures();
  tempDS = ds18b20.getTempCByIndex(0);
  tempDHT = dht.readTemperature();
  humidDHT = dht.readHumidity();
  Serial.printf("Nhiệt độ DS18B20: %.2f°C\n", tempDS);
  Serial.printf("Nhiệt độ DHT22: %.2f°C | Độ ẩm: %.2f%%\n", tempDHT, humidDHT);
//  if (!isRelay4On && humidDHT < 54) {
//    digitalWrite(RELAY4_PIN, HIGH);
//    isRelay4On = true;
//    Serial.println("⚡ BẬT relay vì độ ẩm thấp");
//  } else if (isRelay4On && humidDHT > 60) {
//    digitalWrite(RELAY4_PIN, LOW);
//    isRelay4On = false;
//    Serial.println("🛑 TẮT relay vì độ ẩm cao");
//  }
}

void taskDocCamBien(void* pvParameters) {
  for (;;) {
    doccb();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Đọc mỗi 2 giây
  }
}

void nutnhan() {
  bool currOn = digitalRead(BUTTON_ON);
  bool currOff = digitalRead(BUTTON_OFF);
  bool currDefault = digitalRead(BUTTON_DEFAULT);
  unsigned long now = millis();

  // Nút RESET
  if (lastOnState == HIGH && currOn == LOW && now - lastDebounceTime > debounceDelay) {
    Serial.println("Nút RESET được nhấn");
    digitalWrite(RELAY2_PIN, HIGH);  // ĐÈN VÀNG
    digitalWrite(RELAY3_PIN, LOW);
    digitalWrite(RELAY1_PIN, LOW);
    lastDebounceTime = now;
  }
  lastOnState = currOn;

  // Nút TẮT
  if (lastOffState == HIGH && currOff == LOW && now - lastDebounceTime > debounceDelay) {
    Serial.println("Nút TẮT được nhấn");
    IRCommand irCmd = { raw_2A46B51B, 199, "BẬT" };
    xQueueSend(irQueue, &irCmd, portMAX_DELAY);
    digitalWrite(RELAY1_PIN, HIGH);  // ĐÈN ĐỎ
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);
    lastDebounceTime = now;
  }
  lastOffState = currOff;

  // Nút BẬT
  if (lastDefaultState == HIGH && currDefault == LOW && now - lastDebounceTime > debounceDelay) {
    Serial.println("Nút BẬT được nhấn");
    IRCommand irCmd = { raw_84D3A633, 199, "TẮT" };
    xQueueSend(irQueue, &irCmd, portMAX_DELAY);
    digitalWrite(RELAY3_PIN, HIGH);  // ĐÈN XANH
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY1_PIN, LOW);
    lastDebounceTime = now;
  }
  lastDefaultState = currDefault;
}

void taskLCD(void* pvParameters) {
  for (;;) {
    if (abs(tempDS - lastTempDS) > 0.1 ||
        abs(tempDHT - lastTempDHT) > 0.1 ||
        abs(humidDHT - lastHumidDHT) > 0.5) {
      lcd.clear();
//      lcd.setCursor(0, 0); lcd.print("Nhiet DHT : "); lcd.print(tempDHT, 1); lcd.print(" C");
      lcd.setCursor(0, 0); lcd.print("Nhiet DS18: "); lcd.print(tempDS, 1); lcd.print(" C");
      lcd.setCursor(0, 1); lcd.print("Do am     : "); lcd.print(humidDHT, 0); lcd.print("%");
      lcd.setCursor(0, 2); lcd.print("U: 235V  I: 1.6A");
      lcd.setCursor(0, 3); lcd.print("P: 900W");
      lastTempDS = tempDS;
      lastTempDHT = tempDHT;
      lastHumidDHT = humidDHT;
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void taskDocPZEM(void *pvParameters) {
  for (;;) {
    voltage = pzem.voltage();
    current = pzem.current();
    power = pzem.power();
    energy = pzem.energy();
    frequency = pzem.frequency();
    pf = pzem.pf();

    if (isnan(voltage) || isnan(current) || isnan(power) || isnan(energy) || isnan(frequency) || isnan(pf)) {
      Serial.println("⚠️ Không đọc được dữ liệu từ PZEM.");
    } else {
      Serial.printf("🔌 U: %.2f V | ⚡ I: %.2f A | 💡 P: %.2f W\n", voltage, current, power);
      Serial.printf("🔋 Năng lượng: %.2f Wh | 🎵 F: %.2f Hz | PF: %.2f\n", energy, frequency, pf);
    }

    vTaskDelay(pdMS_TO_TICKS(5000));  // Đọc mỗi 5 giây
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Bắt đầu hệ thống điều khiển bằng nút nhấn + IR");

  irQueue = xQueueCreate(5, sizeof(IRCommand));

  IrSender.begin(IR_LED_PIN, ENABLE_LED_FEEDBACK, false);
  dht.begin();
  lcd.init();         // Khởi tạo LCD
  lcd.backlight();    // Bật đèn nền (nếu có)
  ds18b20.begin();
  
  pinMode(BUTTON_ON, INPUT);
  pinMode(BUTTON_OFF, INPUT);
  pinMode(BUTTON_DEFAULT, INPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  connectWiFi();
  initTime();
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("✅ Đăng nhập Firebase thành công");
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
  } else {
    Serial.printf("❌ Lỗi Firebase: %s\n", config.signer.signupError.message.c_str());
  }
  
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  xTaskCreatePinnedToCore(taskDocCamBien, "Doc_Cam_Bien", 4096, NULL, 1, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(taskLCD, "LCD_Task", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskSendFirebase, "SendFirebase", 8192, NULL, 3, NULL, 0);  
//  xTaskCreatePinnedToCore(taskNhanLenhRTDB, "NhanLenhRTDB", 4096, NULL, 2, NULL, 1); 
  xTaskCreatePinnedToCore(taskIRSender, "IR_Sender", 4096, NULL, 2, NULL, 1); // Core 1
  xTaskCreatePinnedToCore(taskDocPZEM, "Doc_PZEM", 4096, NULL, 1, NULL, 1); // Core 1

}

void loop() {
  nutnhan();
  vTaskDelay(pdMS_TO_TICKS(10));
}
