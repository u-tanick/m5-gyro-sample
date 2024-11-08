#include <M5Unified.h>
#include <Kalman.h>
#include <SensorFusion.h>
#include <esp_now.h>
#include <WiFi.h>

#if defined(ARDUINO_M5STACK_Core2) // Core2の場合
u_int8_t ROTATE = 1;
u_int8_t FONT_SIZE = 3;
u_int8_t LEFT_OFFSET_POS = 10;
u_int8_t LABEL_POS = 15;
u_int8_t PARAM_POS = 50;
#elif defined(ARDUINO_M5Stick_C_PLUS) // M5StickC Plusの場合
u_int8_t ROTATE = 3;
u_int8_t FONT_SIZE = 2;
u_int8_t LEFT_OFFSET_POS = 10;
u_int8_t LABEL_POS = 10;
u_int8_t PARAM_POS = 40;
#endif

// ---------------------------------------------
// カルマンフィルター用クラス変数
Kalman kalmanX;
Kalman kalmanY;

// ---------------------------------------------
// センサーフュージョン用クラス変数
SF fusion;
float deltat;

// ---------------------------------------------
// 角度算出用変数
// 角度取得・計算用変数
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float gyroRateX, gyroRateY;
u_long lastUpdate;

// X,Y:カルマンフィルターにより計算した角度
// Z:センサフュージョンにより計算した角度
float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

// 原点補正のオフセット値
float originAngleX = 0.0;
float originAngleY = 0.0;
float originAngleZ = 0.0;

// 原点補正後の角度
float relativeAngleX = 0.0;
float relativeAngleY = 0.0;
float relativeAngleZ = 0.0;

// GyroZのデータを蓄積するための変数
float stockedGyroZs[20];
u_int stockCnt = 0;
u_int stockedGyroZLength = 0;
float adjustGyroZ = 0.0;

// ---------------------------------------------
// デュアルボタン用変数
// 用ピン情報(Poat.A)
const uint8_t pin1 = 33;
const uint8_t pin2 = 32;

int last_value1 = 0;
int last_value2 = 0;

int cur_value1 = 0;
int cur_value2 = 0;

// ---------------------------------------------
// データ送信用変数
bool SEND_DATA = false;

/**
 * カルマンフィルターを用いた角度計算
 */
void calculateAngle()
{

  // 時間経過を取得
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0; // 秒単位の時間経過
  lastUpdate = now;

  // 加速度から角度を計算
  float pitch = atan2(accY, accZ) * 180 / PI;
  float roll = atan2(accX, accZ) * 180 / PI;

  // カルマンフィルターを使って角度を更新
  // オフセット値を加算することでボタンを押した時点を原点とした数値に補正
  angleX = kalmanX.getAngle(pitch, gyroX, dt);
  angleY = kalmanY.getAngle(roll, gyroY, dt);
}

/**
 * 原点補正の実施（ボタンを押下した時点を原点に設定）
 */
void resetOrigin()
{
  // 原点を引いて相対角度を算出
  relativeAngleX = angleX - originAngleX;
  relativeAngleY = angleY - originAngleY;
  relativeAngleZ = angleZ - originAngleZ;
}

// 受信機のMACアドレスをセットする
uint8_t slaveAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/**
 * 送信完了時の処理実行用コールバック関数
 * 不要な場合は削除してよい
 */
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(5, 10);
  M5.Lcd.println(macStr);
  M5.Lcd.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

/**
 * 送信データセット用関数
 */
void espNowSendData(float sendX, float sendZ)
{
  // 送信データ
  // uint8_t data[2] = { (int)sendX, (int)sendZ };
  // テスト送信データ
  uint8_t data[2] = { 111, 222 };

  // データ送信
  esp_err_t result = esp_now_send(slaveAddress, data, sizeof(data));

  // 送信結果
  Serial.print("Send Status: ");
  switch (result)
  {
  case ESP_OK:
      Serial.println("Success");
      break;
  case ESP_ERR_ESPNOW_NOT_INIT:
      Serial.println("ESPNOW not Init.");
      break;
  case ESP_ERR_ESPNOW_ARG:
      Serial.println("Invalid Argument");
      break;
  case ESP_ERR_ESPNOW_INTERNAL:
      Serial.println("Internal Error");
      break;
  case ESP_ERR_ESPNOW_NO_MEM:
      Serial.println("ESP_ERR_ESPNOW_NO_MEM");
      break;
  case ESP_ERR_ESPNOW_NOT_FOUND:
      Serial.println("Peer not found.");
      break;

  default:
      Serial.println("Not sure what happened");
      break;
  }
}

// ---------------------------------------------

/**
 * main / setup
 */
void setup()
{
  // Z軸（yaw角ドリフト対策）
  stockedGyroZLength = sizeof(stockedGyroZs) / sizeof(int);

  // M5StickC Plusの初期化
  auto cfg = M5.config();
  cfg.internal_imu = true; // 内蔵IMUを有効化
  M5.begin(cfg);

  M5.Imu.init();

  Serial.println(" IMU initialised");

  // Wifi通信設定
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }

  // デュアルボタンのPIN情報設定
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);

  Serial.println(" Dual Button Setuped");

  // 画面の設定
  M5.Lcd.begin();
  // Core2: 1=標準, 2=時計周り90度, 3=上下逆, 4=反時計周り90度
  M5.Lcd.setRotation(ROTATE);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(FONT_SIZE);

  Serial.println(" LCD Setuped");

  // ESPNow送信設定
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, slaveAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  // 送信完了時の処理実行用コールバック関数をセット（不要な場合は削除してよい）
  esp_now_register_send_cb(onSend);

  // 角度補正計算用タイマー取得
  lastUpdate = millis();
}

/**
 * main / loop
 */
void loop()
{

  // --------------------------------------------------------------
  // カルマンフィルタによるジャイロからのX軸、Y軸の傾き角度取得

  // ジャイロデータを取得
  M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.Imu.getAccelData(&accX, &accY, &accZ);

  // カルマンフィルタを使用してangleX（pitch）, angleY（roll）の角度を計算
  calculateAngle();

  // --------------------------------------------------------------
  // センサーフュージョンによるZ軸（方位角度）の角度取得：初期値は180
  // M5Stackのセンサーの既知の問題であるZ軸のドリフトに補正をかけたもの

  // 起動時にstockedGyroZLengthの数だけデータを貯める
  if (stockCnt < stockedGyroZLength)
  {
    stockedGyroZs[stockCnt] = gyroZ;
    stockCnt++;
  }
  else
  {
    if (adjustGyroZ == 0)
    {
      for (int i = 0; i < stockedGyroZLength; i++)
      {
        adjustGyroZ += stockedGyroZs[i] / stockedGyroZLength;
      }
    }
    // 貯めたデータの平均値を使ってgyroZを補正する
    gyroZ -= adjustGyroZ;

    // センサーフュージョンを使用してangleZ（yaw）を取得
    deltat = fusion.deltatUpdate();
    // 補正したgyroデータを使ってyawを計算
    fusion.MahonyUpdate(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, deltat);
    // 地磁気データ mx, my, mzを使う関数もあるが、、、
    angleZ = fusion.getYaw();
  }

  // 画面に表示する前にクリア
  M5.Lcd.fillScreen(BLACK);

  // --------------------------------------------------------------
  // デュアルボタンのイベント取得
  cur_value1 = digitalRead(33);
  cur_value2 = digitalRead(32);

  if (cur_value1 != last_value1)
  {
    // Blueボタン
    if (cur_value1 == 0)
    {
      M5.Speaker.tone(2500, 200);
      M5.Lcd.setCursor(LEFT_OFFSET_POS, LABEL_POS);
      M5.Lcd.print("Blue Button Prsed");
      // いったん何もしない
    }
    last_value1 = cur_value1;
  }
  if (cur_value2 != last_value2)
  {
    // Redボタン
    if (cur_value2 == 0)
    {
      if (!SEND_DATA)
      {
        M5.Speaker.tone(1500, 200);
        M5.Lcd.setCursor(LEFT_OFFSET_POS, LABEL_POS);
        M5.Lcd.print("Reset Origin");

        // ボタンを押した時点の角度を原点として扱うために取得
        originAngleX = angleX;
        originAngleY = angleY;
        originAngleZ = angleZ;
        // 通信開始のフラグを立てる
        SEND_DATA = true;
      } else {
        // 二回押すとデータ送信をストップ
        SEND_DATA = false;
      }
    }

    last_value2 = cur_value2;
  }

  // ボタンを押した時点の originAngleX, originAngleY を使用して原点をリセット
  resetOrigin();

  // 送信フラグがONの場合、原点補正済みのX,Zの角度情報をESPNowでサーバー側に送信する
  if (SEND_DATA)
  {
    espNowSendData(relativeAngleX, relativeAngleZ);
  }

  // --------------------------------------------------------------
  // 画面表示

  // 前後に傾ける（ジャイロから計算）
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS);
  M5.Lcd.printf("RAW   X: %d\n", int(angleX));
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE * 10 * 1);
  M5.Lcd.printf("RESET X: %d\n", int(relativeAngleX));

  // 左右に傾ける（ジャイロから計算）
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE * 10 * 2);
  M5.Lcd.printf("RAW   Y: %d\n", int(angleY));
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE * 10 * 3);
  M5.Lcd.printf("RESET Y: %d\n", int(relativeAngleY));

  // 左右に水平に回転（地磁気から計算）
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE * 10 * 4);
  M5.Lcd.printf("RAW   Z: %d\n", int(angleZ));
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE * 10 * 5);
  M5.Lcd.printf("RESET Z: %d\n", int(relativeAngleZ));

  M5.update(); // ボタンの状態を更新
  delay(10);
}
