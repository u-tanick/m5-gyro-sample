#include <M5Unified.h>
#include <Kalman.h>

//Filter
Kalman kalmanX;
Kalman kalmanY;

// IMU paramater
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;

float angleX, angleZ;
float gyroRateX, gyroRateZ;
unsigned long lastUpdate;

float filteredAngleX;
float filteredAngleZ;

float originAngleX = 0.0;
float originAngleY = 0.0;
float originAngleZ = 0.0;

float relativeAngleX;
float relativeAngleZ;

void calculateAngle() {
  // 時間経過を取得
  unsigned long now = millis();
  float dt = (now - lastUpdate) / 1000.0; // 秒単位の時間経過
  lastUpdate = now;

  // 加速度から角度を計算
  angleX = atan2(accY, accZ) * 180 / PI;
  angleZ = atan2(accX, accZ) * 180 / PI;

  // ジャイロデータを使って角速度を取得
  gyroRateX = gyroX; // ジャイロの出力がdeg/sであることを前提
  gyroRateZ = gyroY;

  // カルマンフィルターを使って角度を更新
  // オフセット値を加算することでボタンを押した時点を原点とした数値に補正
  filteredAngleX = kalmanX.getAngle(angleX, gyroRateX, dt);
  filteredAngleZ = kalmanY.getAngle(angleZ, gyroRateZ, dt);
}

void resetOrigin() {
    // 原点を引いて相対角度を算出
    relativeAngleX = filteredAngleX - originAngleX;
    relativeAngleZ = filteredAngleZ - originAngleZ;
}


const uint8_t pin1 = 33;  // M5StickC
const uint8_t pin2 = 32;  // M5StickC

void setup() {

  // M5StickC Plusの初期化
  auto cfg = M5.config();
  cfg.internal_imu = true;  // 内蔵IMUを有効化
  M5.begin(cfg);
  M5.Imu.init();
  
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);
  
  // 画面の設定
  M5.Lcd.begin();
  M5.Lcd.setRotation(3);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);

  lastUpdate = millis();
}

int last_value1 = 0;
int last_value2 = 0;

int cur_value1 = 0;
int cur_value2 = 0;

void loop() {

  // --------------------------------------------------------------
  // カルマンフィルタによるジャイロからのX軸、Z軸の

  // ジャイロデータを取得
  M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.Imu.getAccelData(&accX,&accY,&accZ);

  // カルマンフィルタを使用して角度を計算
  calculateAngle();

  // 画面に表示する前にクリア
  M5.Lcd.fillScreen(BLACK);

  // ホームボタンを押したか？（1度だけ取得可能）
  if ( M5.BtnA.wasPressed() ) {
    M5.Speaker.tone(1500, 100);
    // ボタンを押した時点の角度を原点として扱うために取得
    originAngleX = filteredAngleX;
    originAngleZ = filteredAngleZ;
  }

  // ボタンを押した時点の originAngleX, originAngleY を使用して原点をリセット
  resetOrigin();

  // // 縦置きで前後回転（ジャイロから計算）
  // M5.Lcd.setCursor(0, 20);
  // M5.Lcd.printf("Angle X: %d\n", int(filteredAngleX));
  // M5.Lcd.setCursor(0, 40);
  // M5.Lcd.printf("Angle X: %d\n", int(relativeAngleX));

  // // 縦置きで左右に傾ける（ジャイロから計算）
  // M5.Lcd.setCursor(0, 70);
  // M5.Lcd.printf("Angle Z: %d\n", int(filteredAngleZ));
  // M5.Lcd.setCursor(0, 90);
  // M5.Lcd.printf("Angle Z: %d\n", int(relativeAngleZ));

  // // 縦置きで左右に水平に回転（地磁気から計算）
  // TODO

  // M5.Lcd.printf("Angle Z: %d\n", int(relativeAngleZ));
  // M5.Lcd.printf("Angle Z: %d\n", int(relativeAngleZ));

  // --------------------------------------------------------------
  // デュアルボタン
  cur_value1 = digitalRead(33);
  cur_value2 = digitalRead(32);

  if (cur_value1 != last_value1) {
      if (cur_value1 == 0) {
          M5.Lcd.setCursor(40, 25);
          M5.Lcd.print("Blue");
      } else {
          M5.Lcd.setCursor(40, 25);
          M5.Lcd.print("1");
      }
      last_value1 = cur_value1;
  }
  if (cur_value2 != last_value2) {
      M5.Lcd.fillRect(70, 25, 45, 40, BLACK);
      if (cur_value2 == 0) {
          M5.Lcd.setCursor(80, 25);
          M5.Lcd.print("Red");
      } else {
          M5.Lcd.setCursor(80, 25);
          M5.Lcd.print("1");
      }
      last_value2 = cur_value2;
  }

  M5.update(); // ボタンの状態を更新
  delay(10);
}
