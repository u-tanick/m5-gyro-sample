#include <M5Unified.h>
#include <Kalman.h>

#if defined(ARDUINO_M5STACK_Core2)	// Core2の場合
  u_int8_t ROTATE = 1;
  u_int8_t FONT_SIZE= 3;
  u_int8_t LEFT_OFFSET_POS = 10;
  u_int8_t LABEL_POS = 15;
  u_int8_t PARAM_POS = 50;
#elif defined(ARDUINO_M5Stick_C_PLUS)	// M5StickC Plusの場合
  u_int8_t ROTATE = 3;
  u_int8_t FONT_SIZE= 2;
  u_int8_t LEFT_OFFSET_POS = 10;
  u_int8_t LABEL_POS = 10;
  u_int8_t PARAM_POS = 40;
#endif

// カルマンフィルター用クラス変数
Kalman kalmanX;
Kalman kalmanY;

// 角度取得・計算用変数
float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float angleX, angleY;
float gyroRateX, gyroRateY;
unsigned long lastUpdate;

// カルマンフィルターによる補正計算後の角度
float filteredAngleX;
float filteredAngleY;

// 原点補正のオフセット値
float originAngleX = 0.0;
float originAngleY = 0.0;
float originAngleZ = 0.0;

// 原点補正後の角度
float relativeAngleX;
float relativeAngleY;

// デュアルボタン用ピン情報(Poat.A)
const uint8_t pin1 = 33;
const uint8_t pin2 = 32;

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
  angleX = atan2(accY, accZ) * 180 / PI;
  angleY = atan2(accX, accZ) * 180 / PI;

  // ジャイロデータを使って角速度を取得
  gyroRateX = gyroX; // ジャイロの出力がdeg/sであることを前提
  gyroRateY = gyroY;

  // カルマンフィルターを使って角度を更新
  // オフセット値を加算することでボタンを押した時点を原点とした数値に補正
  filteredAngleX = kalmanX.getAngle(angleX, gyroRateX, dt);
  filteredAngleY = kalmanY.getAngle(angleY, gyroRateY, dt);
}

/**
 * 原点補正後の実施（ボタンを押下した時点を原点に設定）
 */
void resetOrigin()
{
  // 原点を引いて相対角度を算出
  relativeAngleX = filteredAngleX - originAngleX;
  relativeAngleY = filteredAngleY - originAngleY;
}

/**
 * main / setup
 */
void setup()
{

  // M5StickC Plusの初期化
  auto cfg = M5.config();
  cfg.internal_imu = true; // 内蔵IMUを有効化
  M5.begin(cfg);
  M5.Imu.init();

  // デュアルボタンのPIN情報設定
  pinMode(pin1, INPUT);
  pinMode(pin2, INPUT);

  // 画面の設定
  M5.Lcd.begin();
  // Core2: 1=標準, 2=時計周り90度, 3=上下逆, 4=反時計周り90度
  M5.Lcd.setRotation(ROTATE);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setTextSize(FONT_SIZE);

  lastUpdate = millis();
}

int last_value1 = 0;
int last_value2 = 0;

int cur_value1 = 0;
int cur_value2 = 0;

/**
 * main / loop
 */
void loop()
{

  // --------------------------------------------------------------
  // カルマンフィルタによるジャイロからのX軸、Y軸の傾き取得

  // ジャイロデータを取得
  M5.Imu.getGyroData(&gyroX, &gyroY, &gyroZ);
  M5.Imu.getAccelData(&accX, &accY, &accZ);

  // カルマンフィルタを使用して角度を計算
  calculateAngle();

  // 画面に表示する前にクリア
  M5.Lcd.fillScreen(BLACK);

  // ホームボタンを押したか？（1度だけ取得可能）
  if (M5.BtnA.wasPressed())
  {
    M5.Speaker.tone(1500, 100);
    // ボタンを押した時点の角度を原点として扱うために取得
    originAngleX = filteredAngleX;
    originAngleY = filteredAngleY;
  }

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
    }
    last_value1 = cur_value1;
  }
  if (cur_value2 != last_value2)
  {
    // Redボタン
    M5.Lcd.fillRect(70, 25, 45, 40, BLACK);
    if (cur_value2 == 0)
    {
      M5.Speaker.tone(1500, 200);
      M5.Lcd.setCursor(LEFT_OFFSET_POS, LABEL_POS);
      M5.Lcd.print("Red Button Prsed");
      // ボタンを押した時点の角度を原点として扱うために取得
      originAngleX = filteredAngleX;
      originAngleY = filteredAngleY;
    }
    last_value2 = cur_value2;
  }

  // ボタンを押した時点の originAngleX, originAngleY を使用して原点をリセット
  resetOrigin();

  // 縦置きで前後回転（ジャイロから計算）
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS);
  M5.Lcd.printf("Angle X: %d\n", int(filteredAngleX));
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE*10*1);
  M5.Lcd.printf("Angle X: %d\n", int(relativeAngleX));

  // 縦置きで左右に傾ける（ジャイロから計算）
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE*10*2);
  M5.Lcd.printf("Angle Y: %d\n", int(filteredAngleY));
  M5.Lcd.setCursor(LEFT_OFFSET_POS, PARAM_POS + FONT_SIZE*10*3);
  M5.Lcd.printf("Angle Y: %d\n", int(relativeAngleY));

  // // 縦置きで左右に水平に回転（地磁気から計算）
  // TODO
  // M5.Lcd.printf("Angle Z: %d\n", int(relativeAngleZ));
  // M5.Lcd.printf("Angle Z: %d\n", int(relativeAngleZ));

  M5.update(); // ボタンの状態を更新
  delay(100);
}
