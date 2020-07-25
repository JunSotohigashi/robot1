#include "BMX055.hpp"
#include "mbed.h"

BMX055::BMX055(PinName pin_SDA, PinName pin_SCL)
    : mI2C(pin_SDA, pin_SCL),
      mAddr_Acc(0x19 << 1),
      mAddr_Gyr(0x69 << 1),
      mAddr_Mag(0x13 << 1) {
        init();
        //mTicker.attach(callback(this, &BMX055::calc_Attitude), 0.005f);
        mThread.start(callback(&mEventQueue, &EventQueue::dispatch_forever));
        mEventQueue.call_every(5, callback(this, &BMX055::calc_Attitude));
}


BMX055::~BMX055() {}


void BMX055::init() {
    int delay = 1;  //データ送信の間隔
    char buffer[2];

    //加速度センサー
    //-----------------------------------------------------//
    buffer[0] = 0x0F;    //PMU_Range レジスタを選択
    buffer[1] = 0x03;    //レンジを +/- 2g に設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x10;    //PMU_BW レジスタを選択
    buffer[1] = 0x08;    //帯域幅を 7.18Hz に設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x11;    //PMU_LPW レジスタを選択
    buffer[1] = 0x00;    //ノーマルモードに設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);

    //ジャイロセンサー
    //-----------------------------------------------------//
    buffer[0] = 0x0F;    //PMU_Range レジスタを選択
    buffer[1] = 0x03;    //レンジを +/- 250deg/s に設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x10;    //PMU_BW レジスタを選択
    buffer[1] = 0b10;    //ODR = 1000Hz に設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x11;    //PMU_LPW レジスタを選択
    buffer[1] = 0x00;    //ノーマルモードに設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    
    //磁気センサー
    //-----------------------------------------------------//
    buffer[0] = 0x4B;    //レジスタを選択
    buffer[1] = 0x83;    //ソフトリセット
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4B;    //レジスタを選択
    buffer[1] = 0x01;    //ソフトリセット
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4C;    //レジスタを選択
    buffer[1] = 0x00;    //ODR = 10Hz
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4E;    //レジスタを選択
    buffer[1] = 0x84;    //X, Y, Z軸を有効化
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x51;    //レジスタを選択
    buffer[1] = 0x04;    //No. of Repetitions for X-Y Axis = 9
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x52;    //レジスタを選択
    buffer[1] = 0x16;    //No. of Repetitions for Z-Axis = 15
    mI2C.write(mAddr_Mag, buffer, 2, false);
}


float BMX055::read_Accl_X(){
    char buffer[2];
    buffer[0] = 0x02;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


float BMX055::read_Accl_Y(){
    char buffer[2];
    buffer[0] = 0x04;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


float BMX055::read_Accl_Z(){
    char buffer[2];
    buffer[0] = 0x06;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


float BMX055::read_Gyro_X(){
    char buffer[2];
    buffer[0] = 0x02;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


float BMX055::read_Gyro_Y(){
    char buffer[2];
    buffer[0] = 0x04;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


float BMX055::read_Gyro_Z(){
    char buffer[2];
    buffer[0] = 0x06;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


void BMX055::calc_Attitude(){
    float dt = 0.005f;
    float accl_x = read_Accl_X();  //一時的に加速度を保存する
    float accl_z = read_Accl_Z();
    float accl_pitch = atan2(accl_x, accl_z) / 3.141592654f * 180.0f;  //加速度センサーからのピッチ角を計算
    float accl_amp = sqrt(accl_x * accl_x + accl_z * accl_z);  //加速度の大きさを計算
    
    float gyro_y = -read_Gyro_Y() - offset_gyro;
    float k =  0.005f;//0.02f * pow(2.718281828f, (-(accl_amp - 9.8f) * (accl_amp - 9.8f)) / 0.5f);


    float old_mPitch_Angle = mPitch_Angle_raw;
    float new_mPitch_Angle = old_mPitch_Angle + (k * (accl_pitch - old_mPitch_Angle)) + (1.0f-k) * (gyro_y * dt);

    mPitch_Speed = gyro_y;//(new_mPitch_Angle - old_mPitch_Angle) / dt;
    mPitch_Angle_raw = new_mPitch_Angle;
    mPitch_Angle = new_mPitch_Angle + offset_angle;
}