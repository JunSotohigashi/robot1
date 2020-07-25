#include "NJM4350D.hpp"
#include "mbed.h"


NJM4350D::NJM4350D(PinName pin_enable, PinName pin_mode, PinName pin_dir, PinName pin_step)
    : mPin_enable(pin_enable),
      mPin_mode(pin_mode),
      mPin_dir(pin_dir),
      mPin_step(pin_step) {
    mPin_enable.write(0); //出力を有効化するよう指示
    mPin_mode.write(0);   //ハーフステップモードに設定
    // 50us毎にupdate関数が呼び出されるよう設定
    mTicker.attach_us(callback(this, &NJM4350D::update), 25);
}


NJM4350D::~NJM4350D() {
    // update関数の呼び出しを終了
    mTicker.detach();
    //モーター出力を無効化し、シャフトのロックを解除
    mPin_enable = 1;
}


void NJM4350D::update() {
    float cmp = abs(rps);
    //目標速度がとても小さいときは0として処理
    if (cmp > 59E-9f) {
        count++;
        if (count > 100.0f / cmp) {
        count = 0;
        if (rps >= 0) {
            one_pulse(0);
            angle_raw++;
        } else {
            one_pulse(1);
            angle_raw--;
        }
        angle = angle_raw * 0.0025f;
        }
    }
}


float NJM4350D::setSpeed(float speed_rps) {
    rps = speed_rps;
    return rps;
}


float NJM4350D::getSpeed() {
    return rps;
}


float NJM4350D::getAngle() {
    return angle;
}


void NJM4350D::one_pulse(int dir) {
    mPin_dir.write(dir);
    //ダウンエッジのパルスを生成
    mPin_step.write(0);
    mPin_step.write(1);
}
