// NJM4350D(ユニポーラステッピングモータドライバ)の制御用クラス
//スレッドを利用した並列処理を実装(予定)

#ifndef NJM4350D_H_
#define NJM4350D_H_

#include "mbed.h"


class NJM4350D {
    public:
        NJM4350D(PinName pin_enable, PinName pin_mode, PinName pin_dir, PinName pin_step);
        ~NJM4350D();
        void update();
        float setSpeed(float rps);
        float getSpeed();
        float getAngle();

    private:
        //出力ピンを格納する
        DigitalOut mPin_enable;
        DigitalOut mPin_mode;
        DigitalOut mPin_dir;
        DigitalOut mPin_step;
        //周期的にupdate関数を呼び出すためTickerを用意
        Ticker mTicker;

        void one_pulse(int dir); // 1パルス送る
        int angle_raw = 0;       //現在角度(ステップ数)
        int count = 0;           //周期的に増加するカウンタ
        float rps = 0;           //速度(rps)
        float angle = 0;         //角度(rev)
};

#endif
