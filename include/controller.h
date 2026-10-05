#ifndef CONTROLLER_H
#define CONTROLLER_H
struct ControllerData {
    // 摇杆 (范围 -100 ~ 100)
    int Ch1;   // 右摇杆 X
    int Ch2;   // 右摇杆 Y
    int Ch3;   // 左摇杆 Y
    int Ch4;   // 左摇杆 X

    // 右侧 ABXY 按键
    bool BtnA;
    bool BtnB;
    bool BtnX;
    bool BtnY;

    // 方向键
    bool BtnUp;
    bool BtnDown;
    bool BtnLeft;
    bool BtnRight;

    // 肩部按键
    bool L1;
    bool L2;
    bool R1;
    bool R2;
};
extern ControllerData controller_data;
void controller_data_read();

void handleDriveControl();
#endif