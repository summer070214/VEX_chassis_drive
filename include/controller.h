#ifndef CONTROLL_H
#define CONTROLL_H
struct Controller_Data {
    int Ch1;
    int Ch3;
    int Ch4;
    bool BtnX;
    bool BtnY;
    bool BtnUp;
    bool BtnDown;
    bool BtnL;
    bool BtnR;
    bool LL1;
    bool LL2;
};
extern Controller_Data controller_data;
void controller_data_read();

void handleDriveControl();
#endif