
#include "MMC34160PJ.h"

MMC34160PJ::MMC34160PJ(int addr){
    _wire = &Wire;
    i2cAddress = addr;   
}

MMC34160PJ::MMC34160PJ(){
    _wire = &Wire;
    i2cAddress = MMC34160PJ_ADDRESS;   
}

MMC34160PJ::MMC34160PJ(TwoWire *w, int addr){
    _wire = w;
    i2cAddress = addr; 
}

MMC34160PJ::MMC34160PJ(TwoWire *w){
    _wire = w;
    i2cAddress = MMC34160PJ_ADDRESS;
}
    
bool MMC34160PJ::init(){ 
    float out1[3];
    float out2[3];
    byte ack = writeRegister(0x07, 0x80);
    if (ack != 0) return false;
    delay(60);
    ack = writeRegister(0x07, 0x20);
    if (ack != 0) return false;
    delay(10);
    if (!readData()) return false;
    for (int i=0; i<3; i++) {
        out1[i] = 0.48828125 * (float)raw[i];
    }
    ack = writeRegister(0x07, 0x80);
    if (ack != 0) return false;
    delay(60);
    ack = writeRegister(0x07, 0x40);
    if (ack != 0) return false;
    delay(10);
    if (!readData()) return false;
    for (int i=0; i<3; i++) {
        out2[i] = 0.48828125 * (float)raw[i];
    }
    for (int i=0; i<3; i++) {
        offset[i] = (out1[i]+out2[i])*0.5;
    }
    ack = writeRegister(0x07, 0x40);
    if (ack != 0) return false;
    delay(10);
    ack = writeRegister(0x08, 0x00);
    if (ack != 0) return false;
    delay(10);
    return true;
}

void MMC34160PJ::setDeclination(float _declination){
    declination = _declination;
}

float MMC34160PJ::getDeclination(){
    return declination;
}

bool MMC34160PJ::readData(){
    byte ack = writeRegister(0x07, 0x01);
    if (ack != 0) return false;
    bool flag = false;
    uint8_t temporal = 0;
    unsigned long _t = millis() + 1000;
    while (!flag) {
        _wire->beginTransmission(i2cAddress);
        _wire->write(byte(0x06));
        _wire->endTransmission();
        _wire->requestFrom(i2cAddress,1);
        if(_wire->available()){
            temporal = _wire->read();
        }
        temporal &= 1;
        if (temporal != 0) {
          flag = true;
        }
        if (millis() > _t) return false;
    }

    byte tmp[6] = {0, 0, 0, 0, 0, 0};
    _wire->beginTransmission(i2cAddress);
    _wire->write(byte(0x00));
    _wire->endTransmission();
    _wire->requestFrom(i2cAddress,6);
    if(_wire->available()){
        for (int i = 0; i < 6; i++) {
            tmp[i] = _wire->read(); //save it
        }
    } else {
        return false;
    }

    raw[0] = tmp[1] << 8 | tmp[0]; //x
    raw[1] = tmp[3] << 8 | tmp[2]; //y
    raw[2] = tmp[5] << 8 | tmp[4]; //z


    return true;
}

float MMC34160PJ::getAngle(){
    float data[3];
    float temp0 = 0;
    float temp1 = 0;
    float deg = 0;

    for (int i=0; i<3; i++) {
        data[i] = 0.48828125 * (float)raw[i] - offset[i];
    }
    if (data[0] < 0) {
        if (data[1] > 0) {
          //Quadrant 1
          temp0 = data[1];
          temp1 = -data[0];
          deg = 90 - atan(temp0 / temp1) * (180 / 3.14159);
        } else {
          //Quadrant 2
          temp0 = -data[1];
          temp1 = -data[0];
          deg = 90 + atan(temp0 / temp1) * (180 / 3.14159);
        }
    } else {
        if (data[1] < 0) {
          //Quadrant 3
          temp0 = -data[1];
          temp1 = data[0];
          deg = 270 - atan(temp0 / temp1) * (180 / 3.14159);
        } else {
          //Quadrant 4
          temp0 = data[1];
          temp1 = data[0];
          deg = 270 + atan(temp0 / temp1) * (180 / 3.14159);
        }
    }

    deg += declination;
    if (declination > 0) {
        if (deg > 360) {
          deg -= 360;
        }
    } else {
        if (deg < 0) {
          deg += 360;
        }
    }
    return deg;

}

uint8_t MMC34160PJ::writeRegister(uint8_t reg, uint8_t val){
  _wire->beginTransmission(i2cAddress);
  _wire->write(reg);
  _wire->write(val);
  return _wire->endTransmission();
}



