/**
 * @brief Driver for InvenSense MPU-9250/6500/9255
 * @file MPU9250.cpp
 * @author Victor Bondaruk<victorbondaruk@gmail.com>
 * 
 * IMU MPU-9250 MEMS 9-axis DOF
 * Gyroscope, Accelerometer, Magnetometer and Temperature.
 * 
 * |------------------------------------------------------------|
 * |   MPU    | SBC-PCB ||||||||||||||||||||||||||| - Cabling - |
 * |----------+---------+---------------------------------------|
 * | VCC     -> VCC     | Power supply voltage                  |
 * | GND     -> GND     | Power supply ground                   |
 * | SDA     -> SDA     | I2C data line                         |
 * | SCL     -> SCL     | I2C clock line                        |
 * | INT     -> ---     | Interrupt digital output              |
 * | FSYNC   -> GND     | Frame synchronisaion digital input    |
 * |--------------------+---------------------------------------|
 * 
 * Supply voltage: 3.3V to 5V DC
 * Signal voltage: 3.3V (not tolerant of 5V)
 * Gyroscope range: ± 250, ± 500, ± 1000, ± 2000 ° / s (16 bits)
 * Accelerometer range: ± 2, ± 4, ± 8, ± 16 g (16 bits)
 * Magnetometer range: ± 4800μT (14 or 16 bits)
 * MPU9250 requires 3.3V but some modules have 3.3V regulator built in.
 * MPU-9250 = Gyro + Accelerometer + Compass
 * MPU-6500 = Gyro + Accelerometer
 * MPU-9255 = acceleration, angular velocity and magnetic
 * MPU-9250 Product Specification Revision 1.1
 * https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
 * MPU-9250 Register Map and Descriptions Revision 1.6
 * https://invensense.tdk.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf
*/
#include "MPU9250.h"

//************************ Constructors

MPU9250::MPU9250(int addr){
    _wire = &Wire;
    i2cAddress = addr;
}

MPU9250::MPU9250(){
    _wire = &Wire;
    i2cAddress = 0x68;
}

MPU9250::MPU9250(TwoWire *w, int addr){
    _wire = w;
    i2cAddress = addr;
}

MPU9250::MPU9250(TwoWire *w){
    _wire = w;
    i2cAddress = 0x68;
}

//************************ Power Management

void MPU9250::sleep(bool sleep){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(sleep){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

void MPU9250::enableCycle(bool cycle){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(cycle){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

void MPU9250::enableGyrStandby(bool gyroStandby){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(gyroStandby){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

//************************ Settings

void MPU9250::autoOffsets(){
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    
    enableGyrDLPF();
    setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
    setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
    setAccRange(MPU9250_ACC_RANGE_2G);
    enableAccDLPF(true);
    setAccDLPF(MPU9250_DLPF_6);
    delay(100);
    
    for(int i=0; i<50; i++){
        getAccRawValues();
        accOffsetVal.x += accRawVal.x;
        accOffsetVal.y += accRawVal.y;
        accOffsetVal.z += accRawVal.z;
        delay(1);
    }
    
    accOffsetVal.x /= 50;
    accOffsetVal.y /= 50;
    accOffsetVal.z /= 50;
    accOffsetVal.z -= 16384.0;
    
    for(int i=0; i<50; i++){
        getGyrRawValues();
        gyrOffsetVal.x += gyrRawVal.x;
        gyrOffsetVal.y += gyrRawVal.y;
        gyrOffsetVal.z += gyrRawVal.z;
        delay(1);
    }
    
    gyrOffsetVal.x /= 50;
    gyrOffsetVal.y /= 50;
    gyrOffsetVal.z /= 50;
    
}

void MPU9250::setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    accOffsetVal.x = (xMax + xMin) * 0.5;
    accOffsetVal.y = (yMax + yMin) * 0.5;
    accOffsetVal.z = (zMax + zMin) * 0.5;
}

void MPU9250::setGyrOffsets(float xOffset, float yOffset, float zOffset){
    gyrOffsetVal.x = xOffset;
    gyrOffsetVal.y = yOffset;
    gyrOffsetVal.z = zOffset;
}

void MPU9250::setGyrDLPF(MPU9250_dlpf dlpf){
    regVal = readMPU9250Register8(MPU9250_CONFIG);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU9250_CONFIG, regVal);
}

void MPU9250::setSampleRateDivider(uint8_t splRateDiv){
    writeMPU9250Register(MPU9250_SMPLRT_DIV, splRateDiv);
}

void MPU9250::setGyrRange(MPU9250_gyroRange gyroRange){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xE7;
    regVal |= (gyroRange<<3);
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
    gyrRangeFactor = (1<<gyroRange);
}

void MPU9250::enableGyrDLPF(){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xFC;
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250::disableGyrDLPF(MPU9250_bw_wo_dlpf bw){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xFC;
    regVal |= bw;
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250::setAccRange(MPU9250_accRange accRange){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG);
    regVal &= 0xE7;
    regVal |= (accRange<<3);
    writeMPU9250Register(MPU9250_ACCEL_CONFIG, regVal);
    accRangeFactor = 1<<accRange;
}

void MPU9250::enableAccDLPF(bool enable){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG_2);
    if(enable){
        regVal &= ~8;
    }
    else{
        regVal |= 8;
    }
    writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, regVal);
}

void MPU9250::setAccDLPF(MPU9250_dlpf dlpf){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG_2);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, regVal);
}

void MPU9250::setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr){
    writeMPU9250Register(MPU9250_LP_ACCEL_ODR, lpaodr); 
}

void MPU9250::enableAccAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_2);
    regVal &= ~(0x38);
    regVal |= (enable<<3);
    writeMPU9250Register(MPU9250_PWR_MGMT_2, regVal);
}

void MPU9250::enableGyrAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_2);
    regVal &= ~(0x07);
    regVal |= enable;
    writeMPU9250Register(MPU9250_PWR_MGMT_2, regVal);
}

void MPU9250::setMagOpMode(AK09916_opMode opMode){
    writeAK09916Register8(AK09916_CNTL_2, opMode);
    delay(10);
    if(opMode!=AK09916_PWR_DOWN){
        enableMagDataRead(AK09916_HXL, 0x08);
    }
}

//************************ Interrupts

void MPU9250::setIntPinPolarity(MPU9250_intPinPol pol){
    regVal = readMPU9250Register8(MPU9250_INT_PIN_CFG);
    if(pol){
        regVal |= 0x80;
    }
    else{
        regVal &= ~(0x80);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250::enableIntLatch(bool latch){
    regVal = readMPU9250Register8(MPU9250_INT_PIN_CFG);
    if(latch){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250::enableClearIntByAnyRead(bool clearByAnyRead){
    regVal = readMPU9250Register8(MPU9250_INT_PIN_CFG);
    if(clearByAnyRead){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250::enableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU9250_INT_ENABLE);
    regVal |= intType;
    writeMPU9250Register(MPU9250_INT_ENABLE, regVal);   
}

void MPU9250::disableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU9250_INT_ENABLE);
    regVal &= ~intType;
    writeMPU9250Register(MPU9250_INT_ENABLE, regVal);
}

bool MPU9250::checkInterrupt(uint8_t source, MPU9250_intType type){
    source &= type;
    return source;
}

uint8_t MPU9250::readAndClearInterrupts(){
    regVal = readMPU9250Register8(MPU9250_INT_STATUS);
    return regVal;
}

void MPU9250::setWakeOnMotionThreshold(uint8_t womthresh){
    writeMPU9250Register(MPU9250_WOM_THR, womthresh);
}

void MPU9250::enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn){
    regVal = 0;
    if(womEn){
        regVal |= 0x80;
    }
    if(womCompEn){
        regVal |= 0x40;
    }
    writeMPU9250Register(MPU9250_MOT_DET_CTRL, regVal);
}

//************************ FIFO

void MPU9250::startFifo(MPU9250_fifo_type fifo){
    fifoType = fifo;
    writeMPU9250Register(MPU9250_FIFO_EN, fifoType);
}

void MPU9250::stopFifo(){
    writeMPU9250Register(MPU9250_FIFO_EN, 0);
}

void MPU9250::enableFifo(bool fifo){
    regVal = readMPU9250Register8(MPU9250_USER_CTRL);
    if(fifo){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_USER_CTRL, regVal);
}

void MPU9250::resetFifo(){
    regVal = readMPU9250Register8(MPU9250_USER_CTRL);
    regVal |= 0x04;
    writeMPU9250Register(MPU9250_USER_CTRL, regVal);
}

int16_t MPU9250::getFifoCount(){
    uint16_t regVal16 = (uint16_t) readMPU9250Register16(MPU9250_FIFO_COUNT);
    return regVal16;
}

void MPU9250::setFifoMode(MPU9250_fifoMode mode){
    regVal = readMPU9250Register8(MPU9250_CONFIG);
    if(mode){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_CONFIG, regVal);
    
}

int16_t MPU9250::getNumberOfFifoDataSets(){
    int16_t numberOfSets = getFifoCount();
        
    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        numberOfSets /= 6;
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        numberOfSets /= 12;
    }
    
    return numberOfSets;
}

void MPU9250::findFifoBegin(){
    int16_t count = getFifoCount();
        
    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        if(count > 510){
            for(int i=0; i<2; i++){
                readMPU9250Register8(MPU9250_FIFO_R_W);
            }
        }
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        if(count > 504){
            for(int i=0; i<8; i++){
                readMPU9250Register8(MPU9250_FIFO_R_W);
            }
        }
    }
}

//************************ Results

xyzFloat MPU9250::getAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;
     
    return accRawVal;
}

xyzFloat MPU9250::getCorrectedAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;
    
    correctAccRawValues();
    
    return accRawVal;
}

xyzFloat MPU9250::getGValues(){
    xyzFloat gVal;
    getCorrectedAccRawValues();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

xyzFloat MPU9250::getAccRawValuesFromFifo(){
    xyzFloat accRawVal = readMPU9250xyzValFromFifo();
    return accRawVal;   
}

xyzFloat MPU9250::getCorrectedAccRawValuesFromFifo(){
    accRawVal = getAccRawValuesFromFifo();
    
    correctAccRawValues();
    
    return accRawVal;
}

xyzFloat MPU9250::getGValuesFromFifo(){
    xyzFloat gVal;
    getCorrectedAccRawValuesFromFifo();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

float MPU9250::getResultantG(xyzFloat gVal){
    float resultant = 0.0;
    resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));
    
    return resultant;
}

float MPU9250::getTemperature(){
    int16_t regVal16 = readMPU9250Register16(MPU9250_TEMP_OUT);
    float tmp = (regVal16*1.0 - MPU9250_ROOM_TEMP_OFFSET)/MPU9250_T_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat MPU9250::getGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;
     
    return gyrRawVal;
}

xyzFloat MPU9250::getCorrectedGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;
     
    correctGyrRawValues();
    
    return gyrRawVal;
}

xyzFloat MPU9250::getGyrValues(){
    xyzFloat gyrVal;
    getCorrectedGyrRawValues();
    
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;
     
    return gyrVal;
}

xyzFloat MPU9250::getGyrValuesFromFifo(){
    xyzFloat gyrVal;
    xyzFloat gyrRawVal = readMPU9250xyzValFromFifo();
    
    correctGyrRawValues();
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;
    
    return gyrVal;
}

xyzFloat MPU9250::getMagValues(){
    int16_t x,y,z;
    xyzFloat mag;
    
    x = (int16_t)((buffer[15]) << 8) | buffer[14];
    y = (int16_t)((buffer[17]) << 8) | buffer[16];
    z = (int16_t)((buffer[19]) << 8) | buffer[18];
    
    mag.x = x * AK09916_MAG_LSB;
    mag.y = y * AK09916_MAG_LSB;
    mag.z = z * AK09916_MAG_LSB;
    
    return mag;
}

xyzFloat MPU9250::getAngles(){
    xyzFloat angleVal;
    xyzFloat gVal = getGValues();
    if(gVal.x > 1.0){
        gVal.x = 1.0;
    }
    else if(gVal.x < -1.0){
        gVal.x = -1.0;
    }
    angleVal.x = (asin(gVal.x)) * 57.296;
    
    if(gVal.y > 1.0){
        gVal.y = 1.0;
    }
    else if(gVal.y < -1.0){
        gVal.y = -1.0;
    }
    angleVal.y = (asin(gVal.y)) * 57.296;
    
    if(gVal.z > 1.0){
        gVal.z = 1.0;
    }
    else if(gVal.z < -1.0){
        gVal.z = -1.0;
    }
    angleVal.z = (asin(gVal.z)) * 57.296;
    
    return angleVal;
}

MPU9250_orientation MPU9250::getOrientation(){
    xyzFloat angleVal = getAngles();
    MPU9250_orientation orientation = MPU9250_FLAT;
    if(abs(angleVal.x) < 45){      // |x| < 45
        if(abs(angleVal.y) < 45){      // |y| < 45
            if(angleVal.z > 0){          //  z  > 0
                orientation = MPU9250_FLAT;
            }
            else{                        //  z  < 0
                orientation = MPU9250_FLAT_1;
            }
        }
        else{                         // |y| > 45 
            if(angleVal.y > 0){         //  y  > 0
                orientation = MPU9250_XY;
            }
            else{                       //  y  < 0
                orientation = MPU9250_XY_1;   
            }
        }
    }
    else{                           // |x| >= 45
        if(angleVal.x > 0){           //  x  >  0
            orientation = MPU9250_YX;       
        }
        else{                       //  x  <  0
            orientation = MPU9250_YX_1;
        }
    }
    return orientation;
}

String MPU9250::getOrientationAsString(){
    MPU9250_orientation orientation = getOrientation();
    String orientationAsString = "";
    switch(orientation){
        case MPU9250_FLAT:      orientationAsString = "z up";   break;
        case MPU9250_FLAT_1:    orientationAsString = "z down"; break;
        case MPU9250_XY:        orientationAsString = "y up";   break;
        case MPU9250_XY_1:      orientationAsString = "y down"; break;
        case MPU9250_YX:        orientationAsString = "x up";   break;
        case MPU9250_YX_1:      orientationAsString = "x down"; break;
    }
    return orientationAsString;
}

float MPU9250::getPitch(){
    xyzFloat angleVal = getAngles();
    float pitch = (atan2(angleVal.x, sqrt(abs((angleVal.x*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
    return pitch;
}
    
float MPU9250::getRoll(){
    xyzFloat angleVal = getAngles();
    float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    return roll;
}

/**
 * @brief Magnetometer identification
 * 
 * @return int16_t 
 */
int16_t MPU9250::whoAmIMag(){
    return readAK09916Register16(AK09916_WIA_1);
}


void MPU9250::readSensor(){
    readAllData(buffer);
}


//************************ Basic

bool MPU9250::init(){ 
    if(!reset_MPU9250()){
       return false;
    }
    delay(100);
    writeMPU9250Register(MPU9250_INT_PIN_CFG, MPU9250_BYPASS_EN);  // Bypass Enable
    delay(100);
    
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    accRangeFactor = 1;
    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;
    gyrRangeFactor = 1;
    fifoType = MPU9250_FIFO_ACC;
    sleep(false);
    
    return true;
}

/**
 * @brief Initialize the Magnetometer
 * 
 * @return true 
 * @return false 
 */
bool MPU9250::initMagnetometer(){
    enableI2CMaster();
    resetMagnetometer();
    //reset_ICM20948();
    sleep(false);
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR 
    enableI2CMaster();
    
    
    if(!(whoAmIMag() == AK09916_WHO_AM_I)){
        return false;
    }
    
    setMagOpMode(AK09916_CONT_MODE_100HZ); 
   
    return true;
}

/**
 * @brief Reset Magnetometer
 * 
 */
void MPU9250::resetMagnetometer(){
    writeAK09916Register8(AK09916_CNTL_3, 0x01);
    delay(100);
}

/**
 * @brief Initialize everything
 * 
 * @return true 
 * @return false 
 */
bool MPU9250::begin(){ 
    if(!initMagnetometer()){
       return false;
    }

    return true;
}

/************************************************ 
     Private Functions
*************************************************/

uint8_t MPU9250::writeMPU9250Register(uint8_t reg, uint8_t val){
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(val);
    
    return _wire->endTransmission();
}

/**
 * @brief  Write the 8-bit registers
 * 
 * @param bank 
 * @param reg 
 * @param val 
 * @return uint8_t 
 */
uint8_t MPU9250::writeRegister8(uint8_t bank, uint8_t reg, uint8_t val){
    _wire->beginTransmission(0x68);
    //_wire->beginTransmission(i2cAddress);
    
    _wire->write(reg);
    _wire->write(val);
    
    return _wire->endTransmission();
}

/**
 * @brief Write the 8-bit registers to the AK09916 (magnetometer)
 * 
 * @param reg 
 * @param val 
 */
void MPU9250::writeAK09916Register8(uint8_t reg, uint8_t val){
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS); // write AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be written to
    writeRegister8(3, ICM20948_I2C_SLV0_DO, val);
}

uint8_t MPU9250::writeMPU9250Register16(uint8_t reg, int16_t val){
    int8_t MSByte = (val>>7) & 0xFF;
    uint8_t LSByte = (val<<1) & 0xFE;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(MSByte);
    _wire->write(LSByte);
    
    return _wire->endTransmission();  
}

int16_t MPU9250::readRegister16(uint8_t bank, uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t reg16Val = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,2);
    if(_wire->available()){
        MSByte = _wire->read();
        LSByte = _wire->read();
    }
    reg16Val = (MSByte<<8) + LSByte;
    return reg16Val;
}

uint8_t MPU9250::readAK09916Register8(uint8_t reg){
    enableMagDataRead(reg, 0x01);
    enableMagDataRead(AK09916_HXL, 0x08);
    regVal = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);
    return regVal;
}

int16_t MPU9250::readAK09916Register16(uint8_t reg){
    int16_t regValue = 0;
    enableMagDataRead(reg, 0x02);
    regValue = readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00);
    enableMagDataRead(AK09916_HXL, 0x08);
    return regValue;
}

uint8_t MPU9250::readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,1);
    if(_wire->available()){
        regValue = _wire->read();
    }
    return regValue;
}

int16_t MPU9250::readMPU9250Register16(uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t regValue = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,2);
    if(_wire->available()){
        MSByte = _wire->read();
        LSByte = _wire->read();
    }
    regValue = (MSByte<<8) + LSByte;
    return regValue;
}

uint64_t MPU9250::readMPU9250Register3x16(uint8_t reg){    
    uint8_t byte0 = 0, byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0, byte5 = 0;
    uint64_t regValue = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,6);
    if(_wire->available()){
        byte0 = _wire->read();
        byte1 = _wire->read();
        byte2 = _wire->read();
        byte3 = _wire->read();
        byte4 = _wire->read();
        byte5 = _wire->read();
    }
    regValue = ((uint64_t) byte0<<40) + ((uint64_t) byte1<<32) +((uint64_t) byte2<<24) + 
           + ((uint64_t) byte3<<16) + ((uint64_t) byte4<<8) +  (uint64_t)byte5;
    return regValue;
}

xyzFloat MPU9250::readMPU9250xyzValFromFifo(){
    uint8_t MSByte = 0, LSByte = 0;
    xyzFloat xyzResult = {0.0, 0.0, 0.0};
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.x = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.y = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.z = ((int16_t)((MSByte<<8) + LSByte)) * 1.0;
    return xyzResult; 
}

void MPU9250::readAllData(uint8_t* data){
    _wire->beginTransmission(i2cAddress);
    _wire->write(ICM20948_ACCEL_OUT);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,20);
    if(_wire->available()){
        for(int i=0; i<20; i++){
            data[i] = _wire->read();
        }
    }
}

// void MPU9250::enableI2CMaster(){
//     writeRegister8(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN); //enable I2C master
//     writeRegister8(3, ICM20948_I2C_MST_CTRL, 0x07); // set I2C clock to 345.60 kHz
//     delay(10);
// }
void MPU9250::enableI2CMaster(){
    regVal = readMPU9250Register8(MPU9250_USER_CTRL);
    regVal |= MPU9250_I2C_MST_EN;
    writeMPU9250Register(MPU9250_USER_CTRL, regVal); //enable I2C master
    writeMPU9250Register(MPU9250_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
    delay(10);
}
  
void MPU9250::enableMagDataRead(uint8_t reg, uint8_t bytes){
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ); // read AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be read
    writeRegister8(3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
    delay(10);
}


void MPU9250::correctAccRawValues(){
    accRawVal.x -= (accOffsetVal.x / accRangeFactor);
    accRawVal.y -= (accOffsetVal.y / accRangeFactor);
    accRawVal.z -= (accOffsetVal.z / accRangeFactor);
}

void MPU9250::correctGyrRawValues(){
    gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
    gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
    gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);
}

uint8_t MPU9250::reset_MPU9250(){
    uint8_t ack = writeMPU9250Register(MPU9250_PWR_MGMT_1, MPU9250_RESET);
    delay(10);  // wait for registers to reset
    return (ack == 0);
}




// void MPU9250::resetMagnetometer(){
//     writeAK8963Register(AK8963_CNTL_2, 0x01);
//     delay(100);
// }
// void MPU9250::getAsaVals(){
//     byte rawCorr = 0;
//     rawCorr = readAK8963Register8(AK8963_ASAX);
//     magCorrFactor.x = (0.5 * (rawCorr-128)/128.0) + 1.0;
//     rawCorr = readAK8963Register8(AK8963_ASAY);
//     magCorrFactor.y = (0.5 * (rawCorr-128)/128.0) + 1.0;
//     rawCorr = readAK8963Register8(AK8963_ASAZ);
//     magCorrFactor.z = (0.5 * (rawCorr-128)/128.0) + 1.0;
// }
// void MPU9250::enableMagDataRead(uint8_t reg, uint8_t bytes){
//     writeMPU9250Register(MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS | AK8963_READ); // read AK8963
//     writeMPU9250Register(MPU9250_I2C_SLV0_REG, reg); // define AK8963 register to be read
//     writeMPU9250Register(MPU9250_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
//     delay(10);
// }
// void MPU9250::writeAK8963Register(uint8_t reg, uint8_t val){
//     writeMPU9250Register(MPU9250_I2C_SLV0_ADDR, AK8963_ADDRESS); // write AK8963
//     writeMPU9250Register(MPU9250_I2C_SLV0_REG, reg); // define AK8963 register to be written to
//     writeMPU9250Register(MPU9250_I2C_SLV0_DO, val);
// }


// uint8_t MPU9250::readAK8963Register8(uint8_t reg){
//     enableMagDataRead(reg, 0x01);
//     regVal = readMPU9250Register8(MPU9250_EXT_SLV_SENS_DATA_00);
//     enableMagDataRead(AK8963_HXL, 0x08);
    
//     return regVal;
// }


// uint64_t MPU9250::readAK8963Data(){    
//     uint8_t byte0 = 0, byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0, byte5 = 0;
//     uint64_t regValue = 0;
    
//     _wire->beginTransmission(i2cAddress);
//     _wire->write(MPU9250_EXT_SLV_SENS_DATA_00);
//     _wire->endTransmission(false);
//     _wire->requestFrom(i2cAddress,6);
//     if(_wire->available()){
//         byte0 = _wire->read();
//         byte1 = _wire->read();
//         byte2 = _wire->read();
//         byte3 = _wire->read();
//         byte4 = _wire->read();
//         byte5 = _wire->read();
//     }
//     regValue = ((uint64_t) byte1<<40) + ((uint64_t) byte0<<32) +((uint64_t) byte3<<24) + 
//            + ((uint64_t) byte2<<16) + ((uint64_t) byte5<<8) +  (uint64_t)byte4;
//     return regValue;
// }

// void MPU9250::setMagnetometer16Bit(){
//     regVal = readAK8963Register8(AK8963_CNTL_1);
//     regVal |= AK8963_16_BIT;
//     writeAK8963Register(AK8963_CNTL_1, regVal);
// }
// uint8_t MPU9250::getStatus2Register(){
//     regVal = readAK8963Register8(AK8963_STATUS_2);
//     return regVal;
// }


