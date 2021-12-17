/**
 * @brief Driver for InvenSense MPU-9250/6500/9255
 * @file MPU9250.h
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

#ifndef MPU9250_H_
#define MPU9250_H_

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#ifndef MPU9250_AADDRESS
#define MPU9250_ADDRESS 0x68
#endif
#ifndef AK8963_ADDRESS
#define AK8963_ADDRESS 0x0C
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Registers MPU9250 version 1.6 01/07/2015 */
// Register Mapfor Gyroscope and Accelerometer
#define MPU9250_SELF_TEST_X_GYRO		0x00
#define MPU9250_SELF_TEST_Y_GYRO		0x01
#define MPU9250_SELF_TEST_Z_GYRO		0x02
#define MPU9250_SELF_TEST_X_ACCEL		0x0D
#define MPU9250_SELF_TEST_Y_ACCEL		0x0E
#define MPU9250_SELF_TEST_Z_ACCEL		0x0F
#define MPU9250_XG_OFFSET_H		        0x13
#define MPU9250_XG_OFFSET_L		        0x14
#define MPU9250_YG_OFFSET_H		        0x15
#define MPU9250_YG_OFFSET_L		        0x16
#define MPU9250_ZG_OFFSET_H		        0x17
#define MPU9250_ZG_OFFSET_L		        0x18
#define MPU9250_SMPLRT_DIV		        0x19
#define MPU9250_CONFIG		            0x1A
#define MPU9250_GYRO_CONFIG		        0x1B
#define MPU9250_ACCEL_CONFIG		    0x1C
#define MPU9250_ACCEL_CONFIG_2		    0x1D
#define MPU9250_LP_ACCEL_ODR		    0x1E
#define MPU9250_WOM_THR		            0x1F
#define MPU9250_FIFO_EN		            0x23
#define MPU9250_I2C_MST_CTRL		    0x24
#define MPU9250_I2C_SLV0_ADDR		    0x25
#define MPU9250_I2C_SLV0_REG		    0x26
#define MPU9250_I2C_SLV0_CTRL		    0x27
#define MPU9250_I2C_SLV1_ADDR		    0x28
#define MPU9250_I2C_SLV1_REG		    0x29
#define MPU9250_I2C_SLV1_CTRL		    0x2A
#define MPU9250_I2C_SLV2_ADDR		    0x2B
#define MPU9250_I2C_SLV2_REG		    0x2C
#define MPU9250_I2C_SLV2_CTRL		    0x2D
#define MPU9250_I2C_SLV3_ADDR		    0x2E
#define MPU9250_I2C_SLV3_REG		    0x2F
#define MPU9250_I2C_SLV3_CTRL		    0x30
#define MPU9250_I2C_SLV4_ADDR		    0x31
#define MPU9250_I2C_SLV4_REG		    0x32
#define MPU9250_I2C_SLV4_DO		        0x33
#define MPU9250_I2C_SLV4_CTRL		    0x34
#define MPU9250_I2C_SLV4_DI		        0x35
#define MPU9250_I2C_MST_STATUS		    0x36
#define MPU9250_INT_PIN_CFG		        0x37
#define MPU9250_INT_ENABLE		        0x38
#define MPU9250_INT_STATUS		        0x3A
#define MPU9250_ACCEL_XOUT_H		    0x3B
#define MPU9250_ACCEL_XOUT_L		    0x3C
#define MPU9250_ACCEL_YOUT_H		    0x3D
#define MPU9250_ACCEL_YOUT_L		    0x3E
#define MPU9250_ACCEL_ZOUT_H		    0x3F
#define MPU9250_ACCEL_ZOUT_L		    0x40
#define MPU9250_TEMP_OUT_H		        0x41
#define MPU9250_TEMP_OUT_L		        0x42
#define MPU9250_GYRO_XOUT_H		        0x43
#define MPU9250_GYRO_XOUT_L		        0x44
#define MPU9250_GYRO_YOUT_H		        0x45
#define MPU9250_GYRO_YOUT_L		        0x46
#define MPU9250_GYRO_ZOUT_H		        0x47
#define MPU9250_GYRO_ZOUT_L		        0x48
#define MPU9250_EXT_SENS_DATA_00		0x49
#define MPU9250_EXT_SENS_DATA_01		0x4A
#define MPU9250_EXT_SENS_DATA_02		0x4B
#define MPU9250_EXT_SENS_DATA_03		0x4C
#define MPU9250_EXT_SENS_DATA_04		0x4D
#define MPU9250_EXT_SENS_DATA_05		0x4E
#define MPU9250_EXT_SENS_DATA_06		0x4F
#define MPU9250_EXT_SENS_DATA_07		0x50
#define MPU9250_EXT_SENS_DATA_08		0x51
#define MPU9250_EXT_SENS_DATA_09		0x52
#define MPU9250_EXT_SENS_DATA_10		0x53
#define MPU9250_EXT_SENS_DATA_11		0x54
#define MPU9250_EXT_SENS_DATA_12		0x55
#define MPU9250_EXT_SENS_DATA_13		0x56
#define MPU9250_EXT_SENS_DATA_14		0x57
#define MPU9250_EXT_SENS_DATA_15		0x58
#define MPU9250_EXT_SENS_DATA_16		0x59
#define MPU9250_EXT_SENS_DATA_17		0x5A
#define MPU9250_EXT_SENS_DATA_18		0x5B
#define MPU9250_EXT_SENS_DATA_19		0x5C
#define MPU9250_EXT_SENS_DATA_20		0x5D
#define MPU9250_EXT_SENS_DATA_21		0x5E
#define MPU9250_EXT_SENS_DATA_22		0x5F
#define MPU9250_EXT_SENS_DATA_23		0x60
#define MPU9250_I2C_SLV0_DO		        0x63
#define MPU9250_I2C_SLV1_DO		        0x64
#define MPU9250_I2C_SLV2_DO		        0x65
#define MPU9250_I2C_SLV3_DO		        0x66
#define MPU9250_I2C_MST_DELAY_CTRL		0x67
#define MPU9250_SIGNAL_PATH_RESET		0x68
#define MPU9250_MOT_DETECT_CTRL		    0x69
#define MPU9250_USER_CTRL		        0x6A
#define MPU9250_PWR_MGMT_1		        0x6B
#define MPU9250_PWR_MGMT_2		        0x6C
#define MPU9250_FIFO_COUNTH		        0x72
#define MPU9250_FIFO_COUNTL		        0x73
#define MPU9250_FIFO_R_W		        0x74
#define MPU9250_WHO_AM_I		        0x75
#define MPU9250_XA_OFFSET_H		        0x77
#define MPU9250_XA_OFFSET_L		        0x78
#define MPU9250_YA_OFFSET_H		        0x7A
#define MPU9250_YA_OFFSET_L		        0x7B
#define MPU9250_ZA_OFFSET_H		        0x7D
#define MPU9250_ZA_OFFSET_L		        0x7E

#define MPU9250_MOT_DET_CTRL            0x69
#define MPU9250_FIFO_COUNT              0x72 // 0x72 is COUNT_H
#define MPU9250_ACCEL_OUT               0x3B // accel data registers begin
#define MPU9250_TEMP_OUT                0x41

#define MPU9250_ROOM_TEMP_OFFSET        0.0f
#define MPU9250_T_SENSITIVITY           333.87f
#define AK8963_WHO_AM_I                 0x48

#define MPU9250_RESET                   0x80
#define MPU9250_BYPASS_EN               0x02
#define MPU9250_CLK_SEL_PLL             0x01
#define AK8963_16_BIT                   0x10
#define AK8963_OVF                      0x08
#define AK8963_READ                     0x80

// Register Map for Magnetometer
#define MPU9250_I2C_MST_EN              0x20
#define ICM20948_I2C_SLV0_ADDR          0x03
#define ICM20948_I2C_SLV0_REG           0x04
#define ICM20948_I2C_SLV0_CTRL          0x05
#define ICM20948_I2C_SLV0_DO            0x06
#define ICM20948_EXT_SLV_SENS_DATA_00   0x3B
#define ICM20948_USER_CTRL              0x03
#define ICM20948_I2C_MST_EN             0x20
#define ICM20948_I2C_MST_CTRL           0x01
#define ICM20948_ACCEL_OUT              0x2D // accel data registers begin
#define ICM20948_REG_BANK_SEL           0x7F
#define ICM20948_ODR_ALIGN_EN           0x09

#define AK09916_ADDRESS                 0x0C
#define AK09916_WIA_1                   0x00 // Who I am, Company ID
#define AK09916_WIA_2                   0x01 // Who I am, Device ID
#define MPU9250_GYRO_OUT                0x43 // gyro data registers begin
#define AK09916_STATUS_1                0x10 
#define AK09916_HXL                     0x11
#define AK09916_HXH                     0x12
#define AK09916_HYL                     0x13
#define AK09916_HYH                     0x14
#define AK09916_HZL                     0x15
#define AK09916_HZH                     0x16
#define AK09916_STATUS_2                0x18
#define AK09916_CNTL_2                  0x31
#define AK09916_CNTL_3                  0x32

#define AK09916_16_BIT                  0x10
#define AK09916_OVF                     0x08
#define AK09916_READ                    0x80

#define AK09916_WHO_AM_I                0x48

#define AK09916_MAG_LSB                 0.1495f

typedef enum MPU9250_BW_WO_DLPF {
    MPU9250_BW_WO_DLPF_3600 = 0x02, 
    MPU9250_BW_WO_DLPF_8800 = 0x01
} MPU9250_bw_wo_dlpf;

typedef enum MPU9250_DLPF {
    MPU9250_DLPF_0, MPU9250_DLPF_1, MPU9250_DLPF_2, MPU9250_DLPF_3, MPU9250_DLPF_4, MPU9250_DLPF_5, 
    MPU9250_DLPF_6, MPU9250_DLPF_7
} MPU9250_dlpf;

typedef enum MPU9250_GYRO_RANGE {
    MPU9250_GYRO_RANGE_250, MPU9250_GYRO_RANGE_500, MPU9250_GYRO_RANGE_1000, MPU9250_GYRO_RANGE_2000
} MPU9250_gyroRange;

typedef enum MPU9250_ACC_RANGE {
    MPU9250_ACC_RANGE_2G, MPU9250_ACC_RANGE_4G, MPU9250_ACC_RANGE_8G, MPU9250_ACC_RANGE_16G
} MPU9250_accRange;

typedef enum MPU9250_LOW_PWR_ACC_ODR {
    MPU9250_LP_ACC_ODR_0_24, MPU9250_LP_ACC_ODR_0_49, MPU9250_LP_ACC_ODR_0_98, MPU9250_LP_ACC_ODR_1_95,
    MPU9250_LP_ACC_ODR_3_91, MPU9250_LP_ACC_ODR_7_81, MPU9250_LP_ACC_ODR_15_63, MPU9250_LP_ACC_ODR_31_25,
    MPU9250_LP_ACC_ODR_62_5, MPU9250_LP_ACC_ODR_125, MPU9250_LP_ACC_ODR_250, MPU9250_LP_ACC_ODR_500
} MPU9250_lpAccODR;

typedef enum MPU9250_INT_PIN_POL {
    MPU9250_ACT_HIGH, MPU9250_ACT_LOW
} MPU9250_intPinPol;

typedef enum MPU9250_INT_TYPE {
    MPU9250_DATA_READY = 0x01,
    MPU9250_FIFO_OVF   = 0x10,
    MPU9250_WOM_INT    = 0x40
} MPU9250_intType;

typedef enum MPU9250_WOM_EN {
    MPU9250_WOM_DISABLE, MPU9250_WOM_ENABLE
} MPU9250_womEn;

typedef enum MPU9250_WOM_COMP {
    MPU9250_WOM_COMP_DISABLE, MPU9250_WOM_COMP_ENABLE
} MPU9250_womCompEn;

typedef enum MPU9250_XYZ_ENABLE {
    MPU9250_ENABLE_XYZ,  //all axes are enabled (default)
    MPU9250_ENABLE_XY0,  // x, y enabled, z disabled
    MPU9250_ENABLE_X0Z,   
    MPU9250_ENABLE_X00,
    MPU9250_ENABLE_0YZ,
    MPU9250_ENABLE_0Y0,
    MPU9250_ENABLE_00Z,
    MPU9250_ENABLE_000,  // all axes disabled
} MPU9250_xyzEn;

typedef enum MPU9250_ORIENTATION {
  MPU9250_FLAT, MPU9250_FLAT_1, MPU9250_XY, MPU9250_XY_1, MPU9250_YX, MPU9250_YX_1
} MPU9250_orientation;

typedef enum MPU9250_FIFO_MODE {
    MPU9250_CONTINUOUS, MPU9250_STOP_WHEN_FULL
} MPU9250_fifoMode;

typedef enum MPU9250_FIFO_TYPE {
    MPU9250_FIFO_ACC        = 0x08,
    MPU9250_FIFO_GYR        = 0x70,
    MPU9250_FIFO_ACC_GYR    = 0x78
} MPU9250_fifo_type;

typedef enum AK09916_OP_MODE {
    AK09916_PWR_DOWN           = 0x00,
    AK09916_TRIGGER_MODE       = 0x01,
    AK09916_CONT_MODE_10HZ     = 0x02,
    AK09916_CONT_MODE_20HZ     = 0x04,
    AK09916_CONT_MODE_50HZ     = 0x06,
    AK09916_CONT_MODE_100HZ    = 0x08
} AK09916_opMode;

struct xyzFloat {
    float x;
    float y;
    float z;
};

class MPU9250
{
public:
    //************************ Constructors
    MPU9250(int addr);
    MPU9250();
    MPU9250(TwoWire *w, int addr);
    MPU9250(TwoWire *w);
    //************************ Power Management
    void sleep(bool sleep);
    void enableCycle(bool cycle);
    void enableGyrStandby(bool gyroStandby);
    //************************ Settings
    void autoOffsets();
    void setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    void setGyrOffsets(float xOffset, float yOffset, float zOffset);
    void setGyrDLPF(MPU9250_dlpf dlpf);
    void setSampleRateDivider(uint8_t splRateDiv);
    void setGyrRange(MPU9250_gyroRange gyroRange);
    void enableGyrDLPF();
    void disableGyrDLPF(MPU9250_bw_wo_dlpf bw);
    void setAccRange(MPU9250_accRange accRange);
    void enableAccDLPF(bool enable);
    void setAccDLPF(MPU9250_dlpf dlpf);
    void setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr);
    void enableAccAxes(MPU9250_xyzEn enable);
    void enableGyrAxes(MPU9250_xyzEn enable);
    void setMagOpMode(AK09916_opMode opMode);
    //************************ Interrupts
    void setIntPinPolarity(MPU9250_intPinPol pol);
    void enableIntLatch(bool latch);
    void enableClearIntByAnyRead(bool clearByAnyRead);
    void enableInterrupt(MPU9250_intType intType);
    void disableInterrupt(MPU9250_intType intType);
    bool checkInterrupt(uint8_t source, MPU9250_intType type);
    uint8_t readAndClearInterrupts();
    void setWakeOnMotionThreshold(uint8_t womthresh);
    void enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn);
    //************************ FIFO
    void startFifo(MPU9250_fifo_type fifo);
    void stopFifo();
    void enableFifo(bool fifo);
    void resetFifo();
    int16_t getFifoCount();
    void setFifoMode(MPU9250_fifoMode mode);
    int16_t getNumberOfFifoDataSets();
    void findFifoBegin();
    //************************ Results
    xyzFloat getAccRawValues();
    xyzFloat getCorrectedAccRawValues();
    xyzFloat getGValues();
    xyzFloat getAccRawValuesFromFifo();
    xyzFloat getCorrectedAccRawValuesFromFifo();
    xyzFloat getGValuesFromFifo();
    float getResultantG(xyzFloat gVal); 
    float getTemperature();
    xyzFloat getGyrRawValues();
    xyzFloat getCorrectedGyrRawValues();
    xyzFloat getGyrValues(); 
    xyzFloat getGyrValuesFromFifo();
    xyzFloat getMagValues();
    xyzFloat getAngles();
    MPU9250_orientation getOrientation();
    String getOrientationAsString();
    float getPitch();
    float getRoll();
    int16_t whoAmIMag();
    void readSensor();
void startMagMeasurement();
    //************************ Basic
    bool begin();
    bool init();
    bool initMagnetometer();
private:
    //************************ Constructors
    TwoWire *_wire;
    int i2cAddress;

    uint8_t writeMPU9250Register(uint8_t reg, uint8_t val);
    uint8_t writeRegister8(uint8_t bank, uint8_t reg, uint8_t val);
    void writeAK09916Register8(uint8_t reg, uint8_t val);
    uint8_t writeMPU9250Register16(uint8_t reg, int16_t val);
    int16_t readRegister16(uint8_t bank, uint8_t reg);
    uint8_t readAK09916Register8(uint8_t reg);
    int16_t readAK09916Register16(uint8_t reg);
    uint8_t readMPU9250Register8(uint8_t reg);
    int16_t readMPU9250Register16(uint8_t reg);
    uint64_t readMPU9250Register3x16(uint8_t reg);
    xyzFloat readMPU9250xyzValFromFifo();
    void readAllData(uint8_t* data);
    void enableI2CMaster();
    void enableMagDataRead(uint8_t reg, uint8_t bytes);
    void correctAccRawValues();
    void correctGyrRawValues();
    uint8_t reset_MPU9250();

    void setMagnetometer16Bit();
    uint8_t getStatus2Register();
    void getAsaVals();
    uint8_t readMPU9250Register(uint8_t reg);

    uint8_t writeRegister16(uint8_t bank, uint8_t reg, int16_t val);
    uint8_t readRegister8(uint8_t bank, uint8_t reg);

    xyzFloat accRawVal;
    xyzFloat gyrRawVal;
    xyzFloat accOffsetVal;
    xyzFloat gyrOffsetVal;
    xyzFloat magCorrFactor;
    uint8_t accRangeFactor;
    uint8_t gyrRangeFactor;
    uint8_t regVal;   // intermediate storage of register values
    MPU9250_fifo_type fifoType;
    void resetMagnetometer();
    uint8_t buffer[20]; 
};

#endif
