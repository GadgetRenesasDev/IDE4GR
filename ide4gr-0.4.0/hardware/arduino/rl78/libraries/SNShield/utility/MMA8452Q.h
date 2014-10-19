

#ifndef MMA8452Q_h
#define MMA8452Q_h

#define MMA8452_ADDRESS 0x1D  // 0x1D if SA0 is high, 0x1C if low

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB     0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I      0x0D
#define CTRL_REG1     0x2A

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.

bool initMMA8452();
void readAccelData(float *);
void readRegisters(byte , int , byte * );
byte readRegister(byte);
void writeRegister(byte , byte );
void MMA8452Standby();
void MMA8452Active();

#endif //MMA8452Q_h
