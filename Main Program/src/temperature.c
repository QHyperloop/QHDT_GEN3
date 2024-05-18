#include "temperature.h"

#define ADS1115_ID0 0x48
#define ADS1115_ID1 0x49
#define MCP9600_ID0 0x67
#define MCP9600_ID1 0x66
#define MCP9600_ID2 0x65
#define MCP9600_ID3 0x60
#define MCP_CONFIG 0x06
#define MCP_THERMO 0x05
#define ADS_CONFIG 0x01
#define ADS_CONVER 0x00
#define ADS_CON1 0xC583
#define ADS_CON2 0xD583
#define ADS_CON3 0xE583
#define ADS_CON4 0xF583

float TEMP_ADS[8];
float TEMP_MCP[4];

int MCP[4];
int ADS[2];
int temp;
#define I2C_BUS 0
#define I2C_FLAGS 0


error_handler TEMP_INIT(){
    MCP[0] = i2cOpen(I2C_BUS, MCP9600_ID0, I2C_FLAGS);
    MCP[1] = i2cOpen(I2C_BUS, MCP9600_ID1, I2C_FLAGS);
    MCP[2] = i2cOpen(I2C_BUS, MCP9600_ID2, I2C_FLAGS);
    MCP[3] = i2cOpen(I2C_BUS, MCP9600_ID3, I2C_FLAGS);
    ADS[0] = i2cOpen(I2C_BUS, ADS1115_ID0, I2C_FLAGS);
    ADS[1] = i2cOpen(I2C_BUS, ADS1115_ID1, I2C_FLAGS);

    if(i2cWriteByteData(MCP[0], MCP_CONFIG, 0b00100000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[0], MCP_THERMO, 0b00000000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[1], MCP_CONFIG, 0b00100000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[1], MCP_THERMO, 0b00000000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[2], MCP_CONFIG, 0b00100000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[2], MCP_THERMO, 0b00000000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[3], MCP_CONFIG, 0b00100000)<0){
        return MCP_CONFIG_FAIL;
    }
    if(i2cWriteByteData(MCP[3], MCP_THERMO, 0b00000000)<0){
        return MCP_CONFIG_FAIL;
    }

    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON1)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON2)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON3)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON4)<0){
        return ADS_CONFIG_FAIL;
    }
    
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON1)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON2)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON3)<0){
        return ADS_CONFIG_FAIL;
    }
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON4)<0){
        return ADS_CONFIG_FAIL;
    }
    return TEMP_INIT_SUCCESS;

}

error_handler UPDATE_TEMP(){
    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON1)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[0] =  i2cReadWordData(ADS[0], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON2)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[1] =  i2cReadWordData(ADS[0], ADS_CONVER)/(2^16);
    
    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON3)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[2] =  i2cReadWordData(ADS[0], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[0], ADS_CONFIG, ADS_CON4)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[3] =  i2cReadWordData(ADS[0], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON1)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[4] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON2)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[5] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);
    
    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON3)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[6] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);

    if(i2cWriteWordData(ADS[1], ADS_CONFIG, ADS_CON4)<0){
        return ADS_CONFIG_FAIL;
    }
    TEMP_ADS[7] =  i2cReadWordData(ADS[1], ADS_CONVER)/(2^16);


    temp = i2cReadWordData(MCP[0],0x00);

    TEMP_MCP[0] = ((((uint_8t)temp & 0xFF00)>>8)*16.0 + ((uint_8t)temp & 0xFF)/16.0);

    temp = i2cReadWordData(MCP[1],0x00);

    TEMP_MCP[1] = ((((uint_8t)temp & 0xFF00)>>8)*16.0 + ((uint_8t)temp & 0xFF)/16.0);

    temp = i2cReadWordData(MCP[2],0x00);

    TEMP_MCP[2] = ((((uint_8t)temp & 0xFF00)>>8)*16.0 + ((uint_8t)temp & 0xFF)/16.0);

    temp = i2cReadWordData(MCP[3],0x00);

    TEMP_MCP[3] = ((((uint_8t)temp & 0xFF00)>>8)*16.0 + ((uint_8t)temp & 0xFF)/16.0);


}