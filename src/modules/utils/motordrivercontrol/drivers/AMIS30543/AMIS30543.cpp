#include "AMIS30543.h"

#include "Kernel.h"
#include "StreamOutput.h"
#include "StreamOutputPool.h"
#include "ConfigValue.h"
#include "Config.h"
#include "checksumm.h"

#define motor_driver_control_checksum  CHECKSUM("motor_driver_control")

#define REGWRITE    0x00
#define REGREAD     0x80

// constructor
AMIS30543DRV::AMIS30543DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char d) : spi(spi), designator(d)
{
    error_reported.reset();
}

void AMIS30543DRV::init (uint16_t cs)
{
    // read chip specific config entries
    ///this->gain= THEKERNEL->config->value(motor_driver_control_checksum, cs, gain_checksum)->by_default(20)->as_number();
    //this->resistor= THEKERNEL->config->value(motor_driver_control_checksum, cs, sense_resistor_checksum)->by_default(0.05F)->as_number(); // in ohms

    // initialize the in memory mirror of the registers
}

void AMIS30543DRV::set_motorEnable(bool enable)
{
    // Set Enable
    G_CR2_REG.MOTEN = enable ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | G_CR2_REG.Address;
    uint8_t dataLo = (G_CR2_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
  }

void AMIS30543DRV::set_current( uint32_t current )
{   
    uint8_t CUR;
    //Current range 0
    if(current <= 355)
    {
        if( current <= 132)
            CUR= 0;
        else if(current <= 245)
            CUR=1;
        else 
            CUR=2;
    }
    else if(current <= 715)   //Current range 1
    {
        if( current <= 395)
            CUR=3;
        else if(current <= 445)
            CUR=4;
        else if(current <= 485)
            CUR=5;
        else if(current <= 540)
            CUR=6;
        else if(current <= 585)
            CUR=7;
        else if(current <= 640)
            CUR=8;
        else
            CUR=9;
    }
    else if(current <= 1260)   //Current range 2
    {
      if( current <= 780)
            CUR=10;
        else if(current <= 870)
            CUR=11;
        else if(current <= 995)
            CUR=12;
        else if(current <= 1060)
            CUR=13;
        else if(current <= 1150)
            CUR=14;
        else
            CUR=15;  
    }
      else   //Current range 3
    {
      if( current <= 1405)
            CUR=16;
        else if(current <= 1520)
            CUR=17;
        else if(current <= 1695)
            CUR=18;
        else if(current <= 1850)
            CUR=19;
        else if(current <= 2070)
            CUR=20;
        else if(current <= 2240)
            CUR=21;
        else if(current <= 2440)
            CUR=22;
        else if(current <= 2700)
            CUR=23;
        else if(current <= 2845)
            CUR=24;
        else
            CUR=31;  
    }

    G_CR0_REG.CUR = CUR;
    uint8_t dataHi = REGWRITE | G_CR0_REG.Address;
    uint8_t dataLo = (G_CR0_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}


int AMIS30543DRV::set_microsteps(int number_of_steps)
{
   switch(number_of_steps){
        case 1:
            G_CR0_REG.SM = 0b111;
            G_CR3_REG.ESM = 0; 
            break;
        case 2:
            G_CR0_REG.SM = 0b101;
            G_CR3_REG.ESM = 0; 
            break;
        case 4:
            G_CR0_REG.SM = 0b011;
            G_CR3_REG.ESM = 0; 
            break;
        case 8:
            G_CR0_REG.SM = 0b010;
            G_CR3_REG.ESM = 0; 
            break;
        case 16:
            G_CR0_REG.SM = 0b001;
            G_CR3_REG.ESM = 0; 
            break;
        case 32:
            G_CR0_REG.SM = 0b000;
            G_CR3_REG.ESM = 0; 
            break;
        case 64:
            G_CR3_REG.ESM = 0b010;
            break;
        case 128:
            G_CR3_REG.ESM = 0b001;
            break;
        default:
        //32 Micro steps
        break;
        }

    uint8_t dataHi = REGWRITE | G_CR3_REG.Address;
    uint8_t dataLo = (G_CR3_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
    dataHi = REGWRITE | G_CR0_REG.Address;
    dataLo = (G_CR0_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
  return 0;
}

void AMIS30543DRV::set_sleepEnable(bool enable)
{
    // Set Enable
    G_CR2_REG.SLP = enable ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | G_CR2_REG.Address;
    uint8_t dataLo = (G_CR2_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}

//Slopes of motor driver 
void AMIS30543DRV::set_slope(enum SLOPE slope)
{
    G_CR1_REG.EMC = (uint8_t) slope;
    uint8_t dataHi = REGWRITE | G_CR1_REG.Address;
    uint8_t dataLo = (G_CR1_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}  

//Enables jittery PWM
void AMIS30543DRV::set_jitterPWM(bool enable){
    // Set Enable
    G_CR1_REG.PWMJ = enable ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | G_CR1_REG.Address;
    uint8_t dataLo = (G_CR1_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}

//Speed load angle gain setting
void AMIS30543DRV::set_SLAG(bool mode)
{
        // Set Enable
    G_CR2_REG.SLAG = mode ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | G_CR2_REG.Address;
    uint8_t dataLo = (G_CR2_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}         

//Speed load angle transparency bit
void AMIS30543DRV::set_SLAT(bool mode)
{
        // Set Enable
    G_CR2_REG.SLAT = mode ? 0x01 : 0x00;
    uint8_t dataHi = REGWRITE | G_CR2_REG.Address;
    uint8_t dataLo = (G_CR2_REG.raw & 0x00FF);
    ReadWriteRegister(dataHi, dataLo);
}

  
void AMIS30543DRV::dump_status(StreamOutput *stream)
{

}
 
bool AMIS30543DRV::set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val)
{
return false;
}

bool AMIS30543DRV::check_alarm()
{
   bool error= false;
   G_SR0_REG.raw = ReadRegister(G_SR0_REG.Address);
   G_SR1_REG.raw = ReadRegister(G_SR1_REG.Address);
   G_SR2_REG.raw = ReadRegister(G_SR2_REG.Address);
   G_SR3_REG.raw = ReadRegister(G_SR3_REG.Address);
   G_SR4_REG.raw = ReadRegister(G_SR4_REG.Address);


if(G_SR2_REG.TSD) {
        if(!error_reported.test(0)) THEKERNEL->streams->printf("%c, ERROR: Overtemperature shutdown\n", designator);
        error= true;
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }
   

   if(G_SR0_REG.TW) {
        if(!error_reported.test(0)) THEKERNEL->streams->printf("%c, ERROR: Overtemperature prewarning\n", designator);
        error= true;
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }

    if(G_SR0_REG.CPFAIL) {
        if(!error_reported.test(0)) THEKERNEL->streams->printf("%c, ERROR: Charge pump failure\n", designator);
        error= true;
        error_reported.set(0);
    }else{
        error_reported.reset(0);
    }
    



    return error;


}

uint32_t AMIS30543DRV::getMicrostepPosition(){
     G_SR3_REG.raw = ReadRegister(G_SR3_REG.Address);
     G_SR4_REG.raw = ReadRegister(G_SR4_REG.Address);
     return ((G_SR3_REG.MSP8_2<<2) && 0xF0) || (G_SR4_REG.MSP6_0 && 0x0F);
}

uint16_t AMIS30543DRV::ReadWriteRegister(uint8_t dataHi, uint8_t dataLo)
{
    uint8_t buf[2] {dataHi, dataLo};
    uint8_t rbuf[2];

    spi(buf, 2, rbuf);
    //THEKERNEL->streams->printf("sent: %02X, %02X received:%02X, %02X\n", buf[0], buf[1], rbuf[0], rbuf[1]);
    uint16_t readData = (rbuf[0] << 8) | rbuf[1];
    return readData;
}