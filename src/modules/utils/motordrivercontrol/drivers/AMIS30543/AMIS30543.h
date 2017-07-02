#pragma once

#include <functional>
#include <bitset>

enum SLOPE{VeryFast , Fast , Slow , VerySlow};

class StreamOutput;

class AMIS30543DRV
{
public:
  AMIS30543DRV(std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi, char designator);

  void init(uint16_t cs) ;
  void set_motorEnable(bool enable);
  void set_sleepEnable(bool enable);
  int  set_microsteps(int number_of_steps);
  void set_current(uint32_t currentma);
  void set_slope(enum SLOPE);    //Slopes of motor driver 
  void set_jitterPWM(bool enable);  //Enables jittery PWM
  void set_SLAG(bool mode);         //Speed load angle gain setting
  void set_SLAT(bool mode);         //Speed load angle transparency bit
  void dump_status(StreamOutput *stream) ;
  bool set_raw_register(StreamOutput *stream, uint32_t reg, uint32_t val);
  bool check_alarm();
  uint32_t getMicrostepPosition();

private:

  uint16_t ReadWriteRegister(uint8_t dataHi, uint8_t dataLo);
  uint16_t ReadRegister(uint8_t addr);
  void ReadAllRegisters () ;
  void WriteAllRegisters () ;


//SPI CONTROL REGISTERS 
// WR Register
  typedef union {
    struct {
      uint8_t Reserved: 3;  // bit 2-0
      uint8_t WDT: 4;       // bit 6-3
      uint8_t WDEN: 1;      // bit 7
      uint8_t Address: 5;   //bit 12-8
    };
    uint16_t raw;
  } WR_Register_t;

  // CR0 Register
  typedef union {
    struct {
      uint8_t CUR: 5;     // bit 4-0
      uint8_t SM:  3;     // bit 7-5
      uint8_t Address: 5; //bit 12-8
    };
    uint16_t raw;
  } CR0_Register_t;


  // CR1 Register
  typedef union {
    struct {
      uint8_t EMC     : 2;    // bit 1-0
      uint8_t PWMJ    : 1;    // bit 2
      uint8_t PWMF    : 1;    // bit 3
      uint8_t Reserved: 2;    // bit 5-4
      uint8_t NXTP    : 1;    // bit 6
      uint8_t DIRCTRL : 1;    // bit7
      uint8_t Address : 5;    //bit 12-8
    };
    uint16_t raw;
  } CR1_Register_t;

  // CR2 Register
  typedef union {
    struct {
      uint8_t Reserved  : 4;  // bit 3-0
      uint8_t SLAT      : 1;  // bit 4
      uint8_t SLAG      : 1;  // bit 5
      uint8_t SLP       : 1;  // bit 6
      uint8_t MOTEN     : 1;  //bit 7
      uint8_t Address   : 5;  //bit 12-8
    };
    uint16_t raw;
  } CR2_Register_t;

  // CR3 Register
  typedef union {
    struct {
      uint8_t ESM     : 3; // bit 2-0
      uint8_t Reserved: 5; // bit 7-3
      uint8_t Address : 5; //bit 12-8
    };
    uint16_t raw;
  } CR3_Register_t;


// STATUS Registers
  typedef union {
    struct  {
      uint8_t Reserved  :2; // bit 1-0
      uint8_t OPENY     :1; // bit 2
      uint8_t OPENX     :1; // bit 3
      uint8_t WD        :1; // bit 4
      uint8_t CPFAIL    :1; // bit 5
      uint8_t TW        :1; // bit 6
      uint8_t PAR       :1; // bit 7  Parity check
      uint8_t Address   :5; //bit 12-8
    };
    uint16_t raw;
  } SR0_Register_t;
   
   typedef union {
    struct  {
      uint8_t Reserved  :3;   // bit 2-0
      uint8_t OVCXNB    :1;   // bit 3
      uint8_t OVCXNT    :1;   // bit 4
      uint8_t OVCXPB    :1;   // bit 5
      uint8_t OVCXPT    :1;   // bit 6
      uint8_t PAR       :1;   // bit 7  Parity check
      uint8_t Address   :5;   //bit 12-8
    };
    uint16_t raw;
  } SR1_Register_t;

    typedef union {
    struct  {
      uint8_t Reserved  :2;   // bit 1-0
      uint8_t TSD       :1;   // bit 2
      uint8_t OVCXNB    :1;   // bit 3
      uint8_t OVCXNT    :1;   // bit 4
      uint8_t OVCXPB    :1;   // bit 5
      uint8_t OVCXPT    :1;   // bit 6
      uint8_t PAR       :1;   // bit 7  Parity check
      uint8_t Address   :5;   //bit 12-8
    };
    uint8_t raw;
  } SR2_Register_t;

  typedef union {
    struct  {
      uint8_t MSP8_2  :7; // bit 6-0
      uint8_t PAR     :1; // bit 7  Parity check
      uint8_t Address :5; //bit 12-8
    };
    uint16_t raw;
  } SR3_Register_t;

  typedef union {
    struct  {
      uint8_t MSP6_0  :7; // bit 6-0
      uint8_t PAR     :1; // bit 7  Parity check
      uint8_t Address :5; //bit 12-8
    };
    uint16_t raw;
  } SR4_Register_t;


  WR_Register_t  G_WR_REG={0};
  CR0_Register_t G_CR0_REG={0};
  CR1_Register_t G_CR1_REG={0};
  CR2_Register_t G_CR2_REG={0};
  CR3_Register_t G_CR3_REG={0};
  SR0_Register_t G_SR0_REG={0};
  SR1_Register_t G_SR1_REG={0};
  SR2_Register_t G_SR2_REG={0};
  SR3_Register_t G_SR3_REG={0};
  SR4_Register_t G_SR4_REG={0};

  std::function<int(uint8_t *b, int cnt, uint8_t *r)> spi;
  float resistor{0.05};
  std::bitset<8> error_reported;
  uint8_t gain{20};
  char designator;

};
