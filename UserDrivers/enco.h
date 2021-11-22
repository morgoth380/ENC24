#ifndef _ENCO_H_
#define _ENCO_H_
#include "IQmathLib.h"

#warning ������ �� �������� �����
#define SOFT_VERSION     169
#define SOFT_SUBVERSION  0

#define STM32F303xB_Or_C 0x422UL
#define MCU_IDCODE 0xE0042000UL //����� FLASH-������ � ID ����������

#define ONE_SEC_TACT_CNT    (1000000/encoBlockPnt->encoProcessingPeriod/2) //����� ������ ��������� ��� �������� 1 ��� 
#define ENDAT_ERR_TIME 10

#define FREQ_BASE 80.0F
#define DC_OFFSET 2048
#define  MINIMAL_SPD    0.005F/FREQ_BASE

#define SIN_COS_ADC_AMPL 290 //��������� �������� SIN/COS � �������� ���

#define f_Filtr(valOld, valNew, Kfiltr) (valOld + ((valNew - valOld) * (1.0F / (1 << Kfiltr))))

/* Exported typedef ----------------------------------------------------------*/
#define countof(a)   (sizeof(a) / sizeof(*(a)))
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
typedef enum {
        StopMode = 0,
        SendRequest,
        WaitStartBit,
        ReadFirst12Bit,
        ReadSecond12Bit,
        ReadLastBit,
} ENDAT_SPI_STATE;

typedef enum {
        WaitPacket = 0,
        PacketDone = 1,
} ENDAT_DATA_STATE;

typedef enum {
        SsiStopMode = 0,
        SsiSendRequest,
        SsiReadData,
} SSI_SPI_STATE;


typedef enum{
  NO_SEL = 0,
  SIN_COS = 1,
  INCREMENTAL = 2,
  SERIAL = 3
}encoBlockType;



typedef enum {
        SsiWaitPacket = 0,
        SsiPacketDone = 1,
} SSI_DATA_STATE;


typedef enum {
        DATA_WAIT = 0,
        DATA_DONE = 1,
} ENCO_PROC_STAT;

 typedef enum {
        SinCosDataWait = 0,
        SinCosDataRd = 1,
} SINCOS_DATA_STATE; 

typedef enum {
  speedSignDefState = 0,
  phaseByRsignalState = 1,
  autoPhasingStatedone = 2
}phasingStateType;

typedef struct{
  int32_t encoInputSetting;   //!��������� ����� ����������� ������ ��
  int32_t fastSpdSignSetting; //!���� ������� ���������� ��������
  float phaseShift;             //!�������������� ������������� ������� ����� ���������� ����
}encoFlashMemDataType;


typedef struct {
      u16 SendCommand;             // ������ ������� �������
      u16 TxDataSize;              // ������ ������������ ������ � �����
      u16 TxBitCounter;            // ������� ��� �������� // ������������ � ������� - ����� ������ !!!
      u8 TxInd;                   // ������ ������������� ������� // !!! 
      u8 RxInd;                   // ������ ������������� ������� 
      u8 TxLength;                // ����� ������������� ��������� // !!!
      u8 RxDataSize;              // ����� ���������� ��������� Endat � ����� - ������� �� ��������� � �������� ��������
      ENDAT_SPI_STATE EndatSpiState; 
      u16 StartData;              // ���������� 
      unsigned long long EndatPosition; // ����� ���������� �� Endat
      ENDAT_DATA_STATE PosState;  // ��������� ��������� ������
      unsigned long long txSpiCnt;
      unsigned long long rxSpiCnt;
      
} ENDAT_SPI_BUFFER;

#define DIR_SIGN  4
#define DELTA_PHASE (1.0F/360.0F)     //����������� ��������� ���������� ����������� ����

typedef struct {
      u16 SendCommand;                // ������ ������� �������
      u16 TxDataSize;                 // ������ ������������ ������ � �����
      u16 TxBitCounter;               // ������� ��� �������� // ������������ � ������� - ����� ������ !!!
      u8 TxInd;                       // ������ ������������� ������� // !!! 
      u8 RxInd;                       // ������ ������������� ������� 
      u8 TxLength;                    // ����� ������������� ��������� // !!!
      u8 RxDataSize;                  // ����� ���������� ��������� Endat � ����� - ������� �� ��������� � �������� ��������
      SSI_SPI_STATE SsiSpiState; 
      u16 StartData;                  // ���������� 
      unsigned long long SSIPosition; // ����� ���������� �� SSI
      SSI_DATA_STATE PosState;        // ��������� ��������� ������
      unsigned long long txSpiCnt;
      unsigned long long rxSpiCnt;     
} SSI_SPI_BUFFER;


extern ENDAT_SPI_BUFFER EndatSpiData;
extern SSI_SPI_BUFFER SSI_SpiData;

extern s16 incrementPosCnt;
extern u16 incrSignalErr;


 typedef enum {
        pwmWait = 0,
        impWait = 1,
        pwmOffWait = 2,
} StrgInitStateType;
      

#define RESOLUTION_FACTOR 4 //����������� ���������� ���������� ��������

/* Exported define -----------------------------------------------------------*/

#define SPI1_SCK_PIN                     GPIO_Pin_5                   /* PA.5 */
#define SPI1_SCK_GPIO_PORT               GPIOA                        /* GPIOA */
#define SPI1_SCK_GPIO_CLK                RCC_AHBPeriph_GPIOA
#define SPI1_SCK_SOURCE                  GPIO_PinSource5
#define SPI1_SCK_AF                      GPIO_AF_5

#define SPI1_MISO_PIN                    GPIO_Pin_4                  /* PB.4 */
#define SPI1_MISO_GPIO_PORT              GPIOB                       /* GPIOB */
#define SPI1_MISO_GPIO_CLK               RCC_AHBPeriph_GPIOB
#define SPI1_MISO_SOURCE                 GPIO_PinSource4
#define SPI1_MISO_AF                     GPIO_AF_5

#define SPI1_MOSI_PIN                    GPIO_Pin_7                  /* PA.7 */
#define SPI1_MOSI_GPIO_PORT              GPIOA                       /* GPIOA */
#define SPI1_MOSI_GPIO_CLK               RCC_AHBPeriph_GPIOA
#define SPI1_MOSI_SOURCE                 GPIO_PinSource7
#define SPI1_MOSI_AF                     GPIO_AF_5

#define SPI1_NSS_PIN                     GPIO_Pin_4
#define SPI1_NSS_GPIO_PORT               GPIOA
#define SPI1_NSS_GPIO_CLK                RCC_AHBPeriph_GPIOA
#define SPI1_NSS_SOURCE                  GPIO_PinSource4
#define SPI1_NSS_AF                      GPIO_AF_5

#define SPI_DATA_DIRECT_WRITE()           GPIO_WriteBit(SPI1_NSS_GPIO_PORT, SPI1_NSS_PIN, Bit_SET)     
#define SPI_DATA_DIRECT_READ()            GPIO_WriteBit(SPI1_NSS_GPIO_PORT, SPI1_NSS_PIN, Bit_RESET) 
#define SET_MOSI()                        GPIO_WriteBit(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_PIN, Bit_SET)
#define RESET_MOSI()                      GPIO_WriteBit(SPI1_MOSI_GPIO_PORT, SPI1_MOSI_PIN, Bit_RESET) 
      
#define ADCx_INPUT_SIN1_PIN                GPIO_Pin_0                 /* PA.0 */
#define ADCx_INPUT_SIN1_GPIO_PORT          GPIOA                       /* GPIOA */
#define ADCx_INPUT_SIN1_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define ADCx_INPUT_SIN1_ADC                ADC1
#define ADCx_INPUT_SIN1_CHANNEL            ADC_Channel_1 // ADC_Channel_TempSensor

#define ADCx_INPUT_COS1_PIN                GPIO_Pin_2                 /* PB.2 */
#define ADCx_INPUT_COS1_GPIO_PORT          GPIOB                       /* GPIOB */
#define ADCx_INPUT_COS1_GPIO_CLK           RCC_AHBPeriph_GPIOB
#define ADCx_INPUT_COS1_ADC                ADC2
#define ADCx_INPUT_COS1_CHANNEL            ADC_Channel_12

#define ADCx_INPUT_SIN2_PIN                GPIO_Pin_1                 /* PA.1 */
#define ADCx_INPUT_SIN2_GPIO_PORT          GPIOA                       /* GPIOA */
#define ADCx_INPUT_SIN2_GPIO_CLK           RCC_AHBPeriph_GPIOA
#define ADCx_INPUT_SIN2_ADC                ADC1
#define ADCx_INPUT_SIN2_CHANNEL            ADC_Channel_2

#define ADCx_INPUT_COS2_PIN               /* GPIO_Pin_12*/  GPIO_Pin_6               /* PB.12->PA6 */
#define ADCx_INPUT_COS2_GPIO_PORT          /*GPIOB*/  GPIOA 
#define ADCx_INPUT_COS2_GPIO_CLK           RCC_AHBPeriph_GPIOB
#define ADCx_INPUT_COS2_ADC                ADC2
#define ADCx_INPUT_COS2_CHANNEL            /*ADC_Channel_13*/ADC_Channel_3

#define ADC1_DMA_CHANNEL                   DMA1_Channel1
#define ADC34_DMA_CHANNEL                  DMA2_Channel5

#define ADC1_DMA_FLAG_TC                   DMA1_FLAG_TC1
#define ADC1_DMA_FLAG_GL                   DMA1_FLAG_GL1
#define ADC2_DMA_CHANNEL                   DMA1_Channel2
#define ADC2_DMA_FLAG_TC                   DMA1_FLAG_TC2
#define ADC2_DMA_FLAG_GL                   DMA1_FLAG_GL2

#define ADC_CDR_ADDRESS    ((uint32_t)0x5000030C) // ����� Common �������� ������ ��� ����� Dual
#define ADC34_CDR_ADDRESS  ((uint32_t)0x5000070C) // ����� Common �������� ������ ��� ����� Dual

/*
typedef enum{
  ENDAT_2_0 = 0,
  ENDAT_2_1 = 1,
  ENDAT_2_2 = 2,
  SSI_GRAY = 3,
  SSI_BIN = 4
}serialModeType;
*/

//-------------------------------------
typedef enum {
            ENDAT_100k = 0,
            ENDAT_200k = 1,
            ENDAT_500k = 2,
            ENDAT_1000k = 3,
            ENDAT_2000k = 4,
            ENDAT_5000k = 5,            
} ENDAT_SPEED_Enum; // ������ ������ Endat

typedef enum {
            ELEC_POS = 0,
            MECH_POS = 1,
}ANSVER_POS_TYPE;  // ��� ������������� �������� ������

typedef enum {
            ELEC_SPEED = 0,
            MECH_SPEED = 1,
}ANSVER_SPEED_TYPE; // ��� ������������� �������� ��������

typedef enum {
  encoderDataWait = 0,
  encoderDataPresent = 1,
}ENCODER_DATA_TYPE;


typedef enum {
        pwm_on_wait = 0,
        pwm_on = 1,
}POS_CALC_STATE;  // ��� ������������� �������� ������

//-------------------------------------

#define NUM_ERR_BITS21 1 //!���������� ����� ������ � ������� �� �������� EnDat2.1
#define NUM_ERR_BITS22 2 //!���������� ����� ������ � ������� �� �������� EnDat2.2
#define NUM_CRC_BITS 5   //!���������� ��� CRC � ������� �� ��������
#define ENDAT21 0
#define ENDAT22 1

#define ENCO_ERROR_INTERFAIL 1
#define ENCO_ERROR_CRCFAIL 2
#define NO_ENCO_ANSWER 3
#define  FMAX         80  //!����������� ��������� ������� ����

typedef enum{
  SCALAR_MODE = 0,
  VECTOR_MODE = 1
}drvModeType;
//-------------------------------------
// ��������� �������� ������ ������ ������ � ��������� ����������


typedef enum{
  BITS_NUM_PER_TURN = 0,
  PULSES_NUM_PER_TURN = 1
}resolutionType;


typedef enum{
  ENDAT2_0 = 0,
  ECN1313  = 1,
  ECN1325  = 2,
  SSI_GRAY = 3,
  SSI_BIN  = 4 
}serialModesType;

typedef enum{
  NOT_USE = 0,
  USE = 1
}fastSpdUseType;

typedef struct{
  encoBlockType blockType;           //��� �����: ����������, ����������������, ���������������
  serialModesType serialMode;        //����� ����������������� ����� endat ��� SSI
  u16 polePairsNum;                  //����� ��� ������� ���������
  resolutionType encoResolutionType; //��� ��������� ��������
  u16 bitResolution;                 //���������� �������� � ����� �� ������
  u16 pulseResolution;               //���������� �������� � ��������� �� ������
  u16 encoEmulResol;                 //���������� ��������� ��������
  fastSpdUseType fastSpdUse;         //������� ������������� ���������� ������������ ���������
  u16 fltStrgLen;
  u16 encoSpdFltrTime;
  
}baseEncoDataType;

typedef enum{
  AUTO_SPD_PHASING = 0,
  DIRECT_SPD_PHASING = 1,
  INVERSE_SPD_PHASING = 2
}fastSpdPhasingType;

typedef struct{
  float electricSpd;
  float mechSpd;
  float shadowSpeedElectric;
  float ThetaMechPU;
  float electricTheta;
  float ThetaMechFinePU;
  float ThetaMechCoarsePU;
  float incrementModulSpd;
  float incrementModulAngle;
}calculatedDataType;

typedef struct{
  s16 fastSin;          //!���������� �������� ������� ������������� sin
  s16 fastCos;          //!���������� �������� ������� ������������� cos
  s16 slowSin;          //!���������� �������� sin-������� ���������� �������
  s16 slowCos;          //!���������� �������� cos-������� ���������� ������� 
}analogSignalsType;

typedef enum{
  ENCO_EMUL_OFF = 0,
  ENCO_EMUL_A   = 1,
  ENCO_EMUL_A_B = 2
}encoEmulModeType;

typedef struct {
  calculatedDataType calculatedData;
  u32 incrEncoPos;
  float K1SpdFiltr;
  float K2SpdFiltr;
  u16 procStatus;
  u16 spdPhasingParam; //���� ��������� �������� ��������. ����������� ��������
  drvModeType drvMode; //����� ���������� ��
  u16 thetaOffset;     //�������� ������� ��������
  u16 fltStrgLen;
  u16 encoSpdFltrTime;
  u16 encoProcessingPeriod;
  fastSpdPhasingType incrSpdPhasingSign;
  baseEncoDataType baseEncoMotorData;
  analogSignalsType analogSignals;
  u16 encoErr;
  u16 PWMOn;  
  u16 incrModulSpdPhasingStatus;
  float realTimClockFreq; //������� ������������ ������� ��������� ������� �������� ��������
  u16 incrPosPhasingDone;
  u16 RsignalFlg;
  u16 autoPhasingOn;
  u16 EncoderMode;
  u16 phasingByRSignal;
  float minRefFreq;
  encoEmulModeType encoEmulMode;
  u32 MCU_ID_Code;
  u16 minCapture;  //��� ����������� ���������������� ��������. ����������� ���������� �������� ������ �������
  u16 errCRCcount; //��� ����������� ���������������� ��������. ������� ������.
  u16 queryCnt;    //������� �������� �� ����������������� ���������
  unsigned long long EndatPosition;
  unsigned long long SSIPosition;
  u8 ADC_Amplitude;
  
  void   (*encoBlockPerifInitPnt)(); //������� ������������� ��������� ����������
  void   (*encoBlockCalcPnt)(); 
}encoBlockStatus;

#define CALC_DATA_RESET_VALS {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}
#define BASE_RESET_PARAMS {NO_SEL, ECN1313, 0, BITS_NUM_PER_TURN, 13, 8192, 0, NOT_USE, 8, 2000}
#define ANALOG_SIG_RESET_VALS {0, 0, 0, 0}

#define ENCO_BLOCK_STATUS_DEFAULTS {CALC_DATA_RESET_VALS, 0, 0.0F, 0.0F, 0, 0, SCALAR_MODE, 0, 0, 0, 0, AUTO_SPD_PHASING, \
        BASE_RESET_PARAMS, ANALOG_SIG_RESET_VALS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENCO_EMUL_OFF, 0, 0, 0, 0, 0, 0, 0,\
       (void (*)(u32))encoBlockPerifInit, (void (*)(u32))encoBlockCalc}


//��������� � ������� ��� ����������� �������� �������� ���������
typedef struct {
   float  *thetaStrg;	// ����� ������, ������������� � ����������
   u16  storageLen;	// ����� ������
   u16  storagePos;	// ������� � ������
   float TmpFltr;       // ����������� �������� ��� ����������
   float fltSpeed;      // �������� � ������ ����������
}ENCOFLT;

typedef enum{
  NOT_DEF             = 0,
  INCREMENT_EXT_BLOCK = 1,
  SERIAL_EXT_BLOCK    = 2,
  SIN_COS_EXT_BLOCK   = 3,
  RS485_EXT_BLOCK     = 4,
  DIN_DOUT_EXT_BLOCK  = 5,
  AIN_AOUT_EXT_BLOCK  = 6,
  CAN_EXT_BLOCK       = 7,
  PROFIBUS_EXT_BLOCK  = 8,
  ETHER_EXT_BLOCK     = 9,
  PROFINET_EXT_BLOCK  = 10
}extBlockModeType;

typedef enum{
EXT_DATA_SEND_DELAY =    0,
EXT_BLOCK_IDENT     =    1,
EXT_BLOCK_EXCHANGE  =    2
}telegramModeType;

#define IDENT_TELEGRAM 0
#define EXCHANGE_TELEGRAM 1



typedef struct{
  extBlockModeType extBlock;
  u16 crc;
}identAnswType;


typedef struct {
   float  *captureStrg;	// ����� ������, ������������� � ����������. ����� ������� ��������
   float *sortedCaptStrg;  //����� ��� ����������
   u16  storageLen;	// ����� ������
   u16  storagePos;	// ������� � ������
}SINCOSBUF;

typedef struct {
   u16  *thetaStrg;	// ����� ������, ������������� � ����������
   u16  storageLen;	// ����� ������
   u16  storagePos;	// ������� � ������
        // �������� � ������ ����������
}sin_cosBuf;

typedef enum{
  SPD_SIGN_SYNHRO = 0,
  ERR_CHECKING = 1
}absIncrementPhasingStateType;

/*
typedef enum{
   speedSignDefState = 0,
   phaseByRsignalState = 1,
}autoPhasingStateType;
*/

typedef struct{
  u16 sinCosIncrInputSet;
  float startPhaseShift;
}sinCosStartDataType;

typedef enum{
  ANALOG_FAST_MODE = 0,
  DIGITAL_MODE = 1
}spdCalcModeType;

typedef enum{
  PWM_OFF = 0,
  STAB_SPD_SIGN_WAIT  = 1,
  STAB_SPD_VAL_WAIT   = 2,
  R_SIGNAL_WAIT       = 3,
  R_SIGNAL_WAIT_BREAK = 4,
  R_SIGNAL_MISS       = 5,
  PWM_OFF_WAIT        = 6
}RSignalControlStateType;

typedef  enum{
  DIRECT_PHASING   = 0,
  INVERSE_PHASING  = 1
}spdPhasingType;

typedef enum{
  PULSES_PER_TURN = 0,
  BITS_PER_TURN = 1,
}encoResolModeType;

typedef union{
  u16 word;
  struct{
    u16 telegramType     : 1; //��� �������: �������������� ����� ���������� ��� �����
    fastSpdPhasingType fastSpdSign    : 2; //Sign of speed, calculated  by incremental analog signals
    spdPhasingType  spdPhasingSign    : 1;
    u16 PWM_On         : 1;
    u16 drvType        : 1;
    u16 autoPhasing    : 1;
    u16 fastSpdUse     : 1;
    u16 incrementalMode                 : 1;
    serialModesType serialMode          : 3;
    u16 sinCosMode                      : 1;
    u16 encoEmulMode                    : 2; //�������� ��������
    encoResolModeType encoResolMode     : 1;
  }bits;
}headerType;


//��������� �������� �� �������� ������ ������ 
typedef struct{
  headerType header;     
  u16 bitResolution;
  u16 pulseResolution;
  u16 encoAngleShift;
  u16 motorPolePairsNum;
  u16 processingPeriod;
  u16 angleFltBufNum;
  u16 spdFltTime;
  u8 ADC_Amplitude;
  u8 encoEmulResol;
  u16 crc;
}RxEncoDataType;

typedef union{
  u8 word;
  struct{
    u8 encoErr                  : 5; //��� ������ ��������
    u8 Rsygnal                  : 1; //������ ����������� ����� ��� �������
    spdCalcModeType spdCalcMode : 1; //����� ��������� ��������
    u8 R_PhasingFlg             : 1; //���� ���������� ����������� � ����������� ������
  }bits;
}ansHeaderType;

typedef union{
  u16 word;
  struct{
    u16 version    : 8;
    u16 subVersion : 8;
  }bits;
}softVersionType;

//��������� ������������ �� ������� ������� ������
typedef __packed struct{
  ansHeaderType    TxHeader;      //��������� ������ (��������� ������� �����)
  float            electricTheta; //������� ��������
  float            electricSpd;   //������������� ��������
  softVersionType  softVersion;   //������ �� ����������� ���������
  u16              softCrc;       //CRC ����������� ���������
  u16              dataCRC;       //CRC ������������� ������
}TxEncoDataType;

/*
typedef struct{
  headerType header;     
  u16 crc;
}identDataType;
*/
typedef struct{
  headerType header; 
  u8 reserv;
  u16 crc;
}identDataType;

extern encoBlockStatus encoBlock;
extern u32 fastSin, fastCos;
extern u16 impEvent;
extern u16 phaseShiftIsPresent;

//-----------------------------------------------------
u16 CrcEnDat(u16, u16, u16, u16, uint32_t, u32);
void EndatSpiInit (void);
void EndatTimInit (void);
void encoderAdcInit (encoBlockStatus *encoBlockPnt);
void processingIncrementData(encoBlockStatus *encoBlockPnt);
void sinCosDataProcessing(encoBlockStatus *encoBlockPnt);
void serialDataProcessing(unsigned long long position, encoBlockStatus *encoBlockPnt);
float PLLphaseFilter(encoBlockStatus *encoBlockPnt, float ThetaElec);
float getActualElectrPhase(float ThetaElec, encoBlockStatus *encoBlockPnt);

float velocityMedianFilter(float velocity);
uint8_t encoderInterfacePhaseShift(void);
float getAnalogThetaMechPU(encoBlockStatus *encoBlockPnt);
float analogSpdCalc(float analogThetaMechPU, encoBlockStatus *encoBlockPnt);
float getDiscrThetaMechPU(encoBlockStatus *encoBlockPnt);
float discrSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt);
float discrSpdIncrCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt);
float removeExcessPhase(float shiftPhase);
 u16 RsignalEventPresent(void);
 void RsignalEventAck(void);
 uint8_t getEncoInputSettFromFlash(void);
uint8_t checkSinCosPhasingMiss(uint16_t encoStatus, encoBlockStatus *encoBlockPnt, float phaseAdd);
u16 digitalPhaseInversion(encoBlockStatus *encoBlockPnt, float fltAnalogSpeed, float fltDiscrSpeed, encoFlashMemDataType *pLocNvData);
void nonVolatileDataFlashWrite(encoFlashMemDataType *encoFlashMemData);
void phaseShiftDetect(encoBlockStatus *encoBlockPnt, encoFlashMemDataType *pLocNvData);
float  fastSinCosSignalSpdCalc(encoBlockStatus *encoBlockPnt);
double precThetaCalc(encoBlockStatus *encoBlockPnt);
s32 getFastSpdSignFromFlash(void);
float spdCalcModeValSelect(encoBlockStatus *encoBlockPnt, float disrSpd, float fastAnalogSpd );
float spdModeValCalc(encoBlockStatus *encoBlockPnt);
float EnDatEncoderMechPosCalc(encoBlockStatus *encoBlockPnt, u32 encoderPosition);
float EnDatEncoderElecPosCalc(encoBlockStatus *encoBlockPnt, float ThetaMechPU);
float endatEncoSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt);
float  fastSinCosSignalSpdCalcForEnDat(encoBlockStatus *encoBlockPnt);
void fastSinCosSpdSignPhasing(encoBlockStatus *encoBlockPnt, float digatalSpd, float fastAnalogSpd, encoFlashMemDataType *pLocNvData, s16 digitSpdSignDetectFlg);
u32 getEnDatEncoderPosition(encoBlockStatus *encoBlockPnt, unsigned long long position);
u16 getEnDatEncoderCRC(unsigned long long position);
u16 EnDatEncoderCRCcalc(encoBlockStatus *encoBlockPnt, u32   encoderPosition, unsigned long long position);
uint16_t sinCosEncoderCablesBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt);
uint16_t enDatEncoderIncrSignalBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt);
uint16_t encoderIncrSignalBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt);
uint16_t sinCosEncoderAbsSignalBreakCheck(int16_t encoStatus, encoBlockStatus *encoBlockPnt);
uint16_t sinCosRSignalBreakCheck( uint16_t encoStatus, encoBlockStatus *encoBlockPnt, float spdVal);
u16 enDatDataExchangeErrDetect(uint16_t encoStatus,encoBlockStatus *encoBlockPnt , unsigned long long position);
uint16_t enDatEncoErrDetect(encoBlockStatus *encoBlockPnt, unsigned long long position);
uint16_t sinCosEncoErrDetect(encoBlockStatus *encoBlockPnt, float spdVal, float phaseAdd);
void SSISpiInit(void);
void enDat_SSI_Encoder_A_B_Init(encoBlockStatus *encoBlockPnt);
u16 fastSinCosSignalErrDetectForEnDat(u16 encoErr, float endatAngle, float incrEncoModulAngle, u16 pwmStatus);
float incrAngleSpdCalc(float discrThetaMechPU, encoBlockStatus *encoBlockPnt);
u16 fastSinCosSignalFaultDetect(u16 encoErrStatus, encoBlockStatus *encoBlockPnt);
uint16_t encoderIncrSignalLowSpdBreakCheck(uint16_t encoErrStatus, encoBlockStatus *encoBlockPnt);
int32_t fastSpdSignDef(encoBlockStatus *encoBlockPnt, int32_t autoFastSpdSign);
void incrementalModeInit(encoBlockStatus *encoBlockPnt);
void encoEmulOutputDesable(TIM_TypeDef* TIMx);
void encoEmulOutputEnable(TIM_TypeDef* TIMx);

void encoBlockPerifInit(encoBlockStatus *encoBlockPnt);
void encoBlockCalc(encoBlockStatus *encoBlockPnt);
void encoBlockErrDef(encoBlockStatus *encoBlockPnt);

void sinCosDigitModeInit(encoBlockStatus *encoBlockPnt);
encoBlockType encoBlockTypeDef(void);
void serialModeInit(encoBlockStatus *encoBlockPnt);

void setBasePhasingDataBeforeStart(encoBlockStatus *encoBlockPnt);
void fastSpdSignReadFromFlash(drvModeType drvMode);
void incrementDataProcessing(encoBlockStatus *encoBlockPnt);
void encoEmulStartSettigsSet(TIM_TypeDef* TIMx, encoBlockStatus *encoBlockPnt);
void encoEmulInit(encoBlockStatus *encoBlockPnt);
void encoEmulCalc(encoBlockStatus *encoBlockPnt);
unsigned long long serialDataSel(encoBlockStatus *encoBlockPnt);

//------------------------------������� ������ ��������-------------------------//

#define ENCO_OK                   0
#define C_D_MISS_IN_TUNE_ERR      1      //��� ������ ��� ����� �������� C �/��� D ���������� ������� � �����������
#define C_D_MISS_IN_RUNNING_ERR   2      //��� ������ ��� ����� �������� C �/��� D ���������� ������� � ������
#define A_B_MISS_IN_TUNE_ERR      3      //��� ������ ��� ����� �������� A �/��� B ������������ ������� � �����������
#define A_B_MISS_IN_RUNNING_ERR   4      //��� ������ ��� ����� �������� A �/��� B ������������ ������� � ������
#define CABLE_BREAK_ERR           5      //��� ���� ��������, ���������� ������� � ������������ - ����� ������
#define R_MISS_IN_TUNE_ERR        6      //�� ��������� ������ R �� ����� ����������� �������� sinCos
#define R_MISS_IN_RUNNIG_ERR      7      //�� ��������� ������ R �������� sinCos �� ����� ������
#define INCORRECT_ANGLE_AT_R_ERR  8      //������������ ������� ����� ����� ���������� ����� � ������ R-������� � ������ ���������� ����������
#define INCORRECT_INCR_ABS_PHASE_DIFF_ERR     9  //������� ������� �������� ����� ���������� �������� � ������������ (��������� ����������� �������) ��� ���������� ���������
#define ENDAT_EXCHANGE_MISS_ERR               10 //��� ������ �� ��������� enDat


//--------------------���������� ���� ������----------------------//
#define C_OR_D_MISS           1      //��� ��� �������� D � C
#define A_OR_B_MISS           2      //��� ��� �������� A � B
#define CABLE_BREAK           3      //��� ���� ��������: ������� � ���������
#define A_MISS_IN_TUNE        4      //��� ������ ��� ����� �������� ��� ���� A (������������ sin) � ������ �����������
#define B_MISS_IN_TUNE        5      //��� ������ ��� ����� �������� ��� ���� B (������������ cos) � ������ �����������
#define A_MISS_IN_WORK        6      //��� ������ ��� ����� �������� ��� ���� A (������������ sin) � ������� ������
#define B_MISS_IN_WORK        7      //��� ������ ��� ����� �������� ��� ���� B (������������ cos) � ������� ������
#define C_MISS_IN_WORK        8      //��� ������ ��� ����� �������� ��� ���� D (���������� sin)
#define D_MISS_IN_WORK        9      //��� ������ ��� ����� �������� ��� ���� C (���������� cos)
#define R_MISS_IN_TUNE        10     //�� ��������� ������ R �� ����� ����������� �������� sinCos
#define R_MISS                11     //��� R-������� � ������ ���������� ����������
#define INCORRECT_ANGLE_AT_R  12     //������������ ������� ����� ����� ���������� ����� � ������ R-������� � ������ ���������� ����������
#define INCORRECT_INCR_ABS_PHASE_DIFF 13     //������� ������� �������� ����� ���������� �������� � ������������ (��������� ����������� �������) ��� ���������� ���������
#define ENDAT_EXCHANGE_MISS           14     //��� ������ �� ��������� enDat


#define NOT_DETECT 0
#define POSITIVE   1
#define NEGATIVE   2


//-----------------------------����� ������������ ��������--------------------------//
#define AUTO_DEF 0
#define POSITIVE 1
#define NEGATIVE 2

#define NEGATIVE_SPD -1
#define POSITIVE_SPD 1

#define R_PROCESSING_DELAY 0.2F //�������� �� ��������� R-������� ��� ������ �������, �

#define DATA_EXCHANGE_WAIT 0.5F //������� �������� �������� �� �������� ������

#endif /*_ENCO_H_*/

