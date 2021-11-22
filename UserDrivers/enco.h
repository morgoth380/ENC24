#ifndef _ENCO_H_
#define _ENCO_H_
#include "IQmathLib.h"

#warning Версия ПО задается здесь
#define SOFT_VERSION     169
#define SOFT_SUBVERSION  0

#define STM32F303xB_Or_C 0x422UL
#define MCU_IDCODE 0xE0042000UL //адрес FLASH-памяти с ID процессора

#define ONE_SEC_TACT_CNT    (1000000/encoBlockPnt->encoProcessingPeriod/2) //число тактов программы для выдержки 1 сек 
#define ENDAT_ERR_TIME 10

#define FREQ_BASE 80.0F
#define DC_OFFSET 2048
#define  MINIMAL_SPD    0.005F/FREQ_BASE

#define SIN_COS_ADC_AMPL 290 //Амплитуда сигналов SIN/COS в отсчетах АЦП

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
  int32_t encoInputSetting;   //!настройки входа энкодерного модуля МК
  int32_t fastSpdSignSetting; //!знак быстрой аналоговой скорости
  float phaseShift;             //!корректирующий относительный фазовый сдвиг аналоговой фазы
}encoFlashMemDataType;


typedef struct {
      u16 SendCommand;             // Данные команды запроса
      u16 TxDataSize;              // Размер передаваемых данных в битах
      u16 TxBitCounter;            // Сколько бит передали // Используется в таймере - Можно убрать !!!
      u8 TxInd;                   // Индекс передаваемого символа // !!! 
      u8 RxInd;                   // Индекс передаваемого символа 
      u8 TxLength;                // Длина передаваемого сообщения // !!!
      u8 RxDataSize;              // Длина ожидаемого сообщения Endat в битах - зависит от протокола и битности энкодера
      ENDAT_SPI_STATE EndatSpiState; 
      u16 StartData;              // Вычитываем 
      unsigned long long EndatPosition; // Пакет вычитанный из Endat
      ENDAT_DATA_STATE PosState;  // Состояние получения пакета
      unsigned long long txSpiCnt;
      unsigned long long rxSpiCnt;
      
} ENDAT_SPI_BUFFER;

#define DIR_SIGN  4
#define DELTA_PHASE (1.0F/360.0F)     //максимально возможное отклонение вычисленной фазы

typedef struct {
      u16 SendCommand;                // Данные команды запроса
      u16 TxDataSize;                 // Размер передаваемых данных в битах
      u16 TxBitCounter;               // Сколько бит передали // Используется в таймере - Можно убрать !!!
      u8 TxInd;                       // Индекс передаваемого символа // !!! 
      u8 RxInd;                       // Индекс передаваемого символа 
      u8 TxLength;                    // Длина передаваемого сообщения // !!!
      u8 RxDataSize;                  // Длина ожидаемого сообщения Endat в битах - зависит от протокола и битности энкодера
      SSI_SPI_STATE SsiSpiState; 
      u16 StartData;                  // Вычитываем 
      unsigned long long SSIPosition; // Пакет вычитанный из SSI
      SSI_DATA_STATE PosState;        // Состояние получения пакета
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
      

#define RESOLUTION_FACTOR 4 //Коэффициент увеличения разрешения энкодера

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

#define ADC_CDR_ADDRESS    ((uint32_t)0x5000030C) // Адрес Common регистра данных для режим Dual
#define ADC34_CDR_ADDRESS  ((uint32_t)0x5000070C) // Адрес Common регистра данных для режим Dual

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
} ENDAT_SPEED_Enum; // Режимы работы Endat

typedef enum {
            ELEC_POS = 0,
            MECH_POS = 1,
}ANSVER_POS_TYPE;  // Тип возвращаемого значения позици

typedef enum {
            ELEC_SPEED = 0,
            MECH_SPEED = 1,
}ANSVER_SPEED_TYPE; // Тип возвращаемого значения скорости

typedef enum {
  encoderDataWait = 0,
  encoderDataPresent = 1,
}ENCODER_DATA_TYPE;


typedef enum {
        pwm_on_wait = 0,
        pwm_on = 1,
}POS_CALC_STATE;  // Тип возвращаемого значения позици

//-------------------------------------

#define NUM_ERR_BITS21 1 //!количество битов аварии в посылке от энкодера EnDat2.1
#define NUM_ERR_BITS22 2 //!количество битов аварии в посылке от энкодера EnDat2.2
#define NUM_CRC_BITS 5   //!количество бит CRC в посылке от энкодера
#define ENDAT21 0
#define ENDAT22 1

#define ENCO_ERROR_INTERFAIL 1
#define ENCO_ERROR_CRCFAIL 2
#define NO_ENCO_ANSWER 3
#define  FMAX         80  //!максимально возможная частота поля

typedef enum{
  SCALAR_MODE = 0,
  VECTOR_MODE = 1
}drvModeType;
//-------------------------------------
// Структура текущего режима работы модуля и состояния переменных


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
  encoBlockType blockType;           //Тип блока: аналоговый, последовательный, инкрементальный
  serialModesType serialMode;        //Режим последовательного порта endat или SSI
  u16 polePairsNum;                  //Число пар полюсов двигателя
  resolutionType encoResolutionType; //Тип рарешения энкодера
  u16 bitResolution;                 //Разрешение энкодера в битах на оборот
  u16 pulseResolution;               //Разрешение энкодера в импульсах на оборот
  u16 encoEmulResol;                 //разрешение эмулятора энкодера
  fastSpdUseType fastSpdUse;         //Команда использования аналоговых инкрементных скоростей
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
  s16 fastSin;          //!Мгновенное значение сигнала инкрементного sin
  s16 fastCos;          //!Мгновенное значение сигнала инкрементного cos
  s16 slowSin;          //!Мгновенное значение sin-сигнала абсолютной позиции
  s16 slowCos;          //!Мгновенное значение cos-сигнала абсолютной позиции 
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
  u16 spdPhasingParam; //Знак фазировки скорости энкодера. Принимаемый параметр
  drvModeType drvMode; //Режим управления ПЧ
  u16 thetaOffset;     //Смещение позиции энкодера
  u16 fltStrgLen;
  u16 encoSpdFltrTime;
  u16 encoProcessingPeriod;
  fastSpdPhasingType incrSpdPhasingSign;
  baseEncoDataType baseEncoMotorData;
  analogSignalsType analogSignals;
  u16 encoErr;
  u16 PWMOn;  
  u16 incrModulSpdPhasingStatus;
  float realTimClockFreq; //частота тактирования таймера генерации сигнала эмуляции энкодера
  u16 incrPosPhasingDone;
  u16 RsignalFlg;
  u16 autoPhasingOn;
  u16 EncoderMode;
  u16 phasingByRSignal;
  float minRefFreq;
  encoEmulModeType encoEmulMode;
  u32 MCU_ID_Code;
  u16 minCapture;  //Для обработчика инкрементального энкодера. Минимальное допустимое значение модуля захвата
  u16 errCRCcount; //Для обработчика инкрементального энкодера. Счетчик ошибок.
  u16 queryCnt;    //Счетчик запросов по последовательному протоколу
  unsigned long long EndatPosition;
  unsigned long long SSIPosition;
  u8 ADC_Amplitude;
  
  void   (*encoBlockPerifInitPnt)(); //функция инициализации периферии процессора
  void   (*encoBlockCalcPnt)(); 
}encoBlockStatus;

#define CALC_DATA_RESET_VALS {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F}
#define BASE_RESET_PARAMS {NO_SEL, ECN1313, 0, BITS_NUM_PER_TURN, 13, 8192, 0, NOT_USE, 8, 2000}
#define ANALOG_SIG_RESET_VALS {0, 0, 0, 0}

#define ENCO_BLOCK_STATUS_DEFAULTS {CALC_DATA_RESET_VALS, 0, 0.0F, 0.0F, 0, 0, SCALAR_MODE, 0, 0, 0, 0, AUTO_SPD_PHASING, \
        BASE_RESET_PARAMS, ANALOG_SIG_RESET_VALS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, ENCO_EMUL_OFF, 0, 0, 0, 0, 0, 0, 0,\
       (void (*)(u32))encoBlockPerifInit, (void (*)(u32))encoBlockCalc}


//Структура с данными для определения скорости вращения двигателя
typedef struct {
   float  *thetaStrg;	// Адрес буфера, используемого в фильтрации
   u16  storageLen;	// Длина буфера
   u16  storagePos;	// Позиция в буфере
   float TmpFltr;       // Статическое значение для фильтрации
   float fltSpeed;      // Скорость с учетом фильтрации
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
   float  *captureStrg;	// Адрес буфера, используемого в фильтрации. Буфер текущих значений
   float *sortedCaptStrg;  //буфер для сортировки
   u16  storageLen;	// Длина буфера
   u16  storagePos;	// Позиция в буфере
}SINCOSBUF;

typedef struct {
   u16  *thetaStrg;	// Адрес буфера, используемого в фильтрации
   u16  storageLen;	// Длина буфера
   u16  storagePos;	// Позиция в буфере
        // Скорость с учетом фильтрации
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
    u16 telegramType     : 1; //Тип запроса: индентификация блока расширения или обмен
    fastSpdPhasingType fastSpdSign    : 2; //Sign of speed, calculated  by incremental analog signals
    spdPhasingType  spdPhasingSign    : 1;
    u16 PWM_On         : 1;
    u16 drvType        : 1;
    u16 autoPhasing    : 1;
    u16 fastSpdUse     : 1;
    u16 incrementalMode                 : 1;
    serialModesType serialMode          : 3;
    u16 sinCosMode                      : 1;
    u16 encoEmulMode                    : 2; //Эмуляция энкодера
    encoResolModeType encoResolMode     : 1;
  }bits;
}headerType;


//Струткура принятых от верхнего уровня данных 
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
    u8 encoErr                  : 5; //Код аварии энкодера
    u8 Rsygnal                  : 1; //Сигнал референтной метки для логгера
    spdCalcModeType spdCalcMode : 1; //Режим измерения скорости
    u8 R_PhasingFlg             : 1; //Флаг завершения фазирования с референтной меткой
  }bits;
}ansHeaderType;

typedef union{
  u16 word;
  struct{
    u16 version    : 8;
    u16 subVersion : 8;
  }bits;
}softVersionType;

//Структура отправляемых на верхний уровень данных
typedef __packed struct{
  ansHeaderType    TxHeader;      //Заголовок пакета (структура битовых полей)
  float            electricTheta; //Позиция энкодера
  float            electricSpd;   //Электрическая скорость
  softVersionType  softVersion;   //Версия ПО обработчика энкодеров
  u16              softCrc;       //CRC обработчика энкодеров
  u16              dataCRC;       //CRC передаваемого пакета
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

//------------------------------Статусы ошибок энкодера-------------------------//

#define ENCO_OK                   0
#define C_D_MISS_IN_TUNE_ERR      1      //нет одного или более сигналов C и/или D абсолютной позиции в автотюнинге
#define C_D_MISS_IN_RUNNING_ERR   2      //нет одного или более сигналов C и/или D абсолютной позиции в работе
#define A_B_MISS_IN_TUNE_ERR      3      //нет одного или более сигналов A и/или B инкрементной позиции в автотюнинге
#define A_B_MISS_IN_RUNNING_ERR   4      //нет одного или более сигналов A и/или B инкрементной позиции в работе
#define CABLE_BREAK_ERR           5      //нет всех сигналов, абсолютной позиции и инкрементной - обрыв кабеля
#define R_MISS_IN_TUNE_ERR        6      //не обнаружен сигнал R во время автотюнинга энкодера sinCos
#define R_MISS_IN_RUNNIG_ERR      7      //не обнаружен сигнал R энкодера sinCos во время работы
#define INCORRECT_ANGLE_AT_R_ERR  8      //некорректный фазовый сдвиг между аналоговой фазой в момент R-сигнала в режиме векторного управления
#define INCORRECT_INCR_ABS_PHASE_DIFF_ERR     9  //слишком большая разность между абсолютной позицией и инкрементной (зашумлены инкрементые сигналы) для абсолютных энкодеров
#define ENDAT_EXCHANGE_MISS_ERR               10 //нет обмена по протоколу enDat


//--------------------Внутренние коды аварий----------------------//
#define C_OR_D_MISS           1      //нет пар сигналов D и C
#define A_OR_B_MISS           2      //нет пар сигналов A и B
#define CABLE_BREAK           3      //нет всех сигналов: быстрых и медленных
#define A_MISS_IN_TUNE        4      //нет одного или обоих проводов диф пары A (инкрементный sin) в режиме автотюнинга
#define B_MISS_IN_TUNE        5      //нет одного или обоих проводов диф пары B (инкрементный cos) в режиме автотюнинга
#define A_MISS_IN_WORK        6      //нет одного или обоих проводов диф пары A (инкрементный sin) в рабочем режиме
#define B_MISS_IN_WORK        7      //нет одного или обоих проводов диф пары B (инкрементный cos) в рабочем режиме
#define C_MISS_IN_WORK        8      //нет одного или обоих проводов диф пары D (абсолютный sin)
#define D_MISS_IN_WORK        9      //нет одного или обоих проводов диф пары C (абсолютный cos)
#define R_MISS_IN_TUNE        10     //не обнаружен сигнал R во время автотюнинга энкодера sinCos
#define R_MISS                11     //нет R-сигнала в режиме векторного управления
#define INCORRECT_ANGLE_AT_R  12     //некорректный фазовый сдвиг между аналоговой фазой в момент R-сигнала в режиме векторного управления
#define INCORRECT_INCR_ABS_PHASE_DIFF 13     //слишком большая разность между абсолютной позицией и инкрементной (зашумлены инкрементые сигналы) для абсолютных энкодеров
#define ENDAT_EXCHANGE_MISS           14     //нет обмена по протоколу enDat


#define NOT_DETECT 0
#define POSITIVE   1
#define NEGATIVE   2


//-----------------------------Знаки инкрементной скорости--------------------------//
#define AUTO_DEF 0
#define POSITIVE 1
#define NEGATIVE 2

#define NEGATIVE_SPD -1
#define POSITIVE_SPD 1

#define R_PROCESSING_DELAY 0.2F //Задержка на обработку R-сигнала при подаче питания, с

#define DATA_EXCHANGE_WAIT 0.5F //таймаут ожидания запросов от верхнего уровня

#endif /*_ENCO_H_*/

