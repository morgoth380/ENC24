#ifndef _GLOBAL_DEFINE_H_
#define _GLOBAL_DEFINE_H_
/*
// Структуры и объявления
typedef enum {
            Increment = 0,
            EnDat = 1,
            Sin_Cos = 2,
} ENCODER_MODE_Enum; // Тип энкодера

//-------------------------------------
typedef enum {
            ENDAT_2_0 = 0,
            ENDAT_2_1 = 1,
            ENDAT_2_2 = 2,
            ENDAT_2_0_SC = 3,
            ENDAT_2_1_SC = 4,
            ENDAT_2_2_SC = 5,            
} ENDAT_MODE_Enum; // Режимы работы Endat
//-------------------------------------
typedef enum {
            ENDAT_100k = 0,
            ENDAT_200k = 1,
            ENDAT_500k = 2,
            ENDAT_1000k = 3,
            ENDAT_2000k = 4,
            ENDAT_5000k = 5,            
} ENDAT_SPEED_Enum; // Режимы работы Endat
//-------------------------------------

#define NUM_ERR_BITS21 1 //!количество битов аварии в посылке от энкодера EnDat2.1
#define NUM_ERR_BITS22 2 //!количество битов аварии в посылке от энкодера EnDat2.2
#define NUM_CRC_BITS 5   //!количество CRC в посылке от энкодера
#define ENDAT21 0
#define ENDAT22 1

#define ENCO_ERROR_INTERFAIL 1
#define ENCO_ERROR_CRCFAIL 2
#define NO_ENCO_ANSWER 3

//-------------------------------------
// Структура текущего режима работы модуля и состояния переменных
typedef struct {
        ENCODER_MODE_Enum  EncoderMode; // Тип энкодера
        ENDAT_MODE_Enum    EndatMode;   // Режим Endat
        ENDAT_SPEED_Enum   EndatClockSpeed; // Скорость обмена по каналу Endat - зависит от длины проводов
        u16                BitResolution; // Разрешение в битах
        u32                Resolution;    // Разрешение в метках
        u32                ThetaOffset;   // Сдвиг по углу
        u16                PolePairs;     // Кол-во пар полюсов двигателя
        u32                ThetaMech;     // Механический угол      
        u32                ThetaElectric; // Электрический угол
        u32                SpeedMech;     // Текущая механическая скорость вращения 
        u32                SpeedElectric; // Текущая электрическая скорость вращения    
        u16                ReadEncoPeriod; // Период чтения данных с энкодера
        u16                EncoderPosition; // Текущая позиция энкодера в тиках
} ENCODER_STATUS; // Полный статус блока обработки энкодеров


extern ENCODER_STATUS encoder;
*/
#endif /* _GLOBAL_DEFINE_H_ */
