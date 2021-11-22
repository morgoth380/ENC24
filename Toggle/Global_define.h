#ifndef _GLOBAL_DEFINE_H_
#define _GLOBAL_DEFINE_H_
/*
// ��������� � ����������
typedef enum {
            Increment = 0,
            EnDat = 1,
            Sin_Cos = 2,
} ENCODER_MODE_Enum; // ��� ��������

//-------------------------------------
typedef enum {
            ENDAT_2_0 = 0,
            ENDAT_2_1 = 1,
            ENDAT_2_2 = 2,
            ENDAT_2_0_SC = 3,
            ENDAT_2_1_SC = 4,
            ENDAT_2_2_SC = 5,            
} ENDAT_MODE_Enum; // ������ ������ Endat
//-------------------------------------
typedef enum {
            ENDAT_100k = 0,
            ENDAT_200k = 1,
            ENDAT_500k = 2,
            ENDAT_1000k = 3,
            ENDAT_2000k = 4,
            ENDAT_5000k = 5,            
} ENDAT_SPEED_Enum; // ������ ������ Endat
//-------------------------------------

#define NUM_ERR_BITS21 1 //!���������� ����� ������ � ������� �� �������� EnDat2.1
#define NUM_ERR_BITS22 2 //!���������� ����� ������ � ������� �� �������� EnDat2.2
#define NUM_CRC_BITS 5   //!���������� CRC � ������� �� ��������
#define ENDAT21 0
#define ENDAT22 1

#define ENCO_ERROR_INTERFAIL 1
#define ENCO_ERROR_CRCFAIL 2
#define NO_ENCO_ANSWER 3

//-------------------------------------
// ��������� �������� ������ ������ ������ � ��������� ����������
typedef struct {
        ENCODER_MODE_Enum  EncoderMode; // ��� ��������
        ENDAT_MODE_Enum    EndatMode;   // ����� Endat
        ENDAT_SPEED_Enum   EndatClockSpeed; // �������� ������ �� ������ Endat - ������� �� ����� ��������
        u16                BitResolution; // ���������� � �����
        u32                Resolution;    // ���������� � ������
        u32                ThetaOffset;   // ����� �� ����
        u16                PolePairs;     // ���-�� ��� ������� ���������
        u32                ThetaMech;     // ������������ ����      
        u32                ThetaElectric; // ������������� ����
        u32                SpeedMech;     // ������� ������������ �������� �������� 
        u32                SpeedElectric; // ������� ������������� �������� ��������    
        u16                ReadEncoPeriod; // ������ ������ ������ � ��������
        u16                EncoderPosition; // ������� ������� �������� � �����
} ENCODER_STATUS; // ������ ������ ����� ��������� ���������


extern ENCODER_STATUS encoder;
*/
#endif /* _GLOBAL_DEFINE_H_ */
