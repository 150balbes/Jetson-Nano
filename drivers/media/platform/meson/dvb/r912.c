
//-----------------------------------------------------//
//
// Filename: R912.c
//
// This file is R912 tuner driver
// Copyright 2015 by Rafaelmicro., Inc.
// Author: Ryan Chang
//-----------------------------------------------------//

#include "r912.h"

#define  ADC_MULTI_READ  1
u32 ADC_READ_DELAY = 2;
u8  ADC_READ_COUNT = 1;
u8  VGA_DELAY = 2;
u8  FILTER_DELAY = 2;   
   
#undef dev_dbg
#define dev_dbg(fmt, args...) \
	do {\
		if (debug)\
			printk("R912: %s: " fmt, __func__, ##args);\
	} while (0)
MODULE_PARM_DESC(debug, "\n\t\t Enable Rafael R912 tuner debug information");
static int debug;
module_param(debug, int, 0644);

//PLL LO=161MHz (R20:0x70 ; R24:0x70 ; R28:0x10 ; R29:0x00 ; R30:0x80)

u8 R912_iniArray_hybrid[R912_REG_NUM] = {
						0x00, 0x00, 0x40, 0x44, 0x17, 0x00, 0x06, 0xF0, 0x00, 0x41,
					//  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F  0x10  0x11
						0x7B, 0x0B, 0x70, 0x0A, 0x6E, 0x20, 0x70, 0x87, 0x96, 0x00,    //adr:0x19 from 0x83(1.05+-0.25) modify to 0x87 (1.4+-0.6)
					//  0x12  0x13  0x14  0x15  0x16  0x17  0x18  0x19  0x1A  0x1B  
						0x10, 0x00, 0x80, 0xA5, 0xB7, 0x00, 0x40, 0xCB, 0x95, 0x70,
					//  0x1C  0x1D  0x1E  0x1F  0x20  0x21  0x22  0x23  0x24  0x25
						0x24, 0x00, 0xFD, 0x8B, 0x17, 0x13, 0x01, 0x07, 0x01, 0x3F};
					//  0x26  0x27  0x28  0x29  0x2A  0x2B  0x2C  0x2D  0x2E  0x2F



u8 R912_iniArray_dvbs[R912_REG_NUM] = {
						0x80, 0x05, 0x40, 0x40, 0x1F, 0x1F, 0x07, 0xFF, 0x00, 0x40,
					//  0x08  0x09  0x0A  0x0B  0x0C  0x0D  0x0E  0x0F  0x10  0x11
						0xF0, 0x0F, 0x4D, 0x0A, 0x6F, 0x20, 0x28, 0x87, 0x96, 0x00,  //0x16[1] pulse_flag HPF : Bypass ;  0x19[1:0] Deglich SW Cur : highest
					//  0x12  0x13  0x14  0x15  0x16  0x17  0x18  0x19  0x1A  0x1B   
						0x1C, 0x99, 0xC1, 0x83, 0xB7, 0x00, 0x4F, 0xCB, 0x95, 0x7D,
					//  0x1C  0x1D  0x1E  0x1F  0x20  0x21  0x22  0x23  0x24  0x25
						0xA4, 0x01, 0x24, 0x0B, 0x4F, 0x05, 0x01, 0x47, 0x3F, 0x3F};
					//  0x26  0x27  0x28  0x29  0x2A  0x2B  0x2C  0x2D  0x2E  0x2F




u8 R912_ADDRESS = 0xF4;			
u8 R912_XtalDiv = XTAL_DIV2;
u8 R912_SetTfType = R912_TF_BEAD;   //Depend on HW
//u8 R912_SetTfType = R912_TF_82N_270N;   //Depend on HW
u8 R912_DetectTfType = R912_UL_USING_BEAD;
//u8 R912_DetectTfType = R912_UL_USING_270NH;
u8 R912_Fil_Cal_Gap = 8;
u32 R912_IF_HIGH = 8500;  
u8 R912_Xtal_Pwr = XTAL_SMALL_HIGHEST;
u8 R912_Xtal_Pwr_tmp = XTAL_LARGE_HIGHEST;

//----------------------------------------------------------//
//                   Internal Parameters                    //
//----------------------------------------------------------//

u8 R912_Array[40];
R912_SectType R912_IMR_Data[5] = {
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0},
                                                  {0, 0, 0, 0}
                                                };//Please keep this array data for standby mode.
I2C_TYPE  R912_I2C;
I2C_LEN_TYPE R912_I2C_Len;
I2C_LEN_TYPE Test_Len;

u8  R912_IMR_point_num;
u8  R912_Initial_done_flag = R912_FALSE;
u8  R912_Initial_xtal_check_flag = R912_FALSE;
u8  R912_IMR_done_flag = R912_FALSE;
u8  R912_Bandwidth = 0x00;
u8  R912_SATELLITE_FLAG = 0;
u8  R912_Fil_Cal_flag[R912_STD_SIZE];
static u8 R912_Fil_Cal_BW[R912_STD_SIZE];
static u8 R912_Fil_Cal_code[R912_STD_SIZE];
static u8 R912_IMR_Cal_Type = R912_IMR_CAL;
static R912_Standard_Type R912_pre_standard = R912_STD_SIZE;
static u32 R912_pre_DVBS_bw;

//0: L270n/68n(ISDB-T, DVB-T/T2)
//1: Bead/68n(DTMB)
//2: L270n/68n(N/A)
//3: L270n/68n(ATV)
//4: Bead/68n(DTMB+ATV)
//5: L270n/68n(ATSC, DVB-C, J83B)
//6: Bead/68n(ATSC, DVB-C, J83B)
//7: Bead/82n(R912_DTMB)
//8: L270n/82n(R912_DVB-C, J83B, ATSC, DVB-T/T2, ISDB-T)
u32 R912_LNA_HIGH_MID[R912_TF_SIZE] = { 644000, 644000, 644000, 644000, 644000, 500000, 500000, 500000, 500000}; 
u32 R912_LNA_MID_LOW[R912_TF_SIZE] = { 388000, 388000, 348000, 348000, 348000, 300000, 300000, 300000, 300000};
u32 R912_LNA_LOW_LOWEST[R912_TF_SIZE] = {164000, 164000, 148000, 124000, 124000, 156000, 156000, 108000, 108000};

u32 R912_TF_Freq_High[R912_TF_SIZE][R912_TF_HIGH_NUM] = 
{  	 { 784000, 784000, 776000, 744000, 712000, 680000, 648000, 647000},
	 { 784000, 784000, 776000, 744000, 712000, 680000, 648000, 647000},
	 { 784000, 784000, 776000, 744000, 712000, 680000, 648000, 647000},
	 { 784000, 784000, 776000, 744000, 712000, 680000, 648000, 647000},
	 { 784000, 784000, 776000, 744000, 712000, 680000, 648000, 647000},
         { 784000, 784000, 776000, 680000, 608000, 584000, 536000, 504000},
	 { 784000, 784000, 776000, 680000, 608000, 584000, 536000, 504000},
	 { 784000, 776000, 712000, 616000, 584000, 560000, 520000, 504000},
	 { 784000, 776000, 712000, 616000, 584000, 560000, 520000, 504000}
};


u32 R912_TF_Freq_Mid[R912_TF_SIZE][R912_TF_MID_NUM] = 
{	  {608000, 584000, 560000, 536000, 488000, 440000, 416000, 392000},
	  {608000, 584000, 560000, 536000, 488000, 440000, 416000, 392000},
	  {608000, 560000, 536000, 488000, 440000, 392000, 376000, 352000},
	  {608000, 560000, 536000, 488000, 440000, 392000, 376000, 352000},
	  {608000, 560000, 536000, 488000, 440000, 392000, 376000, 352000},
          {488000, 464000, 440000, 416000, 392000, 352000, 320000, 304000},
	  {488000, 464000, 440000, 416000, 392000, 352000, 320000, 304000},
	  {480000, 464000, 440000, 416000, 392000, 352000, 320000, 304000},
	  {480000, 464000, 440000, 416000, 392000, 352000, 320000, 304000},
};
u32 R912_TF_Freq_Low[R912_TF_SIZE][R912_TF_LOW_NUM] = 
{         {320000, 304000, 272000, 240000, 232000, 216000, 192000, 168000},  //164~388
          {320000, 304000, 272000, 240000, 232000, 216000, 192000, 168000},  //164~388
	  {256000, 240000, 232000, 224000, 216000, 192000, 168000, 160000},  //148~348
	  {256000, 240000, 232000, 192000, 160000, 152000, 144000, 128000},  //124~348
	  {264000, 240000, 192000, 184000, 176000, 152000, 144000, 128000},  //124~348
          {280000, 248000, 232000, 216000, 192000, 176000, 168000, 160000},  //156~300
          {280000, 248000, 232000, 216000, 192000, 176000, 168000, 160000},   //156~300
	  {296000, 280000, 256000, 216000, 184000, 168000, 136000, 112000},   //
	  {296000, 280000, 256000, 216000, 184000, 168000, 136000, 112000}   //
};
u32 R912_TF_Freq_Lowest[R912_TF_SIZE][R912_TF_LOWEST_NUM] = 
{         {160000, 120000, 104000, 88000, 80000, 72000, 56000, 48000},
          {160000, 120000, 104000, 88000, 80000, 72000, 56000, 48000},
	  {144000, 120000, 104000, 88000, 80000, 72000, 56000, 48000},
	  {120000, 96000,   88000,   80000, 72000, 64000, 56000, 48000},
	  {104000, 96000,   88000,   80000, 72000, 64000, 56000, 48000},
	  {136000, 120000, 104000, 88000, 72000, 64000, 56000, 48000},
	  {128000, 120000, 104000, 96000, 80000, 72000, 64000, 56000},
	  {104000, 96000, 88000, 80000, 72000, 64000, 56000, 48000},
	  {104000, 96000, 88000, 80000, 72000, 64000, 56000, 48000}
};

u8 R912_TF_Result_High[R912_TF_SIZE][R912_TF_HIGH_NUM] = 
{         {0x00, 0x00, 0x01, 0x03, 0x04, 0x05, 0x07, 0x07},
          {0x00, 0x00, 0x01, 0x03, 0x04, 0x05, 0x07, 0x07},
	  {0x00, 0x00, 0x01, 0x03, 0x04, 0x05, 0x07, 0x07},
	  {0x00, 0x00, 0x01, 0x03, 0x04, 0x05, 0x07, 0x07},
	  {0x00, 0x00, 0x01, 0x03, 0x04, 0x05, 0x07, 0x07},
          {0x00, 0x00, 0x01, 0x05, 0x0A, 0x0C, 0x13, 0x19},
	  {0x00, 0x00, 0x01, 0x05, 0x0A, 0x0C, 0x13, 0x19},
	  {0x00, 0x03, 0x07, 0x0C, 0x0E, 0x0F, 0x16, 0x1A},
	  {0x00, 0x03, 0x07, 0x0C, 0x0E, 0x0F, 0x16, 0x1A}
};

u8 R912_TF_Result_Mid[R912_TF_SIZE][R912_TF_MID_NUM] = 
{         {0x00, 0x01, 0x03, 0x03, 0x06, 0x0B, 0x0E, 0x11},
          {0x00, 0x01, 0x03, 0x03, 0x06, 0x0B, 0x0E, 0x11},
	  {0x00, 0x03, 0x03, 0x06, 0x0B, 0x11, 0x12, 0x19},  
	  {0x00, 0x03, 0x03, 0x06, 0x0B, 0x11, 0x12, 0x19},  
	  {0x00, 0x03, 0x03, 0x06, 0x0B, 0x11, 0x12, 0x19},
	  {0x06, 0x08, 0x0B, 0x0E, 0x13, 0x17, 0x1E, 0x1F},
          {0x06, 0x08, 0x0B, 0x0E, 0x13, 0x17, 0x1E, 0x1F},
	  {0x09, 0x0D, 0x10, 0x12, 0x16, 0x1B, 0x1E, 0x1F},
	  {0x09, 0x0D, 0x10, 0x12, 0x16, 0x1B, 0x1E, 0x1F}
};
u8 R912_TF_Result_Low[R912_TF_SIZE][R912_TF_LOW_NUM] = 
{         {0x00, 0x02, 0x04, 0x07, 0x0A, 0x0B, 0x0F, 0x16},
          {0x00, 0x02, 0x04, 0x07, 0x0A, 0x0B, 0x0F, 0x16},
	  {0x05, 0x07, 0x0A, 0x0B, 0x0B, 0x0F, 0x16, 0x1A},
	  {0x05, 0x07, 0x0A, 0x0F, 0x1A, 0x1A, 0x23, 0x2A},
	  {0x05, 0x08, 0x10, 0x13, 0x1A, 0x1A, 0x23, 0x2A},
	  {0x05, 0x08, 0x0C, 0x0E, 0x10, 0x14, 0x18, 0x1A},
	  {0x05, 0x08, 0x0C, 0x0E, 0x10, 0x14, 0x18, 0x1A},
	  {0x00, 0x01, 0x03, 0x07, 0x0D, 0x11, 0x1E, 0x2F},
	  {0x00, 0x01, 0x03, 0x07, 0x0D, 0x11, 0x1E, 0x2F}
};
u8 R912_TF_Result_Lowest[R912_TF_SIZE][R912_TF_LOWEST_NUM] = 
{         {0x00, 0x06, 0x0C, 0x15, 0x1C, 0x1F, 0x3C, 0x3F},
          {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08},
	  {0x02, 0x06, 0x0C, 0x15, 0x1C, 0x1F, 0x3C, 0x3F},
	  {0x06, 0x11, 0x15, 0x1C, 0x1F, 0x2F, 0x3C, 0x3F},
          {0x04, 0x08, 0x08, 0x08, 0x10, 0x12, 0x13, 0x13},
	  {0x06, 0x09, 0x0E, 0x18, 0x25, 0x2F, 0x3C, 0x3F},
	  {0x00, 0x04, 0x04, 0x08, 0x08, 0x10, 0x12, 0x13},
	  {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x08},
	  {0x0E, 0x14, 0x18, 0x1E, 0x25, 0x2F, 0x3C, 0x3F}
};


u8  R912_TF = 0;
//----------------------------------------------------------//
//                   Internal static struct                         //
//----------------------------------------------------------//
static R912_SysFreq_Info_Type  SysFreq_Info1;
static R912_Sys_Info_Type         Sys_Info1;
static R912_Freq_Info_Type       Freq_Info1;
//----------------------------------------------------------//
//                   Internal Functions                            //
//----------------------------------------------------------//
R912_ErrCode R912_TF_Check(struct r912_priv *priv);
R912_ErrCode R912_Xtal_Check(struct r912_priv *priv);
R912_ErrCode R912_InitReg(struct r912_priv *priv, R912_Standard_Type R912_Standard);
R912_ErrCode R912_Cal_Prepare(struct r912_priv *priv, u8 u1CalFlag);
R912_ErrCode R912_IMR(struct r912_priv *priv, u8 IMR_MEM, BOOL IM_Flag);
R912_ErrCode R912_PLL(struct r912_priv *priv, u32 LO_Freq, R912_Standard_Type R912_Standard);
R912_ErrCode R912_MUX(struct r912_priv *priv, u32 LO_KHz, u32 RF_KHz, R912_Standard_Type R912_Standard);
R912_ErrCode R912_IQ(struct r912_priv *priv, R912_SectType* IQ_Pont);
R912_ErrCode R912_IQ_Tree(struct r912_priv *priv, u8 FixPot, u8 FlucPot, u8 PotReg, R912_SectType* CompareTree);
R912_ErrCode R912_CompreCor(struct r912_priv *priv, R912_SectType* CorArry);
R912_ErrCode R912_CompreStep(struct r912_priv *priv, R912_SectType* StepArry, u8 Pace);
R912_ErrCode R912_Muti_Read(struct r912_priv *priv, u8* IMR_Result_Data);
R912_ErrCode R912_Section(struct r912_priv *priv, R912_SectType* SectionArry);
R912_ErrCode R912_F_IMR(struct r912_priv *priv, R912_SectType* IQ_Pont);
R912_ErrCode R912_IMR_Cross(struct r912_priv *priv, R912_SectType* IQ_Pont, u8* X_Direct);
R912_ErrCode R912_IMR_Iqcap(struct r912_priv *priv, R912_SectType* IQ_Point);   
R912_ErrCode R912_SetTF(struct r912_priv *priv, u32 u4FreqKHz, u8 u1TfType);
R912_ErrCode R912_SetStandard(struct r912_priv *priv, R912_Standard_Type RT_Standard);
R912_ErrCode R912_SetFrequency(struct r912_priv *priv, R912_Set_Info R912_INFO);
R912_ErrCode R912_DVBS_Setting(struct r912_priv *priv, R912_Set_Info R912_INFO);

R912_Sys_Info_Type R912_Sys_Sel(struct r912_priv *priv, R912_Standard_Type R912_Standard);
R912_Freq_Info_Type R912_Freq_Sel(struct r912_priv *priv, u32 LO_freq, u32 RF_freq, R912_Standard_Type R912_Standard);
R912_SysFreq_Info_Type R912_SysFreq_Sel(struct r912_priv *priv, R912_Standard_Type R912_Standard,u32 RF_freq);

u8 R912_Filt_Cal_ADC(struct r912_priv *priv, u32 IF_Freq, u8 R912_BW, u8 FilCal_Gap);


//---------------------------------------------------------------------------
// I2C functions
//---------------------------------------------------------------------------
int R912_Convert(int InvertNum)
{
	int ReturnNum = 0;
	int AddNum    = 0x80;
	int BitNum    = 0x01;
	int CuntNum   = 0;

	for(CuntNum = 0;CuntNum < 8;CuntNum ++)
	{
		if(BitNum & InvertNum)
			ReturnNum += AddNum;

		AddNum /= 2;
		BitNum *= 2;
	}

	return ReturnNum;
}

int r912_i2c_readreg(struct r912_priv *priv, u8 *data, u16 size)
{
	int ret;
	struct i2c_msg msg[1] = {
		{
			.addr = priv->cfg->i2c_address,
			.flags = I2C_M_RD,
			.buf = data,
			.len = size,
		}
	};

	ret = i2c_transfer(priv->i2c, msg, 1);

	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg("i2c r912_i2c_readreg failed=%d\n", ret);
		ret = -EREMOTEIO;
	}
	//dev_dbg("r912_i2c_readreg: data[0]=%d ret=%d size=%d\n", KBUILD_MODNAME, data[0], ret, size);//vit
	return ret;
}

int r912_i2c_writereg(struct r912_priv *priv, u8 *data, u16 size)
{
	int ret;
	struct i2c_msg msg ={
		.addr = priv->cfg->i2c_address,
		.flags = 0, .buf = data, .len = size };

	ret = i2c_transfer(priv->i2c, &msg, 1);
	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg("i2c r912_i2c_writereg failed=%d\n", ret);
		ret = -EREMOTEIO;
	}
	//dev_dbg("r912_i2c_writereg: data[0]=%d ret=%d size=%d\n", KBUILD_MODNAME, data[0], ret, size);//vit
	return ret;
}

bool I2C_Write_Len(struct r912_priv *priv, I2C_LEN_TYPE *R912_I2C_Len)
{
	int ret = 0;
	u8 buffer[64] = {0};
	u16 i = 0;
	u16 size = 0;

	buffer[0] = R912_I2C_Len->RegAddr;
	for(i = 0;i < R912_I2C_Len->Len; i++)
	{
		buffer[1+i] = R912_I2C_Len->Data[i];
	}

	size = R912_I2C_Len->Len+1;
	ret = r912_i2c_writereg(priv, buffer, size);
	
	//dev_dbg("I2C_Write_Len: ret=%d buffer[0]=%d  size=%d\n", KBUILD_MODNAME, ret, buffer[0], size);//vit

	if (ret)
		return false;
	else
		return true;
}

bool I2C_Write(struct r912_priv *priv, I2C_TYPE *R912_I2C)
{
	I2C_LEN_TYPE i2c_para;

	i2c_para.RegAddr = R912_I2C->RegAddr;
	i2c_para.Data[0] = R912_I2C->Data;
	i2c_para.Len = 1;

	if (I2C_Write_Len(priv,&i2c_para) != true)
    {
        return false;
    }

    return true;
}

bool I2C_Read_Len(struct r912_priv *priv, I2C_LEN_TYPE *R912_I2C_Len)
{
	int ret = 0;
	u8 buffer[64] = {0};
	u16 i = 0;
	u16 size = 0;

	size = R912_I2C_Len->Len;
	ret = r912_i2c_readreg(priv, buffer, size);

	if (ret)
		return false;

	for(i = 0;i < size;i++)
	{
		R912_I2C_Len->Data[i] = R912_Convert(buffer[i]);
	}
	//dev_dbg("I2C_Read_Len: ret=%d buffer[0]=%d  size=%d\n", KBUILD_MODNAME, ret, buffer[0], size);//vit

	return true;
}

//---------------------------------------------------------------------------
// Lock status
//---------------------------------------------------------------------------
int R912_GetLockStatus(struct r912_priv *priv)
{
	int ret;
    I2C_LEN_TYPE i2c_para;

    i2c_para.RegAddr = 0x00;
    i2c_para.Len     = 3;
    if(I2C_Read_Len(priv, &i2c_para) != RT_Success)
    {
	dev_dbg("i2c R912_GetLockStatus failed!\n");
        ret = -EREMOTEIO;
    }

    if ((i2c_para.Data[2] & 0x40) == 0x00)
        ret = 1; //tuner unlock
    else
	ret = 0; //tuner lock
	
    dev_dbg("R912_GETLOCKSTATUS=0x%x\n", i2c_para.Data[2]);

    return ret;
}

//---------------------------------------------------------------------------
// Read regs for debug only
//---------------------------------------------------------------------------
int R912_ReadReg(struct r912_priv *priv)
{
    I2C_LEN_TYPE i2c_para;

    int i, index = 47;

    i2c_para.RegAddr = 0x00;
    i2c_para.Len     = 48;
    if(I2C_Read_Len(priv, &i2c_para) != RT_Success)
    {
        return 1;
    }

    printk("print Reg value start!\n");
	for(i = 0; i<=index; i++)
	{		  
      printk("0x%X\n",i2c_para.Data[i]);		 
	}
    printk("print Reg value end!\n");
    return 0;
}

//-----------------------------------------------------------------------------------------------------------------------------

R912_Sys_Info_Type R912_Sys_Sel(struct r912_priv *priv, R912_Standard_Type R912_Standard)
{
	R912_Sys_Info_Type R912_Sys_Info;

	switch (R912_Standard)
	{
	case R912_MN_5100:   
		R912_Sys_Info.IF_KHz=5100;                    //IF
		R912_Sys_Info.BW=0x60;                          //BW=6M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=6050;          //CAL IF
		R912_Sys_Info.HPF_COR=0x0F;	             //R11[3:0]=15			 R912:R19[3:0]
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable	 R912:R19[4]
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;	 //R30[2]=0, ext normal		 R912:R38[2]
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1	 R912:R38[1:0]
		break;

        case R912_MN_5800:   
		R912_Sys_Info.IF_KHz=5800;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M   R11[6:5]  R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=6800;          //CAL IF  (%)
		R912_Sys_Info.HPF_COR=0x0A;	             //R11[3:0]=10  (%)		 R912:R19[3:0]
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable	 R912:R19[4]
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal		 R912:R38[2]
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1	 R912:R38[1:0]
		break;

	case R912_MN_CIF_5M:     
		R912_Sys_Info.IF_KHz=6750;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M
		R912_Sys_Info.FILT_CAL_IF=7750;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x05;	             //R11[3:0]=5  (2M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

	case R912_PAL_I:         
		R912_Sys_Info.IF_KHz=7300;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8650;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_PAL_I_CIF_5M:       
		R912_Sys_Info.IF_KHz=7750;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=9100;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

	case R912_PAL_DK:
		R912_Sys_Info.IF_KHz=7300;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8500;          //CAL IF     
		R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_PAL_DK_CIF_5M: 
		R912_Sys_Info.IF_KHz=7750;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8950;          //CAL IF     
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

	case R912_PAL_B_7M:  
		R912_Sys_Info.IF_KHz=6600;                     //IF
		R912_Sys_Info.BW=0x40;                           //BW=7M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7800;           //CAL IF
		//R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.HPF_COR=0x0A;	             //R11[3:0]=10 (0.9M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_PAL_B_7M_CIF_5M:  
		R912_Sys_Info.IF_KHz=7250;                     //IF
		R912_Sys_Info.BW=0x00;                           //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8450;           //CAL IF
		R912_Sys_Info.HPF_COR=0x08;	             //R11[3:0]=8 (1.45M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

        case R912_PAL_BGH_8M:  
		R912_Sys_Info.IF_KHz=6600;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7800;          //CAL IF
		R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_PAL_BGH_8M_CIF_5M:  
		R912_Sys_Info.IF_KHz=7750;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8950;          //CAL IF
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

	case R912_SECAM_L: 
		R912_Sys_Info.IF_KHz=7300;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8550;          //CAL IF
		R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_SECAM_L_CIF_5M: 
		R912_Sys_Info.IF_KHz=7750;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8950;          //CAL IF
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

        case R912_SECAM_L1:   
                R912_Sys_Info.IF_KHz=1320;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M  R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8050;          //CAL IF
		R912_Sys_Info.HPF_COR=0x0F;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_SECAM_L1_CIF_5M:   
                R912_Sys_Info.IF_KHz=2250;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8950;          //CAL IF
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

        case R912_SECAM_L1_INV: 
		R912_Sys_Info.IF_KHz=7300;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8550;          //CAL IF
		R912_Sys_Info.HPF_COR=0x0D;	             //R11[3:0]=13
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0, lna=max-1
		break;

	case R912_SECAM_L1_INV_CIF_5M: 
		R912_Sys_Info.IF_KHz=7750;                    //IF
		R912_Sys_Info.BW=0x00;                          //BW=8M
		R912_Sys_Info.FILT_CAL_IF=8950;          //CAL IF
		R912_Sys_Info.HPF_COR=0x09;	             //R11[3:0]=9 (1.15M)
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R11[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R30[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R30[1:0]=0,  lna=max-1 & Buf 4, hpf+1
		break;

	case R912_DVB_T_6M: 
	case R912_DVB_T2_6M: 
		R912_Sys_Info.IF_KHz=4570;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7450;          //CAL IF
		R912_Sys_Info.HPF_COR=0x05;	             //R19[3:0]=5
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;



	case R912_DVB_T_7M:  
	case R912_DVB_T2_7M:  
		R912_Sys_Info.IF_KHz=4570;                     //IF
		R912_Sys_Info.BW=0x40;                           //BW=7M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7750;           //CAL IF
		R912_Sys_Info.HPF_COR=0x08;	             //R19[3:0]=8  (1.45M)
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DVB_T_8M: 
	case R912_DVB_T2_8M: 
		R912_Sys_Info.IF_KHz=4570;                     //IF
		R912_Sys_Info.BW=0x00;                           //BW=8M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8730;           //CAL IF
		R912_Sys_Info.HPF_COR=0x0B;	             //R19[3:0]=11
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DVB_T2_1_7M: 
		R912_Sys_Info.IF_KHz=1900;
		R912_Sys_Info.BW=0x40;                           //BW=7M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7800;           //CAL IF
		R912_Sys_Info.HPF_COR=0x08;	             //R19[3:0]=8
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_DVB_T2_10M: 
		R912_Sys_Info.IF_KHz=5600;
		R912_Sys_Info.BW=0x00;                           //BW=8M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=10800;         //CAL IF
		R912_Sys_Info.HPF_COR=0x0C;	             //R19[3:0]=12
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_DVB_C_8M:  
#if (FOR_TDA10024==1)
		R912_Sys_Info.IF_KHz=5070;
		R912_Sys_Info.BW=0x00;                           //BW=8M   R11[6:5]
		R912_Sys_Info.FILT_CAL_IF=9200;           //CAL IF //8750
		R912_Sys_Info.HPF_COR=0x0A;	             //R19[3:0]=10
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=1, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=10, lna=max-1 & Buf 4
#else
		R912_Sys_Info.IF_KHz=5070;
		R912_Sys_Info.BW=0x00;                           //BW=8M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=9000;           //CAL IF //9150
		R912_Sys_Info.HPF_COR=0x0A;	             //R19[3:0]=10
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=10, lna=max-1 & Buf 4
#endif
		break;

	case R912_DVB_C_6M:  
		R912_Sys_Info.IF_KHz=5070;
		R912_Sys_Info.BW=0x40;                          //BW=7M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8025;          //CAL IF   
		R912_Sys_Info.HPF_COR=0x03;	             //R19[3:0]=3 //3
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_J83B:  
		R912_Sys_Info.IF_KHz=5070;
		R912_Sys_Info.BW=0x40;                          //BW=7M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8025;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x03;	             //R19[3:0]=3 //3
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_ISDB_T: 
		R912_Sys_Info.IF_KHz=4063;
		R912_Sys_Info.BW=0x40;                          //BW=7M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7000;          //CAL IF  //7200
		R912_Sys_Info.HPF_COR=0x08;	             //R19[3:0]=8
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;
	case R912_ISDB_T_4570:
		R912_Sys_Info.IF_KHz=4570;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M
		R912_Sys_Info.FILT_CAL_IF=7250;          //CAL IF
		R912_Sys_Info.HPF_COR=0x05;	             //R19[3:0]=5 (2.0M)
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8, hpf+3
		break;

	case R912_DTMB_4570: 
		R912_Sys_Info.IF_KHz=4570;
		R912_Sys_Info.BW=0x00;                           //BW=8M   R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8330;           //CAL IF  //8130
		R912_Sys_Info.HPF_COR=0x0B;	             //R19[3:0]=11
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

        case R912_DTMB_6000: 
		R912_Sys_Info.IF_KHz=6000;
		R912_Sys_Info.BW=0x00;                           //BW=8M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=9550;//9650; 2015-07-23           //CAL IF
		R912_Sys_Info.HPF_COR=0x03;//0x05;2015-07-23	             //R19[3:0]=3
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=10, lna=max-1 & Buf 4
		break;

	case R912_DTMB_6M_BW_IF_5M:
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x40;                           //BW=7M
		R912_Sys_Info.FILT_CAL_IF=7700;           //CAL IF  
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1 & Buf 4, hpf+1
		break;

	case R912_DTMB_6M_BW_IF_4500:
		R912_Sys_Info.IF_KHz=4500;
		R912_Sys_Info.BW=0x40;                           //BW=7M
		R912_Sys_Info.FILT_CAL_IF=7000;           //CAL IF  
		R912_Sys_Info.HPF_COR=0x05;	             //R19[3:0]=5
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=10, lna=max-1 & Buf 4, hpf+3
		break;
	
	case R912_ATSC:  
		R912_Sys_Info.IF_KHz=5070;
		R912_Sys_Info.BW=0x40;                          //BW=7M      R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7900;          //CAL IF     20130621 Ryan Modify
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4 
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

        case R912_DVB_T_6M_IF_5M: 
	case R912_DVB_T2_6M_IF_5M: 
		R912_Sys_Info.IF_KHz=5000;                    //IF
		R912_Sys_Info.BW=0x40;                          //BW=7M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7800;          //CAL IF
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DVB_T_7M_IF_5M:  
	case R912_DVB_T2_7M_IF_5M:  
		R912_Sys_Info.IF_KHz=5000;                     //IF
		R912_Sys_Info.BW=0x00;                           //BW=8M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8300;           //CAL IF
		R912_Sys_Info.HPF_COR=0x07;	             //R19[3:0]=7
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DVB_T_8M_IF_5M: 
	case R912_DVB_T2_8M_IF_5M: 
		R912_Sys_Info.IF_KHz=5000;                     //IF
		R912_Sys_Info.BW=0x00;                           //BW=8M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=9000;           //CAL IF
		R912_Sys_Info.HPF_COR=0x09;	             //R19[3:0]=9
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=1, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DVB_T2_1_7M_IF_5M: 
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x60;                           //BW=6M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=5900;           //CAL IF
		R912_Sys_Info.HPF_COR=0x01;	             //R19[3:0]=1
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_DVB_C_8M_IF_5M:  
//	case R912_DVB_C_CHINA_IF_5M:   //RF>115MHz
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x00;                           //BW=8M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=9000;           //CAL IF 
		R912_Sys_Info.HPF_COR=0x0A;	             //R19[3:0]=10
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=10, lna=max-1 & Buf 4
		break;

	case R912_DVB_C_6M_IF_5M:  
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x40;                          //BW=7M     R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8100;          //CAL IF   
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4 
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_J83B_IF_5M:  
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x40;                          //BW=7M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8025;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x03;	             //R19[3:0]=3 
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_ISDB_T_IF_5M:
		R912_Sys_Info.IF_KHz=5000;  
		R912_Sys_Info.BW=0x40;                          //BW=7M      R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7940;          //CAL IF  
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x03;   //R38[1:0]=11, buf 8
		break;

	case R912_DTMB_IF_5M: 
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x00;                           //BW=8M      R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8650;//8825;2015-7-23           //CAL IF  8650  2015-07-06 //2015-07-10 9000
		R912_Sys_Info.HPF_COR=0x09;//0x0B;2015-7-23	             //R19[3:0]=9
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x02;   //R38[1:0]=0, lna=max-1
		break;
	
	case R912_ATSC_IF_5M:  
		R912_Sys_Info.IF_KHz=5000;
		R912_Sys_Info.BW=0x40;                          //BW=7M      R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7900;          //CAL IF   
		R912_Sys_Info.HPF_COR=0x04;	             //R19[3:0]=4 
		R912_Sys_Info.FILT_EXT_ENA=0x00;      //R19[4]=0, ext disable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	case R912_FM:  
		R912_Sys_Info.IF_KHz=2400;
		R912_Sys_Info.BW=0x40;                           //BW=7M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=7200;           //CAL IF
		R912_Sys_Info.HPF_COR=0x02;	             //R19[3:0]=2
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	default:  //R912_DVB_T_8M
		R912_Sys_Info.IF_KHz=4570;                     //IF
		R912_Sys_Info.BW=0x00;                           //BW=8M    R912:R19[6:5]
		R912_Sys_Info.FILT_CAL_IF=8330;           //CAL IF
		R912_Sys_Info.HPF_COR=0x0B;	             //R19[3:0]=11
		R912_Sys_Info.FILT_EXT_ENA=0x10;      //R19[4]=1, ext enable
		R912_Sys_Info.FILT_EXT_WIDEST=0x00;//R38[2]=0, ext normal
		R912_Sys_Info.FILT_EXT_POINT=0x00;   //R38[1:0]=0, lna=max-1
		break;

	}


	//Set By DTV/ATV
	if (R912_Standard < R912_ATV_SIZE)  //ATV
	{
		//R912_Sys_Info.INDUC_BIAS = 0x01;      //normal   (R4[0]=1)
		R912_Sys_Info.SWCAP_CLK = 0x02;      //8k      	// R912:R26[1:0]
		R912_Sys_Info.NA_PWR_DET = 0x80;    //off         //R912:R38[7]
	}
	else
	{
		//R912_Sys_Info.INDUC_BIAS = 0x01;     //normal      
		//R912_Sys_Info.SWCAP_CLK = 0x01;     //32k       
		R912_Sys_Info.SWCAP_CLK = 0x02;      //8k        //AGC 500Hz map to 8k	R26[1:0]
		R912_Sys_Info.NA_PWR_DET = 0x00;   //on         						R38[7]
	}


	#if(FOR_KOREA_CTMR==R912_TRUE)
		if (R912_Standard < R912_ATV_SIZE)  //ATV
		{
			R912_Sys_Info.SWBUF_CUR = 0x00;     //high  ( R12[2]=0)
			R912_Sys_Info.FILT_CUR = 0x20;         //high  (R18[6:5]=2'b01)
		}
		else
		{
			R912_Sys_Info.SWBUF_CUR = 0x04;     //low  ( R12[2]=1)
			R912_Sys_Info.FILT_CUR = 0x00;         //highest  (R18[6:5]=2'b00)
		}
	#else
		R912_Sys_Info.SWBUF_CUR = 0x04;          //low  ( R12[2]=1)
		R912_Sys_Info.FILT_CUR = 0x00;         //highest  (R18[6:5]=2'b00)
	#endif




	R912_Sys_Info.TF_CUR = 0x40;                  //low  R11[6]=1
	switch(R912_Standard)
	{
		case R912_DTMB_4570:
		case R912_DTMB_6000:
		case R912_DTMB_6M_BW_IF_5M:
		case R912_DTMB_6M_BW_IF_4500:
		case R912_DTMB_IF_5M:
			R912_Sys_Info.FILT_COMP = 0x00;      //0       R38[6:5] 
			break;
		default: 
		    R912_Sys_Info.FILT_COMP = 0x20;          //1       R38[6:5] 
			break;
	}



	//Filter 3dB
	switch(R912_Standard)
	{
		case R912_DVB_C_8M_IF_5M:
			R912_Sys_Info.FILT_3DB = 0x08;         // ON      R38[3]
		break;
		default: 
		    R912_Sys_Info.FILT_3DB = 0x00;         // OFF     R38[3]
		break;
	}



	//BW 1.7M
	if((R912_Standard==R912_DVB_T2_1_7M) || (R912_Standard==R912_FM))
		R912_Sys_Info.V17M = 0x80;       //enable, 	R19[7]
	else
	    R912_Sys_Info.V17M = 0x00;       //disable, (include DVBT2 1.7M IF=5MHz) R19[7]

	//TF Type select
	switch(R912_Standard)
	{
		case R912_MN_5100:	
		case R912_MN_5800:	
		case R912_PAL_I:
		case R912_PAL_DK:
		case R912_PAL_B_7M:
		case R912_PAL_BGH_8M:
		case R912_SECAM_L:
		case R912_SECAM_L1:
		case R912_SECAM_L1_INV:
		case R912_MN_CIF_5M:	
		case R912_PAL_I_CIF_5M:
		case R912_PAL_DK_CIF_5M:
		case R912_PAL_B_7M_CIF_5M:
		case R912_PAL_BGH_8M_CIF_5M:
		case R912_SECAM_L_CIF_5M:
		case R912_SECAM_L1_CIF_5M:
		case R912_SECAM_L1_INV_CIF_5M:
                if(R912_DetectTfType == R912_UL_USING_BEAD )
				{
					R912_SetTfType = R912_TF_82N_BEAD;   
				}
				else
				{
					R912_SetTfType = R912_TF_82N_270N;   
				}
			 break;

		case R912_DTMB_4570:
		case R912_DTMB_6000:
		case R912_DTMB_6M_BW_IF_5M:
		case R912_DTMB_6M_BW_IF_4500:
		case R912_DTMB_IF_5M:
				if(R912_DetectTfType == R912_UL_USING_BEAD )
				{
					R912_SetTfType = R912_TF_82N_BEAD;   
				}
				else
				{
					R912_SetTfType = R912_TF_82N_270N;   
				}
			 break;	

		default:		
				if(R912_DetectTfType == R912_UL_USING_BEAD)
				{
					R912_SetTfType = R912_TF_82N_BEAD;   
				}
				else
				{
					R912_SetTfType = R912_TF_82N_270N;   
				}	
			break;
	}

	return R912_Sys_Info;
}


R912_Freq_Info_Type R912_Freq_Sel(struct r912_priv *priv, u32 LO_freq, u32 RF_freq, R912_Standard_Type R912_Standard)
{
	R912_Freq_Info_Type R912_Freq_Info;

	//----- RF dependent parameter --------????
	//LNA band  R13[6:5]
	if((RF_freq>=0) && (RF_freq<R912_LNA_LOW_LOWEST[R912_SetTfType]))  //<164
		 R912_Freq_Info.LNA_BAND = 0x60;   // ultra low			// R912:R13[6:5]
	else if((RF_freq>=R912_LNA_LOW_LOWEST[R912_SetTfType]) && (RF_freq<R912_LNA_MID_LOW[R912_SetTfType]))  //164~388
		 R912_Freq_Info.LNA_BAND = 0x40;   //low					// R912:R13[6:5]
	else if((RF_freq>=R912_LNA_MID_LOW[R912_SetTfType]) && (RF_freq<R912_LNA_HIGH_MID[R912_SetTfType]))  //388~612
		 R912_Freq_Info.LNA_BAND = 0x20;   // mid					// R912:R13[6:5]
	else     // >612
		 R912_Freq_Info.LNA_BAND = 0x00;   // high				// R912:R13[6:5]
	
	//----- LO dependent parameter --------
	//IMR point 
	if((LO_freq>=0) && (LO_freq<133000))  
         R912_Freq_Info.IMR_MEM = 0;   
	else if((LO_freq>=133000) && (LO_freq<221000))  
         R912_Freq_Info.IMR_MEM = 1;   
	else if((LO_freq>=221000) && (LO_freq<450000))  
		 R912_Freq_Info.IMR_MEM = 2;  
	else if((LO_freq>=450000) && (LO_freq<775000))  
		 R912_Freq_Info.IMR_MEM = 3; 
	else 
		 R912_Freq_Info.IMR_MEM = 4; 

	//RF polyfilter band   R33[7:6]
	if((LO_freq>=0) && (LO_freq<133000))  
         R912_Freq_Info.RF_POLY = 0x80;   //low	
	else if((LO_freq>=133000) && (LO_freq<221000))  
         R912_Freq_Info.RF_POLY = 0x40;   // mid
	else if((LO_freq>=221000) && (LO_freq<775000))  
		 R912_Freq_Info.RF_POLY = 0x00;   // highest
	else
		 R912_Freq_Info.RF_POLY = 0xC0;   // ultra high

	
	//LPF Cap, Notch
	switch(R912_Standard)
	{
		case R912_MN_5100:	
		case R912_MN_5800:	
		case R912_PAL_I:
		case R912_PAL_DK:
		case R912_PAL_B_7M:
		case R912_PAL_BGH_8M:
		case R912_SECAM_L:
		case R912_SECAM_L1:
		case R912_SECAM_L1_INV:
		case R912_MN_CIF_5M:	
		case R912_PAL_I_CIF_5M:
		case R912_PAL_DK_CIF_5M:
		case R912_PAL_B_7M_CIF_5M:
		case R912_PAL_BGH_8M_CIF_5M:
		case R912_SECAM_L_CIF_5M:
		case R912_SECAM_L1_CIF_5M:
		case R912_SECAM_L1_INV_CIF_5M:
		case R912_DVB_C_8M:
		case R912_DVB_C_6M:
		case R912_J83B:
        case R912_DVB_C_8M_IF_5M:
		case R912_DVB_C_6M_IF_5M:
		case R912_J83B_IF_5M:
			if((LO_freq>=0) && (LO_freq<77000))  
			{
				R912_Freq_Info.LPF_CAP = 16;
				R912_Freq_Info.LPF_NOTCH = 10;
			}
			else if((LO_freq>=77000) && (LO_freq<85000))
			{
				R912_Freq_Info.LPF_CAP = 16;
				R912_Freq_Info.LPF_NOTCH = 4;
			}
			else if((LO_freq>=85000) && (LO_freq<115000))
			{
				R912_Freq_Info.LPF_CAP = 13;
				R912_Freq_Info.LPF_NOTCH = 3;
			}
			else if((LO_freq>=115000) && (LO_freq<125000))
			{
				R912_Freq_Info.LPF_CAP = 11;
				R912_Freq_Info.LPF_NOTCH = 1;
			}
			else if((LO_freq>=125000) && (LO_freq<141000))
			{
				R912_Freq_Info.LPF_CAP = 9;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=141000) && (LO_freq<157000))
			{
				R912_Freq_Info.LPF_CAP = 8;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=157000) && (LO_freq<181000))
			{
				R912_Freq_Info.LPF_CAP = 6;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=181000) && (LO_freq<205000))
			{
				R912_Freq_Info.LPF_CAP = 3;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else //LO>=201M
			{
				R912_Freq_Info.LPF_CAP = 0;
				R912_Freq_Info.LPF_NOTCH = 0;
			}

			break;

		default:
			if((LO_freq>=0) && (LO_freq<73000))  
			{
				R912_Freq_Info.LPF_CAP = 8;
				R912_Freq_Info.LPF_NOTCH = 10;
			}
			else if((LO_freq>=73000) && (LO_freq<81000))
			{
				R912_Freq_Info.LPF_CAP = 8;
				R912_Freq_Info.LPF_NOTCH = 4;
			}
			else if((LO_freq>=81000) && (LO_freq<89000))
			{
				R912_Freq_Info.LPF_CAP = 8;
				R912_Freq_Info.LPF_NOTCH = 3;
			}
			else if((LO_freq>=89000) && (LO_freq<121000))
			{
				R912_Freq_Info.LPF_CAP = 6;
				R912_Freq_Info.LPF_NOTCH = 1;
			}
			else if((LO_freq>=121000) && (LO_freq<145000))
			{
				R912_Freq_Info.LPF_CAP = 4;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=145000) && (LO_freq<153000))
			{
				R912_Freq_Info.LPF_CAP = 3;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=153000) && (LO_freq<177000))
			{
				R912_Freq_Info.LPF_CAP = 2;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else if((LO_freq>=177000) && (LO_freq<201000))
			{
				R912_Freq_Info.LPF_CAP = 1;
				R912_Freq_Info.LPF_NOTCH = 0;
			}
			else //LO>=201M
			{
				R912_Freq_Info.LPF_CAP = 0;
				R912_Freq_Info.LPF_NOTCH = 0;
			}

			break;

	}//end switch(standard)

	return R912_Freq_Info;

}



R912_SysFreq_Info_Type R912_SysFreq_Sel(struct r912_priv *priv, R912_Standard_Type R912_Standard,u32 RF_freq)
{
	R912_SysFreq_Info_Type R912_SysFreq_Info;

	switch(R912_Standard)
	{
	case R912_MN_5100:	
	case R912_MN_5800:	
	case R912_PAL_I:
	case R912_PAL_DK:
	case R912_PAL_B_7M:
	case R912_PAL_BGH_8M:
	case R912_SECAM_L:
    case R912_SECAM_L1:
    case R912_SECAM_L1_INV:
	case R912_MN_CIF_5M:	                      //ATV CIF=5M
	case R912_PAL_I_CIF_5M:
	case R912_PAL_DK_CIF_5M:
	case R912_PAL_B_7M_CIF_5M:
	case R912_PAL_BGH_8M_CIF_5M:
	case R912_SECAM_L_CIF_5M:
    case R912_SECAM_L1_CIF_5M:
    case R912_SECAM_L1_INV_CIF_5M:

			if(RF_freq<165000)
			{
				R912_SysFreq_Info.LNA_TOP=0x04;		       // LNA TOP=5(0x02) =>3(0x04)                 //  R912:R35[2:0]
			}
			else if((RF_freq>=165000) && (RF_freq<345000))
			{
				R912_SysFreq_Info.LNA_TOP=0x05;		       // LNA TOP=4(0x03) =>2 (0x05)                  // R912:R35[2:0]
			}
			else
			{
				R912_SysFreq_Info.LNA_TOP=0x05;		       // LNA TOP=2                     R912:R35[2:0]
			}
				R912_SysFreq_Info.LNA_VTH_L=0xA3;		   // LNA VTH/L=1.34/0.64          (R13=0xA3)		 //  R912:R31[7:0]
				R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=3(0x0C)=>10(0x05)                  (R28[3:0]=4'b1100)// R912:R36[3:0]
				R912_SysFreq_Info.MIXER_VTH_L=0xA5;        // MIXER VTH/L=1.34/0.84        (R14=0xA5)		 // R912:R32[7:0]
				R912_SysFreq_Info.RF_TOP=0xA0;              // RF TOP=2                    (R26[7:5]=3'b101) // R912:R34[7:5]
				R912_SysFreq_Info.NRB_TOP=0x30;            // Nrb TOP=12                   (R28[7:4]=4'b0011)// R912:R36[7:4]
				R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                (R27[7:6]=2'b11)  // R912:R35[7:6]
#if (FOR_KOREA_CTMR==1)	
			if(RF_freq>700000)
		    {
	            R912_SysFreq_Info.LNA_TOP=0x06;		       // LNA TOP=1                    (R27[2:0]=3'b110)
				R912_SysFreq_Info.LNA_VTH_L=0x71;	       // LNA VTH/L=1.04/0.44     (R13=0x71)
				R912_SysFreq_Info.RF_TOP=0x80;              // RF TOP=3                        (R26[7:5]=3'b100)
			}
#endif			
		break;

	case R912_DVB_T_6M:
	case R912_DVB_T_7M:
	case R912_DVB_T_8M:
	case R912_DVB_T_6M_IF_5M:
	case R912_DVB_T_7M_IF_5M:
	case R912_DVB_T_8M_IF_5M:
	case R912_DVB_T2_6M:
	case R912_DVB_T2_7M: 
	case R912_DVB_T2_8M:
	case R912_DVB_T2_1_7M:
	case R912_DVB_T2_10M:
        case R912_DVB_T2_6M_IF_5M:
	case R912_DVB_T2_7M_IF_5M:
	case R912_DVB_T2_8M_IF_5M:
	case R912_DVB_T2_1_7M_IF_5M:
		if((RF_freq>=300000)&&(RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA4;	   // LNA VTH/L=1.34/0.74     (R31=0xA4)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10        (R36[3:0]=4'b0101)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.NRB_TOP=0xC0;            // Nrb TOP=3                       (R36[7:4]=4'b1100)

		}
		else if((RF_freq>=184000) && (RF_freq<=299000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
			R912_SysFreq_Info.MIXER_TOP=0x04;	       // MIXER TOP=11        (R36[3:0]=4'b0101)
			R912_SysFreq_Info.MIXER_VTH_L=0xA6;   // MIXER VTH/L=1.34/0.94  (R32=0xA6)
			R912_SysFreq_Info.NRB_TOP=0x70;            // Nrb TOP=8                       (R36[7:4]=4'b1100)
		}
		else
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10        (R36[3:0]=4'b0101)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.NRB_TOP=0xC0;            // Nrb TOP=3                       (R36[7:4]=4'b1100)
		}
		    R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4           (R35[2:0]=3'b011)
			R912_SysFreq_Info.RF_TOP=0x40;               // RF TOP=5                        (R34[7:5]=3'b010)                 
			R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)	
		break;

        case R912_DVB_C_8M:
	case R912_DVB_C_6M:	
	case R912_J83B:
	case R912_DVB_C_8M_IF_5M:
	case R912_DVB_C_6M_IF_5M:	
	case R912_J83B_IF_5M:
		if(RF_freq<165000)
		{
			R912_SysFreq_Info.LNA_TOP=0x02;		       // LNA TOP=5                    (R35[2:0]=3'b010)
			R912_SysFreq_Info.LNA_VTH_L=0x94;	   // LNA VTH/L=1.24/0.74     (R31=0x94)
			R912_SysFreq_Info.RF_TOP=0x80;               // RF TOP=3                        (R34[7:5]=3'b100)
			R912_SysFreq_Info.NRB_TOP=0x90;            // Nrb TOP=6                       (R36[7:4]=4'b1001)   //R912:R36[7:4]
		}
		else if((RF_freq>=165000) && (RF_freq<=299000))
		{
			R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.LNA_VTH_L=0x94;	   // LNA VTH/L=1.24/0.74     (R31=0x94)
			R912_SysFreq_Info.RF_TOP=0x80;               // RF TOP=3                        (R34[7:5]=3'b100)
			R912_SysFreq_Info.NRB_TOP=0x90;            // Nrb TOP=6                       (R36[7:4]=4'b1001)
		}
		else if((RF_freq>299000) && (RF_freq<=345000))
		{
			R912_SysFreq_Info.LNA_TOP=0x03;		        // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.LNA_VTH_L=0xA4;			// LNA VTH/L=1.34/0.74     (R31=0xA4)
			R912_SysFreq_Info.RF_TOP=0x80;              // RF TOP=3                        (R34[7:5]=3'b100)
			R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                       (R36[7:4]=4'b1001)
		}
		else if((RF_freq>345000) && (RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_TOP=0x04;		        // LNA TOP=3                    (R35[2:0]=3'b100)
			R912_SysFreq_Info.LNA_VTH_L=0x93;		    // LNA VTH/L=1.24/0.64     (R31=0x93)
			R912_SysFreq_Info.RF_TOP=0xA0;				// RF TOP=2                        (R34[7:5]=3'b101)
			R912_SysFreq_Info.NRB_TOP=0x20;             // Nrb TOP=13                       (R36[7:4]=4'b0010)
		}
		else
		{
			R912_SysFreq_Info.LNA_TOP=0x04;		        // LNA TOP=3                    (R35[2:0]=3'b100)
			R912_SysFreq_Info.LNA_VTH_L=0x83;		    // LNA VTH/L=1.14/0.64     (R31=0x83)
			R912_SysFreq_Info.RF_TOP=0xA0;              // RF TOP=2                        (R34[7:5]=3'b101)
			R912_SysFreq_Info.NRB_TOP=0x20;             // Nrb TOP=13                       (R36[7:4]=4'b0010)
		}			
			R912_SysFreq_Info.MIXER_TOP=0x05;	        // MIXER TOP=10               (R36[3:0]=4'b0100)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;			// MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.NRB_BW=0xC0;              // Nrb BW=lowest                  (R35[7:6]=2'b11)                                   
		break;

	case R912_ISDB_T:	
	case R912_ISDB_T_4570:
	case R912_ISDB_T_IF_5M:	
		if((RF_freq>=300000)&&(RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA4;	   // LNA VTH/L=1.34/0.74     (R31=0xA4)
		}
		else
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
		}
			R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10               (R36[3:0]=4'b0101)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			//R912_SysFreq_Info.NRB_TOP=0x20;            // Nrb TOP=13                       (R36[7:4]=4'b0010)
			R912_SysFreq_Info.NRB_TOP=0xB0;            // Nrb TOP=4                       (R36[7:4]=4'b1011)
			R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11) 
                                
		break;

	case R912_DTMB_4570:
	case R912_DTMB_6000:
	case R912_DTMB_6M_BW_IF_5M:
	case R912_DTMB_6M_BW_IF_4500:
	case R912_DTMB_IF_5M:
		if((RF_freq>=300000)&&(RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA4;	   // LNA VTH/L=1.34/0.74     (R31=0xA4)
		}
		else
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
		}
			R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10               (R36[3:0]=4'b0100)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			R912_SysFreq_Info.NRB_TOP=0xA0;            // Nrb TOP=5                       (R36[7:4]=4'b1010)
			R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)         
		break;

	case R912_ATSC:
	case R912_ATSC_IF_5M: 
       if(R912_DetectTfType==R912_UL_USING_BEAD)
	   {
		   if(RF_freq<88000)
		   {
		       R912_SysFreq_Info.LNA_TOP=0x03;	 	       // LNA TOP=4                    (R35[2:0]=3'b011)
			   R912_SysFreq_Info.LNA_VTH_L=0xA5;	       // LNA VTH/L=1.34/0.84     (R31=0xA5)
			   R912_SysFreq_Info.RF_TOP=0xC0;               // RF TOP=1                        (R34[7:5]=3'b110)
			   R912_SysFreq_Info.NRB_TOP=0x30;             // Nrb TOP=12                    (R36[7:4]=4'b0011)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=88000) && (RF_freq<104000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x02;		       // LNA TOP=5                    (R35[2:0]=3'b010)
			   R912_SysFreq_Info.LNA_VTH_L=0xA5;	       // LNA VTH/L=1.34/0.84     (R31=0xA5)		
			   R912_SysFreq_Info.RF_TOP=0xA0;               // RF TOP=2                        (R34[7:5]=3'b101)
			   R912_SysFreq_Info.NRB_TOP=0x30;             // Nrb TOP=12                    (R36[7:4]=4'b0011)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=104000) && (RF_freq<156000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0xA5;	       // LNA VTH/L=1.34/0.84     (R31=0xA5)		
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x30;             // Nrb TOP=12                    (R36[7:4]=4'b0011)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=156000) && (RF_freq<464000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0xA5;	       // LNA VTH/L=1.34/0.84     (R31=0xA5)		
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=464000) && (RF_freq<500000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0xB6;	       // LNA VTH/L=1.44/0.94     (R31=0xB6)		
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else
		   {
				   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
				   R912_SysFreq_Info.LNA_VTH_L=0x94;	       // LNA VTH/L=1.24/0.74     (R31=0x94)		
				   R912_SysFreq_Info.RF_TOP=0x40;               // RF TOP=5                        (R34[7:5]=3'b010)
				   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
				   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
	   }
	   else  //270n
	   {
		    if(RF_freq<88000)
		   {
		       R912_SysFreq_Info.LNA_TOP=0x02;	 	       // LNA TOP=5                    (R35[2:0]=3'b010)
			   R912_SysFreq_Info.LNA_VTH_L=0x94;	       // LNA VTH/L=1.24/0.74     (R31=0x94)
			   R912_SysFreq_Info.RF_TOP=0x80;               // RF TOP=3                        (R34[7:5]=3'b100)
			   R912_SysFreq_Info.NRB_TOP=0xC0;             // Nrb TOP=3                    (R36[7:4]=4'b1100)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=88000) && (RF_freq<104000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x02;		       // LNA TOP=5                    (R35[2:0]=3'b010)
			   R912_SysFreq_Info.LNA_VTH_L=0x94;	       // LNA VTH/L=1.24/0.74     (R31=0x94)
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=104000) && (RF_freq<248000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0x94;	       // LNA VTH/L=1.24/0.74     (R31=0x94)	
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=248000) && (RF_freq<464000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0xA5;	       // LNA VTH/L=1.34/0.84     (R31=0xA5)		
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
		   else if((RF_freq>=464000) && (RF_freq<500000))
		   {
			   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
			   R912_SysFreq_Info.LNA_VTH_L=0xB6;	       // LNA VTH/L=1.44/0.94     (R31=0xB6)		
			   R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
			   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }	
		   else
		   {
		       
				   R912_SysFreq_Info.LNA_TOP=0x01;		       // LNA TOP=6                    (R35[2:0]=3'b001)
				   R912_SysFreq_Info.LNA_VTH_L=0x94;	       // LNA VTH/L=1.24/0.74     (R31=0x94)		
				   R912_SysFreq_Info.RF_TOP=0x40;               // RF TOP=5                        (R34[7:5]=3'b010)
				   R912_SysFreq_Info.NRB_TOP=0x90;             // Nrb TOP=6                      (R36[7:4]=4'b1001)
				   R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)
		   }
	   
	   }
	        R912_SysFreq_Info.MIXER_TOP=0x04;	       // MIXER TOP=11               (R36[3:0]=4'b0100)
			R912_SysFreq_Info.MIXER_VTH_L=0xB7;   // MIXER VTH/L=1.44/1.04  (R32=0xB7)
			//R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)

		break;

	case R912_FM:
		if((RF_freq>=300000)&&(RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA4;	   // LNA VTH/L=1.34/0.74     (R31=0xA4)
		}
		else
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
		}
			R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10               (R36[3:0]=4'b0100)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.RF_TOP=0x60;               // RF TOP=4                        (R34[7:5]=3'b011)
			R912_SysFreq_Info.NRB_TOP=0x20;            // Nrb TOP=13                       (R36[7:4]=4'b0010)
			R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)                                
		break;

	default: //DVB-T 8M
		if((RF_freq>=300000)&&(RF_freq<=472000))
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA4;	   // LNA VTH/L=1.34/0.74     (R31=0xA4)
		}
		else
		{
			R912_SysFreq_Info.LNA_VTH_L=0xA5;	   // LNA VTH/L=1.34/0.84     (R31=0xA5)
		}
			R912_SysFreq_Info.LNA_TOP=0x03;		       // LNA TOP=4                    (R35[2:0]=3'b011)
			R912_SysFreq_Info.MIXER_TOP=0x05;	       // MIXER TOP=10               (R36[3:0]=4'b0100)
			R912_SysFreq_Info.MIXER_VTH_L=0x95;   // MIXER VTH/L=1.24/0.84  (R32=0x95)
			R912_SysFreq_Info.RF_TOP=0x40;               // RF TOP=5                        (R34[7:5]=3'b010)                 
			R912_SysFreq_Info.NRB_TOP=0x20;            // Nrb TOP=13                       (R36[7:4]=4'b0010)
			R912_SysFreq_Info.NRB_BW=0xC0;             // Nrb BW=lowest                  (R35[7:6]=2'b11)	                              
		break;
	
	} //end switch

	R912_SysFreq_Info.RF_FAST_DISCHARGE = 0x00;    //0 	 R912:R46[3:1]=3'b000
	R912_SysFreq_Info.RF_SLOW_DISCHARGE = 0x80;    //4   R912:R22[7:5]=2'b100
	R912_SysFreq_Info.RFPD_PLUSE_ENA = 0x10;	   //1   R912:R38[4]=1

	R912_SysFreq_Info.LNA_FAST_DISCHARGE = 0x0A;    //10  R912:R43[4:0]=5'b01010
	R912_SysFreq_Info.LNA_SLOW_DISCHARGE = 0x00;    //0  R912:R22[4:2]=3'b000
	R912_SysFreq_Info.LNAPD_PLUSE_ENA = 0x00 ;	    //0  R912:R17[7]=0

    //AGC Clk Rate
	R912_SysFreq_Info.AGC_CLK = 0x00;              //1k   R26[4:2]  //default

	//TF LPF setting
	switch(R912_Standard)
	{
		case R912_DTMB_4570:
	    case R912_DTMB_6000:
		case R912_DTMB_6M_BW_IF_5M:
		case R912_DTMB_6M_BW_IF_4500:
		case R912_DTMB_IF_5M:
			 //if(RF_freq<=196000)
			//	 R912_SysFreq_Info.BYP_LPF = 0x40;		  //low pass R12[6]
			 //else
				 R912_SysFreq_Info.BYP_LPF = 0x00;      //bypass  R12[6]

	break;

		case R912_DVB_T_6M:
		case R912_DVB_T_7M:
		case R912_DVB_T_8M:
		case R912_DVB_T_6M_IF_5M:
		case R912_DVB_T_7M_IF_5M:
		case R912_DVB_T_8M_IF_5M:
		case R912_DVB_T2_6M: 
		case R912_DVB_T2_7M:
		case R912_DVB_T2_8M:
		case R912_DVB_T2_1_7M: 
		case R912_DVB_T2_10M: 
		case R912_DVB_T2_6M_IF_5M:
		case R912_DVB_T2_7M_IF_5M:
		case R912_DVB_T2_8M_IF_5M:
		case R912_DVB_T2_1_7M_IF_5M: 
			if(RF_freq>=662000 && RF_freq<=670000)
			{
				R912_SysFreq_Info.NRB_TOP=0xB0;            // Nrb TOP=4                       (R36[7:4]=4'b1011)
				R912_SysFreq_Info.RF_SLOW_DISCHARGE = 0x20;    //1   R912:R22[7:5]=2'b001
				R912_SysFreq_Info.AGC_CLK = 0x18;		 //60Hz   R26[4:2] 
			}
			else
			{
				R912_SysFreq_Info.RF_SLOW_DISCHARGE = 0x80;    //4   R912:R22[7:5]=2'b100
				R912_SysFreq_Info.AGC_CLK = 0x00;		 //1KHz   R26[4:2] 
				switch(R912_Standard)
				{
					case R912_DVB_T2_6M: 
					case R912_DVB_T2_7M:
					case R912_DVB_T2_8M:
					case R912_DVB_T2_1_7M: 
					case R912_DVB_T2_10M: 
					case R912_DVB_T2_6M_IF_5M:
					case R912_DVB_T2_7M_IF_5M:
					case R912_DVB_T2_8M_IF_5M:
					case R912_DVB_T2_1_7M_IF_5M:
						R912_SysFreq_Info.AGC_CLK = 0x1C;		 //250Hz   R26[4:2] 
					break;
					default:
					break;
				}
			}
			if(RF_freq<=236000)
				R912_SysFreq_Info.BYP_LPF = 0x40;      //low pass  R12[6]
			else
				R912_SysFreq_Info.BYP_LPF = 0x00;      //bypass  R12[6]
		break;

		default:  //other standard
			 if(RF_freq<=236000)
				 R912_SysFreq_Info.BYP_LPF = 0x40;      //low pass  R12[6]
			 else
				 R912_SysFreq_Info.BYP_LPF = 0x00;      //bypass  R12[6]

        break;
	}//end switch

	return R912_SysFreq_Info;
	
	}




R912_ErrCode R912_Init(struct r912_priv *priv)
{
		u8 i;
		
		

		if(R912_Initial_done_flag==R912_FALSE)
		{

			  //reset filter cal.
			  for (i=0; i<R912_STD_SIZE; i++)
			  {	  
				  R912_Fil_Cal_flag[i] = R912_FALSE;
				  R912_Fil_Cal_code[i] = 0;
				  R912_Fil_Cal_BW[i] = 0x00;
			  }
			  
			  

			  if(R912_IMR_done_flag==R912_FALSE)
			  {

				if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)        
				 return RT_Fail;
			 
				

				if(R912_TF_Check(priv) != RT_Success)        
				 return RT_Fail;
			 
				

				  //start IMR calibration
				  if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)        //write initial reg before doing IMR Cal
					 return RT_Fail;
					 
				
	    
				  if(R912_Cal_Prepare(priv,R912_IMR_CAL) != RT_Success)     
					  return RT_Fail;
				  
				

				  if(R912_IMR(priv,3, R912_TRUE) != RT_Success)       //Full K node 3
					return RT_Fail;
					
				

				  if(R912_IMR(priv,0, R912_FALSE) != RT_Success)
					return RT_Fail;
				
				

				  if(R912_IMR(priv,1, R912_FALSE) != RT_Success)
					return RT_Fail;
				
				

				  if(R912_IMR(priv,2, R912_FALSE) != RT_Success)
					return RT_Fail;
				
				

				  if(R912_IMR(priv,4, R912_TRUE) != RT_Success)   //Full K node 4
					return RT_Fail;
					
				

				  R912_IMR_done_flag = R912_TRUE;
			  }

#if(R912_SHARE_XTAL_OUT==R912_TRUE)
		R912_Xtal_Pwr = XTAL_LARGE_STRONG;
#else
		//do Xtal check
		if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)        
		 return RT_Fail;

		if(R912_Xtal_Check(priv) != RT_Success)        
			return RT_Fail;

		if(R912_Xtal_Pwr_tmp==XTAL_LARGE_STRONG)
			R912_Xtal_Pwr = XTAL_LARGE_STRONG;
		else
			R912_Xtal_Pwr = R912_Xtal_Pwr_tmp + 1;
#endif

			  R912_Initial_done_flag = R912_TRUE;

		} //end if(check init flag)

		//write initial reg	
             if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)        
			 {  return RT_Fail;}

		R912_pre_standard = R912_STD_SIZE;

	return RT_Success;
}



R912_ErrCode R912_InitReg(struct r912_priv *priv, R912_Standard_Type R912_Standard)
{
	u8 InitArrayCunt = 0;
	
	//Write Full Table, Set Xtal Power = highest at initial
	R912_I2C_Len.RegAddr = 0x08;   //  R912:0x08
	R912_I2C_Len.Len = R912_REG_NUM;
	if(R912_Standard!=R912_DVB_S)
	{
		for(InitArrayCunt = 0; InitArrayCunt<R912_REG_NUM; InitArrayCunt ++)
		{
			R912_I2C_Len.Data[InitArrayCunt] = R912_iniArray_hybrid[InitArrayCunt];
			R912_Array[InitArrayCunt] = R912_iniArray_hybrid[InitArrayCunt];
		}
	}
	else
	{
		for(InitArrayCunt = 0; InitArrayCunt<R912_REG_NUM; InitArrayCunt ++)
		{
			R912_I2C_Len.Data[InitArrayCunt] = R912_iniArray_dvbs[InitArrayCunt];
			R912_Array[InitArrayCunt] = R912_iniArray_dvbs[InitArrayCunt];
		}
	}
	if(I2C_Write_Len(priv,&R912_I2C_Len) != RT_Success)
		return RT_Fail;

	return RT_Success;
}





R912_ErrCode R912_TF_Check(struct r912_priv *priv)
{
	u32   RingVCO = 0;
	u32   RingFreq = 72000;
	u32   RingRef = R912_Xtal;
	u8    divnum_ring = 0;
	u8    VGA_Count = 0;
	u8    VGA_Read = 0;

	if(R912_Xtal==16000)  //16M
	{
         divnum_ring = 11;
	}
	else if(R912_Xtal==24000)   //24M
	{
		 divnum_ring = 2;
	}
	else //27MHz
	{
		divnum_ring = 0;
	}

	

	 RingVCO = (16+divnum_ring)* 8 * RingRef;
	 RingFreq = RingVCO/48;

	 if(R912_Cal_Prepare(priv,R912_TF_LNA_CAL) != RT_Success)      
	      return RT_Fail;

	

     R912_I2C.RegAddr = 0x27;	
     R912_Array[31] = (R912_Array[31] & 0x03) | 0x80 | (divnum_ring<<2);  //pre div=6 & div_num
     R912_I2C.Data = R912_Array[31];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;
	   
	

	 //Ring PLL PW on
	 R912_I2C.RegAddr = 0x12;	
     R912_Array[18] = (R912_Array[18] & 0xEF); 
     R912_I2C.Data = R912_Array[18];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;
	   
	

	 //NAT Det Sour : Mixer buf out
	 R912_I2C.RegAddr = 0x25;	
     R912_Array[37] = (R912_Array[37] & 0x7F); 
     R912_I2C.Data = R912_Array[37];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	

	 //R912 R33[7:0]   

	 R912_Array[25] = (R912_Array[25] & 0x00) | 0x8B;  //out div=8, RF poly=low band, power=min_lp
	 if(RingVCO>=3200000)
	    R912_Array[25] = (R912_Array[25] & 0xDF);      //vco_band=high, R25[5]=0
	 else
        R912_Array[25] = (R912_Array[25] | 0x20);      //vco_band=low, R25[5]=1
	 R912_I2C.RegAddr = 0x21;
	 R912_I2C.Data = R912_Array[25];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	

     //Must do before PLL()
	 if(R912_MUX(priv,RingFreq + 5000, RingFreq, R912_STD_SIZE) != RT_Success)     //FilCal MUX (LO_Freq, RF_Freq)
	     return RT_Fail;
	 
	

	 //Set PLL
	 if(R912_PLL(priv,(RingFreq + 5000), R912_STD_SIZE) != RT_Success)   //FilCal PLL
	       return RT_Fail;

	
		   
	//Set LNA TF=(1,15),for VGA training   // R912 R8[6:0]
	 R912_I2C.RegAddr = 0x08;
     R912_Array[0] = (R912_Array[0] & 0x80) | 0x1F;  	
     R912_I2C.Data = R912_Array[0];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;
	  
	
	  
	//Qctrl=off  // R912 R14[5] 
	 R912_I2C.RegAddr = 0x0E;
     R912_Array[6] = (R912_Array[6] & 0xDF);  	
     R912_I2C.Data = R912_Array[6];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;
	  
	

	// FilterComp OFF  // R912 R14[2]  
	 R912_I2C.RegAddr = 0x0E;
     R912_Array[6] = (R912_Array[6] & 0xFB);  	
     R912_I2C.Data = R912_Array[6];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;
	  
	

	// Byp-LPF: Bypass  R912 R12[6]  12-8=4  12(0x0C) is addr ; [4] is data
	 R912_I2C.RegAddr = 0x0C;
     R912_Array[4] = R912_Array[4] & 0xBF;  	
     R912_I2C.Data = R912_Array[4];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;
	  
	


	 //Adc=on; Vga code mode, Gain = -12dB 
	 //R912 R20[3:0]         set 0
	 //R912 R9[1]            set 1
	 //R912 R11[3]           set 0
	 //R912 R18[7]           set 0
	 //R912 R15[7]           set 0
	 
	 // VGA Gain = -12dB 
     R912_I2C.RegAddr = 0x14;
     R912_Array[12] = (R912_Array[12] & 0xF0);
     R912_I2C.Data = R912_Array[12];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	// Vga code mode
     R912_I2C.RegAddr = 0x09;
     R912_Array[1] = (R912_Array[1] | 0x02);
     R912_I2C.Data = R912_Array[1];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	// VGA 6dB
     R912_I2C.RegAddr = 0x0B;
     R912_Array[3] = (R912_Array[3] & 0xF7);
     R912_I2C.Data = R912_Array[3];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	// VGA PW ON
     R912_I2C.RegAddr = 0x12;
     R912_Array[10] = (R912_Array[10] & 0x7F);
     R912_I2C.Data = R912_Array[10];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	 // Adc on
     R912_I2C.RegAddr = 0x0F;
     R912_Array[7] = (R912_Array[7] & 0x7F);
     R912_I2C.Data = R912_Array[7];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;



	 //------- increase VGA power to let ADC read value significant ---------//

	 for(VGA_Count=5; VGA_Count < 16; VGA_Count ++)
	 {
		R912_I2C.RegAddr = 0x14;
		R912_I2C.Data = (R912_Array[12] & 0xF0) + VGA_Count;  
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_Delay_MS(VGA_DELAY); //
		
		if(R912_Muti_Read(priv,&VGA_Read) != RT_Success)
			return RT_Fail;

		if(VGA_Read > 40*ADC_READ_COUNT)
			break;
	 }

	 //Set LNA TF=(0,0)
	 R912_I2C.RegAddr = 0x08;
     R912_Array[0] =(R912_Array[0] & 0x80);  	
     R912_I2C.Data = R912_Array[0];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;

	 R912_Delay_MS(10); //

	 if(R912_Muti_Read(priv,&VGA_Read) != RT_Success)
		  return RT_Fail;

	 if(VGA_Read > (36*ADC_READ_COUNT))
        R912_DetectTfType = R912_UL_USING_BEAD;
	 else
	    R912_DetectTfType = R912_UL_USING_270NH;

	return RT_Success;
}

R912_ErrCode R912_Xtal_Check(struct r912_priv *priv)
{
	u8 i = 0;
        u8 vco_check_bank = 0;

	// TF force sharp mode (for stable read)
	R912_I2C.RegAddr = 0x16;
	R912_Array[14] = R912_Array[14] | 0x01;    
	R912_I2C.Data = R912_Array[14];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;

	// NA det off (for stable read)
	R912_I2C.RegAddr = 0x26;
	R912_Array[30] = R912_Array[30] | 0x80;  
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;

	//Set NA det 710 = OFF
	R912_I2C.RegAddr  = 0x28;								
	R912_Array[32] = (R912_Array[32] | 0x08);
    R912_I2C.Data = R912_Array[32];
    if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;
	
	// Xtal_pow0=highest(00)  R912:R23[6:5]   ;    Xtal_pow1=low  R912:R23[7]
	R912_I2C.RegAddr = 0x17;
	R912_Array[15] = (R912_Array[15] & 0x9F) | 0x80;
	//set pll autotune = 128kHz  R912:R23[4:3]
	R912_Array[15] = R912_Array[15] & 0xE7;
	R912_I2C.Data = R912_Array[15];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;

	//Xtal_Gm=SMALL(0) R912:R27[0]
	R912_I2C.RegAddr = 0x1B;
	R912_Array[19] = (R912_Array[19] & 0xFE) | 0x00;
	//set manual initial reg = 1 000000; b5=0 => cap 16p;		 R912:R27[7:0]
	R912_Array[19] = (R912_Array[19] & 0x80) | 0x40;
	R912_I2C.Data = R912_Array[19];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	//set auto
	R912_I2C.RegAddr = 0x1B;
	R912_Array[19] = (R912_Array[19] & 0xBF);
	R912_I2C.Data = R912_Array[19];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	for(i=0; i<XTAL_CHECK_SIZE; i++) //from weak to strong
	{
	    //set power
		if(i == XTAL_LARGE_STRONG)// Xtal_pow0=highest(00)  R912:R23[6:5]   ;    Xtal_pow1=low(1)  R912:R23[7]
		{
			R912_Array[15] = (R912_Array[15] & 0x1F) | 0X80 ;
			R912_Array[19] = (R912_Array[19] & 0xFE) | 0x01 ;		//| 0x00;  //LARGE Gm(1)
		}
		else if(i <= XTAL_SMALL_HIGHEST)  //Xtal_pow1=high(0)  R912:R23[7]   
		{
			R912_Array[15] = (R912_Array[15] & 0x1F) | ((u8)(XTAL_SMALL_HIGHEST-i)<<5) ;   
			R912_Array[19] = (R912_Array[19] & 0xFE) | 0x00 ;		//| 0x00;  //SMALL Gm(0)
		}
		else  // Xtal_pow0=highest(00)  R912:R23[6:5]   ;    Xtal_pow1=high(0)  R912:R23[7]
		{
			R912_Array[15] = (R912_Array[15] & 0x1F);
			R912_Array[19] = (R912_Array[19] & 0xFE) | 0x01 ;		//| 0x00;  //LARGE Gm(1)
		}

		R912_I2C.RegAddr = 0x17;
		R912_I2C.Data = R912_Array[15];
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_I2C.RegAddr = 0x1B;
		R912_I2C.Data = R912_Array[19];
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_Delay_MS(10);

		R912_I2C_Len.RegAddr = 0x00;
		R912_I2C_Len.Len = 3;
		if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
		{
			if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
				return RT_Fail;
		}
			

		//depend on init Nint & Div value (N=59.6667, Div=16)
		//lock to VCO band 8 if VCO=2576M for 16M Xtal
		//lock to VCO band 45 if VCO=2576M for 24M Xtal
		//lock to VCO band 54 if VCO=2576M for 27M Xtal
		if(R912_Xtal==16000)  //16M
			vco_check_bank = 8;
		else if(R912_Xtal==24000)
			vco_check_bank = 45;
		else
			vco_check_bank = 54;

		if(((R912_I2C_Len.Data[2] & 0x40) == 0x40) && ((R912_I2C_Len.Data[2] & 0x3F) <= (vco_check_bank + 6)) && ((R912_I2C_Len.Data[2] & 0x3F) >= (vco_check_bank-6))) 
		{
		    R912_Xtal_Pwr_tmp = i;
			break;
		}

	    if(i==(XTAL_CHECK_SIZE-1))
		{
	        R912_Xtal_Pwr_tmp = i;
		}
	}

    return RT_Success;
}	
R912_ErrCode R912_Cal_Prepare(struct r912_priv *priv, u8 u1CalFlag)
{
	 R912_Cal_Info_Type  Cal_Info;
	 u8 InitArrayCunt = 0;
 
	 switch(u1CalFlag)
	 {
	    case R912_IMR_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB		 R912:R38[3]
				//Cal_Info.RFBUF_OUT = 0x60;            //from RF_Buf ON, RF_Buf pwr off		 // R912:R12[5]
				//from RF_Buf ON, RF_Buf pwr off
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF,RF_Buf pwr off  //  R912:R8[7]
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x00;					// TF cal OFF, -6dB	OFF   // R912:R14[6:5]
				Cal_Info.MIXER_AMP_GAIN = 0x08;			//manual +8				  // R912:R15[4:0]
				Cal_Info.MIXER_BUFFER_GAIN = 0x10;		//manual min(0)			  // R912:R34[4:0]
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max		//  R912:R13[7:0]
				//Cal_Info.LNA_GAIN = 0x80;
				R912_IMR_Cal_Type = R912_IMR_CAL;
			break;
	    case R912_IMR_LNA_CAL:						    
				Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x00;              //from RF_Buf ON, RF_Buf pwr on

				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;             // LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x60;				   // TF cal ON, -6dB ON
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max
				//Cal_Info.LNA_GAIN = 0x80;
				R912_IMR_Cal_Type = R912_IMR_LNA_CAL;
			break;
            case R912_TF_CAL: //TBD
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off

				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF

				Cal_Info.RFBUF_OUT = 0x20;
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x00;					//TF cal OFF, -6dB OFF	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                  //manual: max
			break;
             case R912_TF_LNA_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB				
				//Cal_Info.RFBUF_OUT = 0x00;              //from RF_Buf ON, RF_Buf pwr on	

				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;				//LNA need on
				Cal_Info.TF_CAL = 0x60;					// TF cal ON, -6dB ON	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x80;                  //manual: min
			break;
	     case R912_LPF_CAL: 
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB						//  R912:R38[3]
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off
				//Cal_Info.RFBUF_OUT = 0x20;				//RF_Buf pwr off			//  R912:R12[5]
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF

				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF, TF cal OFF, -6dB OFF	
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on			   //  R912:R8[7]
				Cal_Info.TF_CAL = 0x00;					// TF cal OFF, -6dB OFF		// R912:R14[6:5]
				Cal_Info.MIXER_AMP_GAIN = 0x08;    //manual +8						// R912:R15[4:0]
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)					// R912:R34[4:0]	
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max				// R912:R13[7:0]
				R912_IMR_Cal_Type = R912_LPF_CAL;
			break;
	     case R912_LPF_LNA_CAL:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x00;               //from RF_Buf ON, RF_Buf pwr on
				Cal_Info.RFBUF_OUT = 0x00;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x00;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on
				Cal_Info.TF_CAL = 0x20;					// TF cal ON, -6dB OFF	
				Cal_Info.MIXER_AMP_GAIN = 0x00;    //manual min(0)
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x80;                  //manual: min
			break;
	     default:
			    Cal_Info.FILTER_6DB = 0x08;              //+6dB
				//Cal_Info.RFBUF_OUT = 0x60;               //from RF_Buf ON, RF_Buf pwr off
				Cal_Info.RFBUF_OUT = 0x20;				//from RF_Buf ON
				Cal_Info.RFBUF_POWER=0x04;				//RF_BUF_pwr OFF
				Cal_Info.LNA_POWER = 0x80;              //LNA power OFF
				//Cal_Info.LNA_POWER = 0x00;              //LNA need on
				Cal_Info.TF_CAL = 0x00;					//TF cal OFF, -6dB OFF
				Cal_Info.MIXER_AMP_GAIN = 0x08;    //manual +8
				Cal_Info.MIXER_BUFFER_GAIN = 0x10; //manual min(0)
				Cal_Info.LNA_GAIN = 0x9F;                 //manual: max
	 }

	  //Ring From RF_Buf Output & RF_Buf Power
	  //R912_I2C.RegAddr = 0x0C;
      R912_Array[4] = (R912_Array[4] & 0xDF) | Cal_Info.RFBUF_OUT;   //  R912:R12[5]  12-8=4  12(0x0C) is addr ; [4] is data


	  //RF_Buf Power
	  //R912_I2C.RegAddr = 0x09;
      R912_Array[1] = (R912_Array[1] & 0xFB) | Cal_Info.RFBUF_POWER;   //  R912:R12[5]  12-8=4  12(0x0C) is addr ; [4] is data

	  /*//TF cal (LNA power ON/OFF , TF cal ON/OFF, TF_-6dB ON/OFF)
	  R912_I2C.RegAddr = 0x06;
      R912_Array[6] = (R912_Array[6] & 0x1F) | Cal_Info.LNA_POWER;
      R912_I2C.Data = R912_Array[6];
      if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail; */


	  //(LNA power ON/OFF )
	  //R912_I2C.RegAddr = 0x08;
      R912_Array[0] = (R912_Array[0] & 0x7F) | Cal_Info.LNA_POWER;	 //  R912:R8[7]  8-8=0  8(0x08) is addr ; [0] is data
	  //R912_Array[0] = (R912_Array[0] & 0x80) 	 // R912:R8[7]  8-8=0  14(0x08) is addr ; [0] is data

	  //TF cal (TF cal ON/OFF, TF_-6dB ON/OFF)
	  //R912_I2C.RegAddr = 0x0E;
      R912_Array[6] = (R912_Array[6] & 0x9F) | Cal_Info.TF_CAL;	 // R912:R14[6:5]  14-8=6  14(0x0E) is addr ; [6] is data

	  //LNA gain
	  //R912_I2C.RegAddr = 0x0D;
	  R912_Array[5] = (R912_Array[5] & 0x60) | Cal_Info.LNA_GAIN; // R912:R13[7:0]  13-8=5  13(0x0D) is addr ; [5] is data

	  //Mixer Amp Gain
	  //R912_I2C.RegAddr = 0x0F;
	  R912_Array[7] = (R912_Array[7] & 0xE0) | Cal_Info.MIXER_AMP_GAIN; // R912:R15[4:0]  15-8=7  15(0x0F) is addr ; [7] is data

	  //Mixer Buffer Gain
	  //R912_I2C.RegAddr = 0x22;								// R912:R34[4:0]  34-8=26  34(0x22) is addr ; [26] is data
	  R912_Array[26] = (R912_Array[26] & 0xE0) | Cal_Info.MIXER_BUFFER_GAIN;  

	  // Set filter +0/6dB; NA det=OFF 
      //R912_I2C.RegAddr  = 0x26;								// R912:R38[3]  38-8=30  38(0x26) is addr ; [30] is data
	  R912_Array[30] = (R912_Array[30] & 0xF7) | Cal_Info.FILTER_6DB | 0x80;

	  //Set NA det 710 = OFF
	  //R912_I2C.RegAddr  = 0x28;								// R912:R40[3]  40-8=32  40(0x28) is addr ; [32] is data
	  R912_Array[32] = (R912_Array[32] | 0x08);

	 //---- General calibration setting ----//	 
	 //IMR IQ cap=0
	 //R912_I2C.RegAddr = 0x0B;		//R912:R11[1:0]  11-8=3  11(0x0B) is addr ; [3] is data
     R912_Array[3] = (R912_Array[3] & 0xFC);

	 // Set RF_Flag ON(%)
	 //R912_I2C.RegAddr = 0x16;		//R912:R22[0]  22-8=14  22(0x16) is addr ; [14] is data
     R912_Array[14] = R912_Array[14] | 0x01;  //force far-end mode

	 //RingPLL power ON
     //R912_I2C.RegAddr = 0x12;	  //R912:R18[4]  18-8=10  18(0x12) is addr ; [10] is data
     R912_Array[10] = (R912_Array[10] & 0xEF);

	 //LPF filter code = 15
     //R912_I2C.RegAddr = 0x12;	//R912:R18[3:0]  18-8=10  18(0x12) is addr ; [10] is data
     R912_Array[10] = (R912_Array[10] & 0xF0) | 0x0F;
	 
     //HPF corner=narrowest; LPF coarse=6M; 1.7M disable
     //R912_I2C.RegAddr = 0x13;	//R912:R19[7:0]  19-8=11  19(0x13) is addr ; [11] is data
     R912_Array[11] = (R912_Array[11] & 0x00) | 0x60;

     //ADC/VGA PWR on; Vga code mode(b4=1), Gain = 26.5dB; Large Code mode Gain(b5=1)
	 //ADC PWR on (b7=0)
	 //R912_I2C.RegAddr = 0x0F;	//R912:R15[7]  15-8=7  15(0x0F) is addr ; [7] is data
     R912_Array[7] = (R912_Array[7] & 0x7F);

	 //VGA PWR on (b0=0)
	 //R912_I2C.RegAddr = 0x09;	// R912:R9[0]  9-8=1  9(0x09) is addr ; [1] is data
     //R912_Array[1] = (R912_Array[1] & 0xFE);  
     //R912_I2C.Data = R912_Array[1];
     //if(I2C_Write(priv, &R912_I2C) != RT_Success)
     //      return RT_Fail;

	 //VGA PWR on (b0=0)  MT2
	 //R912_I2C.RegAddr = 0x12;	//R912:R18[7]  9-8=1  9(0x09) is addr ; [1] is data
     R912_Array[10] = (R912_Array[10] & 0x7F);  

	 //Large Code mode Gain(b5=1)
	 //R912_I2C.RegAddr = 0x0B;	//R912:R11[3]  11-8=3  11(0x0B) is addr ; [3] is data
     R912_Array[3] = (R912_Array[3] & 0xF7) | 0x08;  

	 //Vga code mode(b4=1)
	 //R912_I2C.RegAddr = 0x09;	//R912:R9[1]  9-8=1  9(0x09) is addr ; [1] is data
     R912_Array[1] = (R912_Array[1] & 0xFD) | 0x02;  

	 //Gain = 26.5dB
     //R912_I2C.RegAddr = 0x14;	//R912:R20[3:0]  20-8=12  20(0x14) is addr ; [12] is data
     R912_Array[12] = (R912_Array[12] & 0xF0) | 0x0B;  

	 //LNA, RF, Nrb dector pw on; det2 cap=IF_det 
     //R912_I2C.RegAddr = 0x25;	//R912:R37[3:0]  37-8=29  37(0x25) is addr ; [29] is data
     R912_Array[29] = (R912_Array[29] & 0xF0) | 0x02;


	 R912_I2C_Len.RegAddr = 0x08;   //  R912:0x08
	 R912_I2C_Len.Len = R912_REG_NUM;
	 for(InitArrayCunt = 0; InitArrayCunt<R912_REG_NUM; InitArrayCunt ++)
	 {
		 R912_I2C_Len.Data[InitArrayCunt] = R912_Array[InitArrayCunt];
	 }
	 if(I2C_Write_Len(priv,&R912_I2C_Len) != RT_Success)
		 return RT_Fail;

      return RT_Success;
}




R912_ErrCode R912_IMR(struct r912_priv *priv, u8 IMR_MEM, BOOL IM_Flag)
{
	u32 RingVCO = 0;
	u32 RingFreq = 0;
	u8  u1MixerGain = 8;
	u32 RingRef = R912_Xtal;
	u8   divnum_ring = 0;

	R912_SectType IMR_POINT;

	R912_Array[31] &= 0x3F;   //clear ring_div1, R24[7:6]	//R912:R39[7:6]  39-8=31  39(0x27) is addr ; [31] is data 
	R912_Array[25] &= 0xFC;   //clear ring_div2, R25[1:0]	//R912:R33[1:0]  33-8=25  33(0x21) is addr ; [25] is data 

	if(R912_Xtal==16000)  //16M
	{
         divnum_ring = 9;    //R39[5:2]=9  0x24
	}
	else if(R912_Xtal==24000) //24M
	{
		 divnum_ring = 1;		//R39[5:2]=1   0x04
	}
	else	//27MHz
	{
		 divnum_ring = 0;		//R39[5:2]=0   0x00
	}

	RingVCO = (16 + divnum_ring) * 8 * RingRef;


	if(RingVCO>=3200000)
	{
		R912_Array[25] &= 0xDF;   //clear vco_band, R25[5]		//R912:R33[5]    33-8=25  33(0x21) is addr ; [25] is data 
	}
	else
	{
		R912_Array[25] |= 0x20;   //clear vco_band, R25[5]		//R912:R33[5]    33-8=25  33(0x21) is addr ; [25] is data 
	}

	switch(IMR_MEM)
	{
	case 0: // RingFreq = 66.66M
		RingFreq = RingVCO/48;
		R912_Array[31] |= 0x80;  // ring_div1 /6 (2)
		R912_Array[25] |= 0x03;  // ring_div2 /8 (3)		
		u1MixerGain = 8;
		break;
	case 1: // RingFreq = 200M
		RingFreq = RingVCO/16;
		R912_Array[31] |= 0x00;  // ring_div1 /4 (0)
		R912_Array[25] |= 0x02;  // ring_div2 /4 (2)		
		u1MixerGain = 6;
		break;
	case 2: // RingFreq = 400M
		RingFreq = RingVCO/8;
		R912_Array[31] |= 0x00;  // ring_div1 /4 (0)
		R912_Array[25] |= 0x01;  // ring_div2 /2 (1)		
		u1MixerGain = 6;
		break;
	case 3: // RingFreq = 533.33M
		RingFreq = RingVCO/6;
		R912_Array[31] |= 0x80;  // ring_div1 /6 (2)
		R912_Array[25] |= 0x00;  // ring_div2 /1 (0)		
		u1MixerGain = 8;
		break;
	case 4: // RingFreq = 800M
		RingFreq = RingVCO/4;
		R912_Array[31] |= 0x00;  // ring_div1 /4 (0)
		R912_Array[25] |= 0x00;  // ring_div2 /1 (0)		
		u1MixerGain = 8;
		break;
	default:
		RingFreq = RingVCO/4;
		R912_Array[31] |= 0x00;  // ring_div1 /4 (0)
		R912_Array[25] |= 0x00;  // ring_div2 /1 (0)		
		u1MixerGain = 8;
		break;
	}


	//Mixer Amp Gain
	//R912_I2C.RegAddr = 0x0F;	//R912:R15[4:0] 
	R912_Array[7] = (R912_Array[7] & 0xE0) | u1MixerGain; 

	//write I2C to set RingPLL
	//R912_I2C.RegAddr = 0x27;
	R912_Array[31]=(R912_Array[31] & 0xC3) | (divnum_ring << 2);

	//Ring PLL power
	//if((RingFreq>=0) && (RingFreq<R912_RING_POWER_FREQ_LOW))
	if((RingFreq>=0) && ((RingFreq<R912_RING_POWER_FREQ_LOW)||(RingFreq>R912_RING_POWER_FREQ_HIGH)))  //R912:R33[3:2] 
         R912_Array[25] = (R912_Array[25] & 0xF3) | 0x08;   //R25[3:2]=2'b10; min_lp
	else
        R912_Array[25] = (R912_Array[25] & 0xF3) | 0x00;   //R25[3:2]=2'b00; min

	
	//Must do MUX before PLL() 
	if(R912_MUX(priv,RingFreq - R912_IMR_IF, RingFreq, R912_STD_SIZE) != RT_Success)      //IMR MUX (LO, RF)
		return RT_Fail;

	if(R912_PLL(priv,(RingFreq - R912_IMR_IF), R912_STD_SIZE) != RT_Success)  //IMR PLL
	    return RT_Fail;

	//Set TF, place after R912_MUX( )
	//TF is dependent to LNA/Mixer Gain setting
	if(R912_SetTF(priv,RingFreq, (u8)R912_SetTfType) != RT_Success)
		return RT_Fail;

	//clear IQ_cap
	IMR_POINT.Iqcap = R912_Array[3] & 0xFC; // R912:R11[1:0] 

	if(IM_Flag == R912_TRUE)
	{
	     if(R912_IQ(priv,&IMR_POINT) != RT_Success)
		    return RT_Fail;
	}
	else
	{
		IMR_POINT.Gain_X = R912_IMR_Data[3].Gain_X;
		IMR_POINT.Phase_Y = R912_IMR_Data[3].Phase_Y;
		IMR_POINT.Value = R912_IMR_Data[3].Value;
		//IMR_POINT.Iqcap = R912_IMR_Data[3].Iqcap;
		if(R912_F_IMR(priv,&IMR_POINT) != RT_Success)
			return RT_Fail;
	}

	//Save IMR Value
	switch(IMR_MEM)
	{
	case 0:
		R912_IMR_Data[0].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[0].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[0].Value = IMR_POINT.Value;
		R912_IMR_Data[0].Iqcap = IMR_POINT.Iqcap;		
		break;
	case 1:
		R912_IMR_Data[1].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[1].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[1].Value = IMR_POINT.Value;
		R912_IMR_Data[1].Iqcap = IMR_POINT.Iqcap;
		break;
	case 2:
		R912_IMR_Data[2].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[2].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[2].Value = IMR_POINT.Value;
		R912_IMR_Data[2].Iqcap = IMR_POINT.Iqcap;
		break;
	case 3:
		R912_IMR_Data[3].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[3].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[3].Value = IMR_POINT.Value;
		R912_IMR_Data[3].Iqcap = IMR_POINT.Iqcap;
		break;
	case 4:
		R912_IMR_Data[4].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[4].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[4].Value = IMR_POINT.Value;
		R912_IMR_Data[4].Iqcap = IMR_POINT.Iqcap;
		break;
        default:
		R912_IMR_Data[4].Gain_X  = IMR_POINT.Gain_X;
		R912_IMR_Data[4].Phase_Y = IMR_POINT.Phase_Y;
		R912_IMR_Data[4].Value = IMR_POINT.Value;
		R912_IMR_Data[4].Iqcap = IMR_POINT.Iqcap;
		break;
	}
	return RT_Success;
}


R912_ErrCode R912_PLL(struct r912_priv *priv, u32 LO_Freq, R912_Standard_Type R912_Standard)
{
	u8 InitArrayCunt = 0;
	u8  MixDiv = 2;
	u8  DivBuf = 0;
	u8  Ni = 0;
	u8  Si = 0;
	u8  DivNum = 0;
	u16  Nint = 0;
	u32 VCO_Min = 2410000; 
	u32 VCO_Max = VCO_Min*2;
	u32 VCO_Freq = 0;
	u32 PLL_Ref	= R912_Xtal;		
	u32 VCO_Fra	= 0;		
	u16 Nsdm = 2;
	u16 SDM = 0;
	u16 SDM16to9 = 0;
	u16 SDM8to1 = 0;
	u8   CP_CUR = 0x00;
	u8   CP_OFFSET = 0x00;
	u8   SDM_RES = 0x00;
	u8   XTAL_POW1 = 0x00;
	u8   XTAL_POW0 = 0x00;
	u8   XTAL_GM = 0x00;
	u16  u2XalDivJudge;
	u8   u1XtalDivRemain;
	u8   VCO_current_trial = 0;

	u8   u1RfFlag = 0;
	u8   u1PulseFlag = 0;
	u8   u1SPulseFlag=0;
	u8   cp_cur_24[50] = {
			5, 5, 5, 5, 5, 4, 0, 4, 5, 4, 
			5, 3, 5, 3, 3, 0, 3, 0, 0, 0, 
			0, 0, 0, 0, 0, 0, 5, 0, 3, 0,
			4, 0, 2, 0, 4, 0, 3, 5, 1, 5,
			0, 5, 2, 5, 3, 5, 3, 5, 1, 5};

	u8   cp_cur_27[38] = {
			0, 0, 0, 0, 0, 0, 1, 5, 1, 5, 
			4, 5, 4, 0, 4, 0, 4, 0, 0, 0, 
			0, 0, 3, 1, 4, 1, 1, 1, 4, 0,
			2, 0, 4, 5, 2, 5, 3, 2};

	//112MHz to 176MHz CP Set Value
	u8   cp_cur_16_low[5] = {
			5, 2, 3, 1, 3};
	//304MHz to 496MHz CP Set Value
	u8   cp_cur_16_mid[25] = {
			2, 0, 4, 0, 3, 0, 1, 0, 4, 0, 
			2, 0, 3, 0, 4, 0, 1, 5, 1, 5, 
			1, 5, 3, 5, 4};

	//TF, NA fix
	u1RfFlag = (R912_Array[14] & 0x01);      //R22[0]
	u1PulseFlag = (R912_Array[30] & 0x80);   //R38[7]
	u1SPulseFlag= (R912_Array[32] & 0x08);   //R40[3]


	//R912_I2C.RegAddr = 0x16;
	R912_Array[14] = R912_Array[14] | 0x01;		// TF force sharp mode


	//R912_I2C.RegAddr = 0x26;	
	R912_Array[30] = R912_Array[30] | 0x80;		// NA det off

	//Set NA det 710 = OFF
	//R912_I2C.RegAddr  = 0x28;								
	R912_Array[32] = (R912_Array[32] | 0x08);

	//Xtal cap
	//R912_I2C.RegAddr = 0x1B;
	if((R912_Xtal==16000)||(R912_Xtal==27000))   // Xtal cap =16pF  (16MHz)
		R912_Array[19] = (R912_Array[19] & 0xDF);
	else				// Xtal cap =10pF  (24MHz)
		R912_Array[19] = (R912_Array[19] | 0x20) ;


	SDM_RES = 0x00;    //short, R27[4:3]=00
	if(R912_Xtal==16000)
	{
		//cp cur & offset setting
		if(R912_Standard < R912_ATV_SIZE) //ATV
		{
			u2XalDivJudge = (u16) (LO_Freq/1000/8);

			if((u2XalDivJudge==6)||(u2XalDivJudge==14)||(u2XalDivJudge==16)||(u2XalDivJudge==18)||(u2XalDivJudge==20))
				//SDM_RES = 0x18;    //400R, R27[4:3]=11
				SDM_RES = 0x10;    //200R, R27[4:3]=10
			else
				SDM_RES = 0x00;    //short, R27[4:3]=00

			//offset
			if(LO_Freq < (160000+R912_IF_HIGH))  //  R912:R21[7]  21-8=13  21(0x15) is addr ; [13] is data	// R912:R21[7]
			{
				CP_OFFSET = 0x80;  //30u,   [2]=1
			}
			else if ((u2XalDivJudge>=38)&&(u2XalDivJudge<63))
			{
				if((u2XalDivJudge % 2)==1) //odd
					CP_OFFSET = 0x00;  //0u,   [2]=0
				else
					CP_OFFSET = 0x80;  //30u,     [2]=1
			}
			else 
			{
				CP_OFFSET = 0x00;  //0u,     [2]=0
			}

			//current
			if(LO_Freq < (48000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000
			else if(LO_Freq < (64000+R912_IF_HIGH))
				CP_CUR = 0x40;        //0.3m, [6:4]=100
			else if(LO_Freq < (80000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000
			else if(LO_Freq < (104000+R912_IF_HIGH))
				CP_CUR = 0x40;        //0.3m, [6:4]=100
			else if(LO_Freq < (176000+R912_IF_HIGH))
				if(u2XalDivJudge % 2 == 1) //odd
					CP_CUR = 0x30;        //0.4m, [6:4]=011
				else
					CP_CUR = ( 0x00 | ( cp_cur_16_low[(u2XalDivJudge-14)/2] << 4));  //14, 16, 18, 20, 22
			else if(LO_Freq < (296000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000		
			else if(LO_Freq < (496000+R912_IF_HIGH))
				CP_CUR = ( 0x00 | ( cp_cur_16_mid[(u2XalDivJudge-38)] << 4));
			else if(LO_Freq < (592000+R912_IF_HIGH))
				CP_CUR = 0x50;        //0.2m, [6:4]=101
			else if(LO_Freq < (744000+R912_IF_HIGH))
				CP_CUR = 0x40;        //0.3m, [6:4]=100
			else if(LO_Freq < (752000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000
			else
				CP_CUR = 0x40;        //0.3m, [6:4]=100
		}
		else
		{
			//DTV
			CP_CUR = 0x00;     //0.7m, R25[6:4]=000
			CP_OFFSET = 0x00;  //0u,     [2]=0
		}
	}
	else if(R912_Xtal==24000)   //24MHz 
	{
		//cp cur & offset setting
		if(R912_Standard < R912_ATV_SIZE) //ATV
		{
			u2XalDivJudge = (u16) (LO_Freq/1000/12);

			if((u2XalDivJudge==6)||(u2XalDivJudge==10)||(u2XalDivJudge==12))
				//SDM_RES = 0x18;    //400R, R27[4:3]=11
				SDM_RES = 0x10;    //200R, R27[4:3]=10
			else
				SDM_RES = 0x00;    //short, R27[4:3]=00
			//offset
			if((LO_Freq < (160000+R912_IF_HIGH))||(u2XalDivJudge==16))  //R912:R21[7]  21-8=13  21(0x15) is addr ; [13] is data
			{
				CP_OFFSET = 0x80;  //30u,   [2]=1
			}
			else if ((u2XalDivJudge>=25)&&(u2XalDivJudge<50))
			{
				if((u2XalDivJudge % 2)==1) //odd
					CP_OFFSET = 0x00;  //0u,   [2]=0
				else
					CP_OFFSET = 0x80;  //30u,     [2]=1
			}
			else 
			{
				CP_OFFSET = 0x00;  //0u,     [2]=0
			}

			//CP Current
			if(LO_Freq < (600000+R912_IF_HIGH))
				CP_CUR = ( 0x00 | ( cp_cur_24[u2XalDivJudge] << 4));
			else if(LO_Freq < (744000+R912_IF_HIGH))
				CP_CUR = 0x40;        //0.3m, [6:4]=100
			else if(LO_Freq < (752000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000
			else
				CP_CUR = 0x40;        //0.3m, [6:4]=100
		}
		else
		{
			//DTV
			CP_CUR = 0x00;     //0.7m, R25[6:4]=000
			CP_OFFSET = 0x00;  //0u,     [2]=0
		}
	}
	else	//27MHz
	{
		//cp cur & offset setting
		if(R912_Standard < R912_ATV_SIZE) //ATV
		{
			u2XalDivJudge = (u16) (LO_Freq/1000/13.5);

			if(LO_Freq <(216000+R912_IF_HIGH))
				//SDM_RES = 0x18;    //400R, R27[4:3]=11
				SDM_RES = 0x10;    //200R, R27[4:3]=10
			else
				SDM_RES = 0x00;    //short, R27[4:3]=00

			//offset
			if(LO_Freq < (189000+R912_IF_HIGH))  //R912:R21[7]  21-8=13  21(0x15) is addr ; [13] is data
			{
				CP_OFFSET = 0x80;  //30u,   [2]=1
			}
			else if ((u2XalDivJudge>=22)&&(u2XalDivJudge<37))
			{
				if((u2XalDivJudge % 2)==1) //odd
					CP_OFFSET = 0x00;  //0u,   [2]=0
				else
					CP_OFFSET = 0x80;  //30u,     [2]=1
			}
			else 
			{
				CP_OFFSET = 0x00;  //0u,     [2]=0
			}

			//CP Current
			if(LO_Freq < (500000+R912_IF_HIGH))
				CP_CUR = ( 0x00 | ( cp_cur_27[u2XalDivJudge] << 4));
			else if(LO_Freq < (600000+R912_IF_HIGH))
				CP_CUR = 0x20;        //0.5m, [6:4]=010
			else if(LO_Freq < (744000+R912_IF_HIGH))
				CP_CUR = 0x40;        //0.3m, [6:4]=100
			else if(LO_Freq < (752000+R912_IF_HIGH))
				CP_CUR = 0x00;        //0.7m, [6:4]=000
			else
				CP_CUR = 0x40;        //0.3m, [6:4]=100
		}
		else
		{
			//DTV
			CP_CUR = 0x00;     //0.7m, R25[6:4]=000
			CP_OFFSET = 0x00;  //0u,     [2]=0
		}
	}
	

	//SDM_res
	//SDM_RES = 0x00;    //short, R27[4:3]=00
	//R912_I2C.RegAddr = 0x1B;
	R912_Array[19]    = (R912_Array[19] & 0xE7) | SDM_RES; 



	//CP current  R25[6:4]=000
	//R912_I2C.RegAddr = 0x19; 
	R912_Array[17]    = (R912_Array[17] & 0x8F)  | CP_CUR ;


	//Div Cuurent   R20[7:6]=2'b01(150uA)	
	//R912_I2C.RegAddr = 0x14;    
	R912_Array[12]    = (R912_Array[12] & 0x3F)  | 0x40;  


	//CPI*2 R28[7]=1
	if((R912_Standard!=R912_DVB_S) && (LO_Freq >= 865000))
	{
		//R912_I2C.RegAddr = 0x1C;
		R912_Array[20] = (R912_Array[20] & 0x7F) | 0x80;
	}
	else
	{
		//R912_I2C.RegAddr = 0x1C;
		R912_Array[20] = (R912_Array[20] & 0x7F);
	}

	//  R912:R26[7:5]  VCO_current= 2
	//R912_I2C.RegAddr = 0x1A;  
	R912_Array[18]    = (R912_Array[18] & 0x1F) | 0x40; 


	//CP Offset R21[7] 
	//R912_I2C.RegAddr = 0x15;  
	R912_Array[13]    = (R912_Array[13] & 0x7F) | CP_OFFSET; 

	//set XTAL Power
#if(R912_SHARE_XTAL_OUT==R912_TRUE)
	XTAL_POW1 = 0x80;        //Low,     // R912:R23[7]    
	XTAL_POW0 = 0x00;        //highest,  // R912:R23[6:5]
	XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1

	R912_SetXtalIntCap(priv, XTAL_CAP_SMALL);

#else
	if(R912_Initial_done_flag==R912_TRUE)
	{
		if(R912_Xtal_Pwr < XTAL_SMALL_HIGH)
		{
			XTAL_POW1 = 0x00;    // high(0),    R912:R23[7]    
			XTAL_POW0 = 0x20;	// high(01),		R912:R23[6:5]      
			XTAL_GM = 0x00;            //SMALL(0),   R27[0]=0
		}
		else if(R912_Xtal_Pwr <= XTAL_SMALL_HIGHEST)
		{
			XTAL_POW1 = 0x00;        //high,    R912:R23[7]    
			XTAL_POW0 = ((u8)(XTAL_SMALL_HIGHEST-R912_Xtal_Pwr)<<5);	//R912:R23[6:5]      
			XTAL_GM = 0x00;            //SMALL(0),   R27[0]=0
		}
		else if(R912_Xtal_Pwr == XTAL_LARGE_HIGHEST)
		{
			XTAL_POW1 = 0x00;        //high,      	// R912:R23[7]  
			XTAL_POW0 = 0x00;        //highest,  	// R912:R23[6:5] 
			XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1
		}
		else
		{
			XTAL_POW1 = 0x80;        //Low,     // R912:R23[7]    
			XTAL_POW0 = 0x00;        //highest,  // R912:R23[6:5]
			XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1
		}
	}
	else
	{
		XTAL_POW1 = 0x80;        //Low,      	// R912:R23[7]  
		XTAL_POW0 = 0x00;        //highest,  	// R912:R23[6:5] 
		XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1
	}
#endif

	//Xtal_Gm R27[0]
	//R912_I2C.RegAddr = 0x1B;
	R912_Array[19] = (R912_Array[19] & 0xFE) | XTAL_GM;


	//R912_I2C.RegAddr = 0x17;		// XTAL_POW0:R23[6:5] ;  XTAL_POW1:R23[7]  
	R912_Array[15]    = ((R912_Array[15] & 0x1F) | XTAL_POW0) | XTAL_POW1; 

	//IQ gen ON 
	//R912_I2C.RegAddr = 0x27;		// R39[1]
	R912_Array[31]    = (R912_Array[31] & 0xFD) | 0x00; //[0]=0'b0

	// current:Dmin, Bmin
	//R912_I2C.RegAddr = 0x23;		// R912:R35[5:4]=2'b00
	R912_Array[27]    = (R912_Array[27] & 0xCF) | 0x00;

	//set pll autotune = 128kHz (fast)  R23[4:3]=2'b00   
	//R912_I2C.RegAddr = 0x17;
	R912_Array[15]    = R912_Array[15] & 0xE7;

	//Divider
	while(MixDiv <= 64)
	{
		if(((LO_Freq * MixDiv) >= VCO_Min) && ((LO_Freq * MixDiv) < VCO_Max))
		{
			DivBuf = MixDiv;
			while(DivBuf > 2)
			{
				DivBuf = DivBuf >> 1;
				DivNum ++;
			}			
			break;
		}
		MixDiv = MixDiv << 1;
	}

	//SDM_Res
	/*if(MixDiv <= 4)  //Div=2,4
	{
		SDM_RES = 0x00;    //short, R27[4:3]=00
	}
	else
	{
		SDM_RES = 0x18;   //400R, R27[4:3]=11	  
	}
	SDM_RES = 0x00;    //short, R27[4:3]=00
	R912_I2C.RegAddr = 0x1B;
	R912_Array[19]    = (R912_Array[19] & 0xE7) | SDM_RES; 
	R912_I2C.Data    = R912_Array[19];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;
	*/

	//Xtal Div
	if( (R912_Standard == R912_STD_SIZE) || (R912_Standard == R912_DVB_S) ) //for cal and Satellite
	{
		R912_XtalDiv = XTAL_DIV1;
	    R912_Array[16] = R912_Array[16] & 0xFB; //b2=0  // R912:R24[2]   
	    PLL_Ref = R912_Xtal;
	}
	else if( (R912_Xtal==16000) || (R912_Xtal==24000))	//16MHz, 24MHz (ATV_DTV_Standard)
	{
			if(R912_Xtal==16000)		//16MHz
				u2XalDivJudge = (u16) (LO_Freq/1000/8);
			else if(R912_Xtal==24000)	//24MHz
				u2XalDivJudge = (u16) (LO_Freq/1000/12);

			u1XtalDivRemain = (u8) (u2XalDivJudge % 2);
		   if(u1XtalDivRemain==1) //odd
		   {
				R912_XtalDiv = XTAL_DIV1;
				R912_Array[16] = R912_Array[16] & 0xFB; //R24[2]=0  
				PLL_Ref = R912_Xtal;
			}
			else  // div2, note that agc clk also div2
			{
				R912_XtalDiv = XTAL_DIV2;
				R912_Array[16] |= 0x04;   	//R24[2]=1
				PLL_Ref = R912_Xtal / 2;
			}
	}
	else	//27MHz Div/2
	{
		R912_XtalDiv = XTAL_DIV2;
		R912_Array[16] |= 0x04;   	//R24[2]=1
		PLL_Ref = R912_Xtal / 2;
	}

/*
	R912_I2C_Len.RegAddr = 0x00;
	R912_I2C_Len.Len = 5;
	if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
		return RT_Fail;	

   if((R912_I2C_Len.Data[3] & 0xE0) < 0xE0)         //0x60
		DivNum = DivNum + 1;
   else if((R912_I2C_Len.Data[3] & 0xE0) > 0xE0)   //0x60
	    DivNum = DivNum - 1; 
*/	

	//Divider num
	//R912_I2C.RegAddr = 0x18; //R24[7:5] 
	R912_Array[16] &= 0x1F;
	R912_Array[16] |= (DivNum << 5);


	VCO_Freq = LO_Freq * MixDiv;
	Nint     = (u16) (VCO_Freq / 2 / PLL_Ref);
	VCO_Fra  = (u16) (VCO_Freq - 2 * PLL_Ref * Nint);

	//Boundary spur prevention
	if (VCO_Fra < PLL_Ref/64)           //2*PLL_Ref/128
		VCO_Fra = 0;
	else if (VCO_Fra > PLL_Ref*127/64)  //2*PLL_Ref*127/128
	{
		VCO_Fra = 0;
		Nint ++;
	}
	else if((VCO_Fra > PLL_Ref*127/128) && (VCO_Fra < PLL_Ref)) //> 2*PLL_Ref*127/256,  < 2*PLL_Ref*128/256
		VCO_Fra = PLL_Ref*127/128;      // VCO_Fra = 2*PLL_Ref*127/256
	else if((VCO_Fra > PLL_Ref) && (VCO_Fra < PLL_Ref*129/128)) //> 2*PLL_Ref*128/256,  < 2*PLL_Ref*129/256
		VCO_Fra = PLL_Ref*129/128;      // VCO_Fra = 2*PLL_Ref*129/256
	else
		VCO_Fra = VCO_Fra;

	//Ni:R912:R28[6:0]   Si:R912:R20[5:4]
	Ni = (u8) ((Nint - 13) / 4);
	Si = (u8) (Nint - 4 *Ni - 13);
	//Si
	//R912_I2C.RegAddr = 0x14;
	R912_Array[12] = (R912_Array[12] & 0xCF) | ((Si << 4));

	//Ni
	//R912_I2C.RegAddr = 0x1C;
	R912_Array[20] = (R912_Array[20] & 0x80) | (Ni);

         	
	//pw_sdm		// R912:R27[7]  
	//R912_I2C.RegAddr = 0x1B;
	R912_Array[19] &= 0x7F;
	if(VCO_Fra == 0)
		R912_Array[19] |= 0x80;

	//SDM calculator
	while(VCO_Fra > 1)
	{			
		if (VCO_Fra > (2*PLL_Ref / Nsdm))
		{		
			SDM = SDM + 32768 / (Nsdm/2);
			VCO_Fra = VCO_Fra - 2*PLL_Ref / Nsdm;
			if (Nsdm >= 0x8000)
				break;
		}
		Nsdm = Nsdm << 1;
	}

	SDM16to9 = SDM >> 8;
	SDM8to1 =  SDM - (SDM16to9 << 8);

	// R912:R30[7:0]  
	//R912_I2C.RegAddr = 0x1E;
	R912_Array[22]    = (u8) SDM16to9;

	//R912:R29[7:0] 
	//R912_I2C.RegAddr = 0x1D;
	R912_Array[21]    = (u8) SDM8to1;


	R912_I2C_Len.RegAddr = 0x08;   //  R912:0x08
	R912_I2C_Len.Len = R912_REG_NUM;
	for(InitArrayCunt = 0; InitArrayCunt<R912_REG_NUM; InitArrayCunt ++)
	{
		R912_I2C_Len.Data[InitArrayCunt] = R912_Array[InitArrayCunt];
	}
	if(I2C_Write_Len(priv,&R912_I2C_Len) != RT_Success)
		return RT_Fail;


	//if(R912_Standard <= R912_SECAM_L1_INV)
	if(R912_XtalDiv == XTAL_DIV2)
	    R912_Delay_MS(20);
	else
	    R912_Delay_MS(10);

	for(VCO_current_trial=0; VCO_current_trial<3; VCO_current_trial++)
	{
		//check PLL lock status 
		R912_I2C_Len.RegAddr = 0x00;
		R912_I2C_Len.Len = 3;
		if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
		{	
			I2C_Read_Len(priv,&R912_I2C_Len);
		}

		// R912:R26[7:5] 
		if( (R912_I2C_Len.Data[2] & 0x40) == 0x00 ) 
		{
			//Set VCO current = 011 (3)
			R912_I2C.RegAddr = 0x1A;
			//R912_Array[18]    = (R912_Array[18] & 0x1F) | 0x60;  //increase VCO current
			R912_Array[18]    = (R912_Array[18] & 0x1F) | ((2-VCO_current_trial) << 5);  //increase VCO current
			R912_I2C.Data    = R912_Array[18];
			if(I2C_Write(priv, &R912_I2C) != RT_Success)
				return RT_Fail;
		}
		else
		{
			break;
		}
	}

	if(VCO_current_trial>=2)
	{
		//check PLL lock status 
		R912_I2C_Len.RegAddr = 0x00;
		R912_I2C_Len.Len = 3;
		if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
			return RT_Fail;

		if( (R912_I2C_Len.Data[2] & 0x40) == 0x00) 
		{
			XTAL_POW1 = 0x80;        //Low,      	// R912:R23[7]  
			XTAL_POW0 = 0x00;        //highest,  	// R912:R23[6:5] 
			XTAL_GM = 0x01;          //LARGE(1),         R27[0]=1

			R912_I2C.RegAddr = 0x17;		// XTAL_POW0:R23[6:5] ;  XTAL_POW1:R23[7]  
			R912_Array[15]    = ((R912_Array[15] & 0x1F) | XTAL_POW0) | XTAL_POW1; 
			R912_I2C.Data    = R912_Array[15];
			if(I2C_Write(priv, &R912_I2C) != RT_Success)
				return RT_Fail;

			R912_I2C.RegAddr = 0x1B;
			R912_Array[19] = (R912_Array[19] & 0xFE) | XTAL_GM;
			R912_I2C.Data = R912_Array[19];
			if(I2C_Write(priv, &R912_I2C) != RT_Success)
				return RT_Fail;
		}
	}

	//set pll autotune = 8kHz (slow)
	R912_I2C.RegAddr = 0x17;
	R912_Array[15] = (R912_Array[15] & 0xE7) | 0x10;
	R912_I2C.Data = R912_Array[15];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
		

	//restore TF, NA det setting
	R912_I2C.RegAddr = 0x16;
	R912_Array[14] = (R912_Array[14] & 0xFE) | u1RfFlag;     
	R912_I2C.Data = R912_Array[14];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;

	R912_I2C.RegAddr = 0x26;
	R912_Array[30] = (R912_Array[30] & 0x7F) | u1PulseFlag;  
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	    return RT_Fail;

	//Set NA det 710 = OFF
	R912_I2C.RegAddr  = 0x28;								
	R912_Array[32] = (R912_Array[32] & 0xF7) | u1SPulseFlag;
    R912_I2C.Data = R912_Array[32];
    if(I2C_Write(priv, &R912_I2C) != RT_Success)
	   return RT_Fail;

	return RT_Success;

}


R912_ErrCode R912_MUX(struct r912_priv *priv, u32 LO_KHz, u32 RF_KHz, R912_Standard_Type R912_Standard)
{	


	u8 Reg08_IMR_Gain   = 0;
	u8 Reg09_IMR_Phase  = 0;
	u8 Reg03_IMR_Iqcap  = 0;

	//Freq_Info_Type Freq_Info1;
	Freq_Info1 = R912_Freq_Sel(priv,LO_KHz, RF_KHz, R912_Standard);

	// LNA band (depend on RF_KHz)
	R912_Array[5] = (R912_Array[5] & 0x9F) | Freq_Info1.LNA_BAND;

	// RF Polyfilter
	R912_Array[25] = (R912_Array[25] & 0x3F) | Freq_Info1.RF_POLY;

	// LNA Cap
	R912_Array[1] = (R912_Array[1] & 0x07) | (Freq_Info1.LPF_CAP<<3);	

	// LNA Notch
	R912_Array[2] = (R912_Array[2] & 0xE0) | (Freq_Info1.LPF_NOTCH);	


	//Set_IMR
	if(R912_IMR_done_flag == R912_TRUE)
	{
		Reg08_IMR_Gain = R912_IMR_Data[Freq_Info1.IMR_MEM].Gain_X & 0x3F;
		Reg09_IMR_Phase = R912_IMR_Data[Freq_Info1.IMR_MEM].Phase_Y & 0x3F;
		Reg03_IMR_Iqcap = R912_IMR_Data[Freq_Info1.IMR_MEM].Iqcap & 0x03;
	}
	else
	{
		Reg08_IMR_Gain = 0;
	    Reg09_IMR_Phase = 0;
		Reg03_IMR_Iqcap = 0;
	}

	// R16[5:0]            
	R912_Array[8] = (R912_Array[8] & 0xC0) | Reg08_IMR_Gain;

	// R17[5:0]  
	R912_Array[9] = (R912_Array[9] & 0xC0) | Reg09_IMR_Phase;

	// R11[1:0]  
	R912_Array[3] = (R912_Array[3] & 0xFC) | Reg03_IMR_Iqcap;

	return RT_Success;
}


R912_ErrCode R912_SetTF(struct r912_priv *priv, u32 u4FreqKHz, u8 u1TfType)
{
    u8    u1FreqCount = 0;
	u32   u4Freq1 = 0;
	u32   u4Freq2 = 0;
	u32   u4Ratio;
	u8    u1TF_Set_Result1 = 0;
	u8    u1TF_Set_Result2 = 0;
	u8    u1TF_tmp1, u1TF_tmp2;
	//u8    u1TFCalNum = R912_TF_HIGH_NUM;

	if((u4FreqKHz>=0) && (u4FreqKHz<R912_LNA_LOW_LOWEST[R912_SetTfType]))  //Ultra Low
	{
		 //u1TFCalNum = R912_TF_LOWEST_NUM;
         while((u4FreqKHz < R912_TF_Freq_Lowest[u1TfType][u1FreqCount]) && (u1FreqCount<R912_TF_LOWEST_NUM))
		 {
            u1FreqCount++;
		 }

		 if(u1FreqCount==0)
		 {
			 R912_TF = R912_TF_Result_Lowest[u1TfType][0];
		 }
		 else if(u1FreqCount==R912_TF_LOWEST_NUM)
         {
			 R912_TF = R912_TF_Result_Lowest[u1TfType][R912_TF_LOWEST_NUM-1];
		 }
		 else
		 {
			 u1TF_Set_Result1 = R912_TF_Result_Lowest[u1TfType][u1FreqCount-1]; 
		     u1TF_Set_Result2 = R912_TF_Result_Lowest[u1TfType][u1FreqCount]; 
		     u4Freq1 = R912_TF_Freq_Lowest[u1TfType][u1FreqCount-1];
		     u4Freq2 = R912_TF_Freq_Lowest[u1TfType][u1FreqCount]; 

			 //u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
             //R912_TF = u1TF_Set_Result1 + (u8)((u1TF_Set_Result2 - u1TF_Set_Result1)*u4Ratio/100);

			 u1TF_tmp1 = ((u1TF_Set_Result1 & 0x40)>>2)*3 + (u1TF_Set_Result1 & 0x3F);  //b6 is 3xb4
			 u1TF_tmp2 = ((u1TF_Set_Result2 & 0x40)>>2)*3 + (u1TF_Set_Result2 & 0x3F);			 
			 u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
			 R912_TF = u1TF_tmp1 + (u8)((u1TF_tmp2 - u1TF_tmp1)*u4Ratio/100);
			 if(R912_TF>=0x40)
				 R912_TF = (R912_TF + 0x10);

		 }
	}
	else if((u4FreqKHz>=R912_LNA_LOW_LOWEST[R912_SetTfType]) && (u4FreqKHz<R912_LNA_MID_LOW[R912_SetTfType]))  //Low
	{
		 //u1TFCalNum = R912_TF_LOW_NUM;
         while((u4FreqKHz < R912_TF_Freq_Low[u1TfType][u1FreqCount]) && (u1FreqCount<R912_TF_LOW_NUM))
		 {
            u1FreqCount++;
		 }

		 if(u1FreqCount==0)
		 {
			 R912_TF = R912_TF_Result_Low[u1TfType][0];
		 }
		 else if(u1FreqCount==R912_TF_LOW_NUM)
        {
			 R912_TF = R912_TF_Result_Low[u1TfType][R912_TF_LOW_NUM-1];
		 }
		 else
		 {
			 u1TF_Set_Result1 = R912_TF_Result_Low[u1TfType][u1FreqCount-1]; 
		     u1TF_Set_Result2 = R912_TF_Result_Low[u1TfType][u1FreqCount]; 
		     u4Freq1 = R912_TF_Freq_Low[u1TfType][u1FreqCount-1];
		     u4Freq2 = R912_TF_Freq_Low[u1TfType][u1FreqCount]; 

			 //u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
             //R912_TF = u1TF_Set_Result1 + (u8)((u1TF_Set_Result2 - u1TF_Set_Result1)*u4Ratio/100);

			 u1TF_tmp1 = ((u1TF_Set_Result1 & 0x40)>>2) + (u1TF_Set_Result1 & 0x3F);  //b6 is 1xb4
			 u1TF_tmp2 = ((u1TF_Set_Result2 & 0x40)>>2) + (u1TF_Set_Result2 & 0x3F);			 
			 u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
			 R912_TF = u1TF_tmp1 + (u8)((u1TF_tmp2 - u1TF_tmp1)*u4Ratio/100);
			 if(R912_TF>=0x40)
				 R912_TF = (R912_TF + 0x30);
		 }
	}
	else if((u4FreqKHz>=R912_LNA_MID_LOW[R912_SetTfType]) && (u4FreqKHz<R912_LNA_HIGH_MID[R912_SetTfType]))  //Mid
    {
		 //u1TFCalNum = R912_TF_MID_NUM;
         while((u4FreqKHz < R912_TF_Freq_Mid[u1TfType][u1FreqCount]) && (u1FreqCount<R912_TF_MID_NUM))
		 {
            u1FreqCount++;
		 }

		 if(u1FreqCount==0)
		 {
			 R912_TF = R912_TF_Result_Mid[u1TfType][0];
		 }
		 else if(u1FreqCount==R912_TF_MID_NUM)
        {
			 R912_TF = R912_TF_Result_Mid[u1TfType][R912_TF_MID_NUM-1];
		 }
		 else
		 {
			 u1TF_Set_Result1 = R912_TF_Result_Mid[u1TfType][u1FreqCount-1]; 
		     u1TF_Set_Result2 = R912_TF_Result_Mid[u1TfType][u1FreqCount]; 
		     u4Freq1 = R912_TF_Freq_Mid[u1TfType][u1FreqCount-1];
		     u4Freq2 = R912_TF_Freq_Mid[u1TfType][u1FreqCount]; 
			 u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
             R912_TF = u1TF_Set_Result1 + (u8)((u1TF_Set_Result2 - u1TF_Set_Result1)*u4Ratio/100);
		 }
	}
	else  //HIGH
	{
		 //u1TFCalNum = R912_TF_HIGH_NUM;
         while((u4FreqKHz < R912_TF_Freq_High[u1TfType][u1FreqCount]) && (u1FreqCount<R912_TF_HIGH_NUM))
		 {
            u1FreqCount++;
		 }

		 if(u1FreqCount==0)
		 {
			 R912_TF = R912_TF_Result_High[u1TfType][0];
		 }
		 else if(u1FreqCount==R912_TF_HIGH_NUM)
        {
			 R912_TF = R912_TF_Result_High[u1TfType][R912_TF_HIGH_NUM-1];
		 }
		 else
		 {
			 u1TF_Set_Result1 = R912_TF_Result_High[u1TfType][u1FreqCount-1]; 
		     u1TF_Set_Result2 = R912_TF_Result_High[u1TfType][u1FreqCount]; 
		     u4Freq1 = R912_TF_Freq_High[u1TfType][u1FreqCount-1];
		     u4Freq2 = R912_TF_Freq_High[u1TfType][u1FreqCount]; 
			 u4Ratio = (u4Freq1- u4FreqKHz)*100/(u4Freq1 - u4Freq2);
             R912_TF = u1TF_Set_Result1 + (u8)((u1TF_Set_Result2 - u1TF_Set_Result1)*u4Ratio/100);
		 }
	}
  
	// R8[6:0] 
	R912_I2C.RegAddr = 0x08;
	R912_Array[0] = (R912_Array[0] & 0x80) | R912_TF;
	R912_I2C.Data = R912_Array[0]  ;
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

    return RT_Success;
}


R912_ErrCode R912_IQ(struct r912_priv *priv, R912_SectType* IQ_Pont)
{
	R912_SectType Compare_IQ[3];
	u8   VGA_Count = 0;
	u8   VGA_Read = 0;
	u8   X_Direction;  // 1:X, 0:Y
		
	// increase VGA power to let image significant
	for(VGA_Count=11; VGA_Count < 16; VGA_Count ++)
	{
		R912_I2C.RegAddr = 0x14; // R912:R20[3:0]  
		R912_I2C.Data    = (R912_Array[12] & 0xF0) + VGA_Count;  
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_Delay_MS(VGA_DELAY); //
		
		if(R912_Muti_Read(priv,&VGA_Read) != RT_Success)
			return RT_Fail;

		if(VGA_Read > 40*ADC_READ_COUNT)
			break;
	}

	Compare_IQ[0].Gain_X  = R912_Array[8] & 0xC0; // R16[5:0]  
	Compare_IQ[0].Phase_Y = R912_Array[9] & 0xC0; // R17[5:0] 
	//Compare_IQ[0].Iqcap = R912_iniArray[3] & 0xFC;

	    // Determine X or Y
	    if(R912_IMR_Cross(priv,&Compare_IQ[0], &X_Direction) != RT_Success)
			return RT_Fail;

		if(X_Direction==1)
		{
			//compare and find min of 3 points. determine I/Q direction
		    if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
			  return RT_Fail;

		    //increase step to find min value of this direction
		    if(R912_CompreStep(priv,&Compare_IQ[0], 0x10) != RT_Success)  //X
			  return RT_Fail;	
		}
		else
		{
		   //compare and find min of 3 points. determine I/Q direction
		   if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		   	 return RT_Fail;

		   //increase step to find min value of this direction
		   if(R912_CompreStep(priv,&Compare_IQ[0], 0x11) != RT_Success)  //Y
			 return RT_Fail;	
		}

		//Another direction
		if(X_Direction==1)
		{	    
           if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success) //Y	
		     return RT_Fail;	

		   //compare and find min of 3 points. determine I/Q direction
		   if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		   	 return RT_Fail;

		   //increase step to find min value of this direction
		   if(R912_CompreStep(priv,&Compare_IQ[0], 0x11) != RT_Success)  //Y
			 return RT_Fail;	
		}
		else
		{
		   if(R912_IQ_Tree(priv,Compare_IQ[0].Phase_Y, Compare_IQ[0].Gain_X, 0x11, &Compare_IQ[0]) != RT_Success) //X
		     return RT_Fail;	
        
		   //compare and find min of 3 points. determine I/Q direction
		   if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		     return RT_Fail;

	       //increase step to find min value of this direction
		   if(R912_CompreStep(priv,&Compare_IQ[0], 0x10) != RT_Success) //X
		     return RT_Fail;	
		}
		

		//--- Check 3 points again---//
		if(X_Direction==1)
		{
		    if(R912_IQ_Tree(priv,Compare_IQ[0].Phase_Y, Compare_IQ[0].Gain_X, 0x11, &Compare_IQ[0]) != RT_Success) //X
			  return RT_Fail;	
		}
		else
		{
		   if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success) //Y
			return RT_Fail;		
		}

		if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
			return RT_Fail;

    //Section-9 check
    //if(R912_F_IMR(&Compare_IQ[0]) != RT_Success)
	if(R912_Section(priv,&Compare_IQ[0]) != RT_Success)
			return RT_Fail;

	//clear IQ_Cap = 0   //  R11[1:0]  
	Compare_IQ[0].Iqcap = R912_Array[3] & 0xFC;

	if(R912_IMR_Iqcap(priv,&Compare_IQ[0]) != RT_Success)
			return RT_Fail;

	*IQ_Pont = Compare_IQ[0];

	//reset gain/phase/iqcap control setting
	R912_I2C.RegAddr = 0x10;	// R16[5:0]  
	R912_Array[8] = R912_Array[8] & 0xC0;
	R912_I2C.Data = R912_Array[8];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	R912_I2C.RegAddr = 0x11;	// R17[5:0]  
	R912_Array[9] = R912_Array[9] & 0xC0;
	R912_I2C.Data = R912_Array[9];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	R912_I2C.RegAddr = 0x0B;	//  R11[1:0] 
	R912_Array[3] = R912_Array[3] & 0xFC;
	R912_I2C.Data = R912_Array[3];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	return RT_Success;
}



//--------------------------------------------------------------------------------------------
// Purpose: record IMC results by input gain/phase location
//          then adjust gain or phase positive 1 step and negtive 1 step, both record results
// input: FixPot: phase or gain
//        FlucPot phase or gain
//        PotReg: 0x10 or 0x11 for R912
//        CompareTree: 3 IMR trace and results
// output: TREU or FALSE
//--------------------------------------------------------------------------------------------

//20131217-Ryan


R912_ErrCode R912_IQ_Tree(struct r912_priv *priv, u8 FixPot, u8 FlucPot, u8 PotReg, R912_SectType* CompareTree)
{
	u8 TreeCunt  = 0;
	u8 TreeTimes = 3;
	u8 PntReg    = 0;

	if(PotReg == 0x10)
		PntReg = 0x11; //phase control
	else
		PntReg = 0x10; //gain control

	for(TreeCunt = 0;TreeCunt < TreeTimes;TreeCunt ++)
	{
		R912_I2C.RegAddr = PotReg;
		R912_I2C.Data    = FixPot;
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_I2C.RegAddr = PntReg;
		R912_I2C.Data    = FlucPot;
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		if(R912_Muti_Read(priv,&CompareTree[TreeCunt].Value) != RT_Success)
			return RT_Fail;
	

		if(PotReg == 0x10)
		{
			CompareTree[TreeCunt].Gain_X  = FixPot;
			CompareTree[TreeCunt].Phase_Y = FlucPot;
		}
		else
		{
			CompareTree[TreeCunt].Phase_Y  = FixPot;
			CompareTree[TreeCunt].Gain_X = FlucPot;
		}
		
		if(TreeCunt == 0)   //try right-side point
			FlucPot ++; 
		else if(TreeCunt == 1) //try left-side point
		{
			if((FlucPot & 0x1F) == 1) //if absolute location is 1, change I/Q direction
			{
				if(FlucPot & 0x20) //b[5]:I/Q selection. 0:Q-path, 1:I-path
				{
					FlucPot = (FlucPot & 0xC0) | 0x01;			
				}
				else
				{
					FlucPot = (FlucPot & 0xC0) | 0x21;
				}
			}
			else
				FlucPot = FlucPot - 2;  
				
		}
	}

	return RT_Success;
}




//-----------------------------------------------------------------------------------/ 
// Purpose: compare IMC result aray [0][1][2], find min value and store to CorArry[0]
// input: CorArry: three IMR data array
// output: TRUE or FALSE
//-----------------------------------------------------------------------------------/
R912_ErrCode R912_CompreCor(struct r912_priv *priv, R912_SectType* CorArry)
{
	u8 CompCunt = 0;
	R912_SectType CorTemp;

	for(CompCunt = 3;CompCunt > 0;CompCunt --)
	{
		if(CorArry[0].Value > CorArry[CompCunt - 1].Value) //compare IMC result [0][1][2], find min value
		{
			CorTemp = CorArry[0];
			CorArry[0] = CorArry[CompCunt - 1];
			CorArry[CompCunt - 1] = CorTemp;
		}
	}

	return RT_Success;
}


//-------------------------------------------------------------------------------------//
// Purpose: if (Gain<9 or Phase<9), Gain+1 or Phase+1 and compare with min value
//          new < min => update to min and continue
//          new > min => Exit
// input: StepArry: three IMR data array
//        Pace: gain or phase register
// output: TRUE or FALSE 
//-------------------------------------------------------------------------------------//
R912_ErrCode R912_CompreStep(struct r912_priv *priv, R912_SectType* StepArry, u8 Pace)
{
	R912_SectType StepTemp;
	
	//min value already saved in StepArry[0]
	StepTemp.Phase_Y = StepArry[0].Phase_Y;
	StepTemp.Gain_X  = StepArry[0].Gain_X;
	//StepTemp.Iqcap  = StepArry[0].Iqcap;

	while(((StepTemp.Gain_X & 0x1F) < R912_IMR_TRIAL) && ((StepTemp.Phase_Y & 0x1F) < R912_IMR_TRIAL))  
	{
		if(Pace == 0x10)	
			StepTemp.Gain_X ++;
		else
			StepTemp.Phase_Y ++;
	
		R912_I2C.RegAddr = 0x10;	
		R912_I2C.Data    = StepTemp.Gain_X ;
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_I2C.RegAddr = 0x11;	
		R912_I2C.Data    = StepTemp.Phase_Y;
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		if(R912_Muti_Read(priv,&StepTemp.Value) != RT_Success)
			return RT_Fail;

		if(StepTemp.Value <= StepArry[0].Value)
		{
			StepArry[0].Gain_X  = StepTemp.Gain_X;
			StepArry[0].Phase_Y = StepTemp.Phase_Y;
			//StepArry[0].Iqcap = StepTemp.Iqcap;
			StepArry[0].Value   = StepTemp.Value;
		}
		else if((StepTemp.Value - 2*ADC_READ_COUNT) > StepArry[0].Value)
		{
			break;		
		}
		
	} //end of while()
	
	return RT_Success;
}


//-----------------------------------------------------------------------------------/ 
// Purpose: read multiple IMC results for stability
// input: IMR_Reg: IMC result address
//        IMR_Result_Data: result 
// output: TRUE or FALSE
//-----------------------------------------------------------------------------------/
R912_ErrCode R912_Muti_Read(struct r912_priv *priv, u8* IMR_Result_Data)
{
#if (ADC_MULTI_READ==3)
	u8 ReadCunt     = 0;
	u16 ReadAmount  = 0;
	u8 ReadMax = 0;
	u8 ReadMin = 255;
	u8 ReadData = 0;
	
    R912_Delay_MS(ADC_READ_DELAY);//3

	for(ReadCunt = 0; ReadCunt < (ADC_READ_COUNT+2); ReadCunt ++)
	{
		R912_I2C_Len.RegAddr = 0x00;
		R912_I2C_Len.Len = 2;              // read 0x01
		if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
		{
			I2C_Read_Len(priv,&R912_I2C_Len);
		}

		ReadData = (R912_I2C_Len.Data[1] & 0x3F);
		
		ReadAmount = ReadAmount + (u16)ReadData;
		
		if(ReadData < ReadMin)
			ReadMin = ReadData;
		
        if(ReadData > ReadMax)
			ReadMax = ReadData;
	}
	*IMR_Result_Data = (u8) (ReadAmount - (u16)ReadMax - (u16)ReadMin);
#else
	R912_Delay_MS(ADC_READ_DELAY);//2

	R912_I2C_Len.RegAddr = 0x00;
	R912_I2C_Len.Len = 2;              // read 0x01
	if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
	{
		I2C_Read_Len(priv,&R912_I2C_Len);
	}
	*IMR_Result_Data = (R912_I2C_Len.Data[1] & 0x3F);
#endif

	return RT_Success;
}


R912_ErrCode R912_Section(struct r912_priv *priv, R912_SectType* IQ_Pont)
{
	R912_SectType Compare_IQ[3];
	R912_SectType Compare_Bet[3];

	//Try X-1 column and save min result to Compare_Bet[0]
	if((IQ_Pont->Gain_X & 0x1F) == 0x00)
	{
		Compare_IQ[0].Gain_X = ((IQ_Pont->Gain_X) & 0xDF) + 1;  //Q-path, Gain=1
	}
	else
	{
		Compare_IQ[0].Gain_X  = IQ_Pont->Gain_X - 1;  //left point
	}
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)  // y-direction
		return RT_Fail;		

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[0].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[0].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[0].Value = Compare_IQ[0].Value;

	//Try X column and save min result to Compare_Bet[1]
	Compare_IQ[0].Gain_X = IQ_Pont->Gain_X;
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)
		return RT_Fail;	

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[1].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[1].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[1].Value = Compare_IQ[0].Value;

	//Try X+1 column and save min result to Compare_Bet[2]
	if((IQ_Pont->Gain_X & 0x1F) == 0x00)		
		Compare_IQ[0].Gain_X = ((IQ_Pont->Gain_X) | 0x20) + 1;  //I-path, Gain=1
	else
	    Compare_IQ[0].Gain_X = IQ_Pont->Gain_X + 1;
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)
		return RT_Fail;		

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[2].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[2].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[2].Value = Compare_IQ[0].Value;

	if(R912_CompreCor(priv,&Compare_Bet[0]) != RT_Success)
		return RT_Fail;

	*IQ_Pont = Compare_Bet[0];
	
	return RT_Success;
}


R912_ErrCode R912_IMR_Cross(struct r912_priv *priv, R912_SectType* IQ_Pont, u8* X_Direct)
{

	R912_SectType Compare_Cross[9]; //(0,0)(0,Q-1)(0,I-1)(Q-1,0)(I-1,0)+(0,Q-2)(0,I-2)(Q-2,0)(I-2,0)
	R912_SectType Compare_Temp;
	u8 CrossCount = 0;
	u8 Reg16 = R912_Array[8] & 0xC0;	
	u8 Reg17 = R912_Array[9] & 0xC0;	

	memset(&Compare_Temp,0, sizeof(R912_SectType));
	Compare_Temp.Value = 255;

	for(CrossCount=0; CrossCount<9; CrossCount++)
	{

		if(CrossCount==0)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16;
		  Compare_Cross[CrossCount].Phase_Y = Reg17;
		}
		else if(CrossCount==1)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16;       //0
		  Compare_Cross[CrossCount].Phase_Y = Reg17 + 1;  //Q-1
		}
		else if(CrossCount==2)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16;               //0
		  Compare_Cross[CrossCount].Phase_Y = (Reg17 | 0x20) + 1; //I-1
		}
		else if(CrossCount==3)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16 + 1; //Q-1
		  Compare_Cross[CrossCount].Phase_Y = Reg17;
		}
		else if(CrossCount==4)
		{
		  Compare_Cross[CrossCount].Gain_X = (Reg16 | 0x20) + 1; //I-1
		  Compare_Cross[CrossCount].Phase_Y = Reg17;
		}
		else if(CrossCount==5)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16;       //0
		  Compare_Cross[CrossCount].Phase_Y = Reg17 + 2;  //Q-2
		}
		else if(CrossCount==6)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16;               //0
		  Compare_Cross[CrossCount].Phase_Y = (Reg17 | 0x20) + 2; //I-2
		}
		else if(CrossCount==7)
		{
		  Compare_Cross[CrossCount].Gain_X = Reg16 + 2; //Q-2
		  Compare_Cross[CrossCount].Phase_Y = Reg17;
		}
		else if(CrossCount==8)
		{
		  Compare_Cross[CrossCount].Gain_X = (Reg16 | 0x20) + 2; //I-2
		  Compare_Cross[CrossCount].Phase_Y = Reg17;
		}

    	R912_I2C.RegAddr = 0x10;	
	    R912_I2C.Data    = Compare_Cross[CrossCount].Gain_X;
	    if(I2C_Write(priv, &R912_I2C) != RT_Success)
		   return RT_Fail;

	    R912_I2C.RegAddr = 0x11;	
	    R912_I2C.Data    = Compare_Cross[CrossCount].Phase_Y;
	    if(I2C_Write(priv, &R912_I2C) != RT_Success)
		  return RT_Fail;
	
        if(R912_Muti_Read(priv,&Compare_Cross[CrossCount].Value) != RT_Success)
		  return RT_Fail;

		if( Compare_Cross[CrossCount].Value < Compare_Temp.Value)
		{
		  Compare_Temp.Value = Compare_Cross[CrossCount].Value;
		  Compare_Temp.Gain_X = Compare_Cross[CrossCount].Gain_X;
		  Compare_Temp.Phase_Y = Compare_Cross[CrossCount].Phase_Y;		
		}
	} //end for loop


    if(((Compare_Temp.Phase_Y & 0x3F)==0x01) || (Compare_Temp.Phase_Y & 0x3F)==0x02)  //phase Q1 or Q2
	{
      *X_Direct = (u8) 0;
	  IQ_Pont[0].Gain_X = Compare_Cross[0].Gain_X;    //0
	  IQ_Pont[0].Phase_Y = Compare_Cross[0].Phase_Y; //0
	  IQ_Pont[0].Value = Compare_Cross[0].Value;

	  IQ_Pont[1].Gain_X = Compare_Cross[1].Gain_X;    //0
	  IQ_Pont[1].Phase_Y = Compare_Cross[1].Phase_Y; //Q1
	  IQ_Pont[1].Value = Compare_Cross[1].Value;

	  IQ_Pont[2].Gain_X = Compare_Cross[5].Gain_X;   //0
	  IQ_Pont[2].Phase_Y = Compare_Cross[5].Phase_Y;//Q2
	  IQ_Pont[2].Value = Compare_Cross[5].Value;
	}
	else if(((Compare_Temp.Phase_Y & 0x3F)==0x21) || (Compare_Temp.Phase_Y & 0x3F)==0x22)  //phase I1 or I2
	{
      *X_Direct = (u8) 0;
	  IQ_Pont[0].Gain_X = Compare_Cross[0].Gain_X;    //0
	  IQ_Pont[0].Phase_Y = Compare_Cross[0].Phase_Y; //0
	  IQ_Pont[0].Value = Compare_Cross[0].Value;

	  IQ_Pont[1].Gain_X = Compare_Cross[2].Gain_X;    //0
	  IQ_Pont[1].Phase_Y = Compare_Cross[2].Phase_Y; //Q1
	  IQ_Pont[1].Value = Compare_Cross[2].Value;

	  IQ_Pont[2].Gain_X = Compare_Cross[6].Gain_X;   //0
	  IQ_Pont[2].Phase_Y = Compare_Cross[6].Phase_Y;//Q2
	  IQ_Pont[2].Value = Compare_Cross[6].Value;
	}
	else if(((Compare_Temp.Gain_X & 0x3F)==0x01) || (Compare_Temp.Gain_X & 0x3F)==0x02)  //gain Q1 or Q2
	{
      *X_Direct = (u8) 1;
	  IQ_Pont[0].Gain_X = Compare_Cross[0].Gain_X;    //0
	  IQ_Pont[0].Phase_Y = Compare_Cross[0].Phase_Y; //0
	  IQ_Pont[0].Value = Compare_Cross[0].Value;

	  IQ_Pont[1].Gain_X = Compare_Cross[3].Gain_X;    //Q1
	  IQ_Pont[1].Phase_Y = Compare_Cross[3].Phase_Y; //0
	  IQ_Pont[1].Value = Compare_Cross[3].Value;

	  IQ_Pont[2].Gain_X = Compare_Cross[7].Gain_X;   //Q2
	  IQ_Pont[2].Phase_Y = Compare_Cross[7].Phase_Y;//0
	  IQ_Pont[2].Value = Compare_Cross[7].Value;
	}
	else if(((Compare_Temp.Gain_X & 0x3F)==0x21) || (Compare_Temp.Gain_X & 0x3F)==0x22)  //gain I1 or I2
	{
      *X_Direct = (u8) 1;
	  IQ_Pont[0].Gain_X = Compare_Cross[0].Gain_X;    //0
	  IQ_Pont[0].Phase_Y = Compare_Cross[0].Phase_Y; //0
	  IQ_Pont[0].Value = Compare_Cross[0].Value;

	  IQ_Pont[1].Gain_X = Compare_Cross[4].Gain_X;    //I1
	  IQ_Pont[1].Phase_Y = Compare_Cross[4].Phase_Y; //0
	  IQ_Pont[1].Value = Compare_Cross[4].Value;

	  IQ_Pont[2].Gain_X = Compare_Cross[8].Gain_X;   //I2
	  IQ_Pont[2].Phase_Y = Compare_Cross[8].Phase_Y;//0
	  IQ_Pont[2].Value = Compare_Cross[8].Value;
	}
	else //(0,0) 
	{	
	  *X_Direct = (u8) 1;
	  IQ_Pont[0].Gain_X = Compare_Cross[0].Gain_X;
	  IQ_Pont[0].Phase_Y = Compare_Cross[0].Phase_Y;
	  IQ_Pont[0].Value = Compare_Cross[0].Value;

	  IQ_Pont[1].Gain_X = Compare_Cross[3].Gain_X;    //Q1
	  IQ_Pont[1].Phase_Y = Compare_Cross[3].Phase_Y; //0
	  IQ_Pont[1].Value = Compare_Cross[3].Value;

	  IQ_Pont[2].Gain_X = Compare_Cross[4].Gain_X;   //I1
	  IQ_Pont[2].Phase_Y = Compare_Cross[4].Phase_Y; //0
	  IQ_Pont[2].Value = Compare_Cross[4].Value;
	}
	return RT_Success;
}


//----------------------------------------------------------------------------------------//
// purpose: search surrounding points from previous point 
//          try (x-1), (x), (x+1) columns, and find min IMR result point
// input: IQ_Pont: previous point data(IMR Gain, Phase, ADC Result, RefRreq)
//                 will be updated to final best point                 
// output: TRUE or FALSE
//----------------------------------------------------------------------------------------//
R912_ErrCode R912_F_IMR(struct r912_priv *priv, R912_SectType* IQ_Pont)
{
	R912_SectType Compare_IQ[3];
	R912_SectType Compare_Bet[3];
	u8 VGA_Count = 0;
	u8 VGA_Read = 0;

	//VGA
	for(VGA_Count=11; VGA_Count < 16; VGA_Count ++)
	{
		R912_I2C.RegAddr = 0x14;	//  R20[3:0]  
        R912_I2C.Data    = (R912_Array[12] & 0xF0) + VGA_Count;
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_Delay_MS(10);
		
		if(R912_Muti_Read(priv,&VGA_Read) != RT_Success)
			return RT_Fail;

		if(VGA_Read > 40*ADC_READ_COUNT)
		break;
	}

	//Try X-1 column and save min result to Compare_Bet[0]
	if((IQ_Pont->Gain_X & 0x1F) == 0x00)
	{
		Compare_IQ[0].Gain_X = ((IQ_Pont->Gain_X) & 0xDF) + 1;  //Q-path, Gain=1
	}
	else
	{
		Compare_IQ[0].Gain_X  = IQ_Pont->Gain_X - 1;  //left point
	}
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)  // y-direction
		return RT_Fail;	

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[0].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[0].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[0].Value = Compare_IQ[0].Value;

	//Try X column and save min result to Compare_Bet[1]
	Compare_IQ[0].Gain_X = IQ_Pont->Gain_X;
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)
		return RT_Fail;	

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[1].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[1].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[1].Value = Compare_IQ[0].Value;

	//Try X+1 column and save min result to Compare_Bet[2]
	if((IQ_Pont->Gain_X & 0x1F) == 0x00)		
		Compare_IQ[0].Gain_X = ((IQ_Pont->Gain_X) | 0x20) + 1;  //I-path, Gain=1
	else
	    Compare_IQ[0].Gain_X = IQ_Pont->Gain_X + 1;
	Compare_IQ[0].Phase_Y = IQ_Pont->Phase_Y;

	if(R912_IQ_Tree(priv,Compare_IQ[0].Gain_X, Compare_IQ[0].Phase_Y, 0x10, &Compare_IQ[0]) != RT_Success)
		return RT_Fail;		

	if(R912_CompreCor(priv,&Compare_IQ[0]) != RT_Success)
		return RT_Fail;

	Compare_Bet[2].Gain_X = Compare_IQ[0].Gain_X;
	Compare_Bet[2].Phase_Y = Compare_IQ[0].Phase_Y;
	Compare_Bet[2].Value = Compare_IQ[0].Value;

	if(R912_CompreCor(priv,&Compare_Bet[0]) != RT_Success)
		return RT_Fail;

	//clear IQ_Cap = 0
	Compare_Bet[0].Iqcap = R912_Array[3] & 0xFC;	//  R11[1:0] 

	if(R912_IMR_Iqcap(priv,&Compare_Bet[0]) != RT_Success)
		return RT_Fail;

	*IQ_Pont = Compare_Bet[0];
	
	return RT_Success;
}


R912_ErrCode R912_IMR_Iqcap(struct r912_priv *priv, R912_SectType* IQ_Point)   
{
    R912_SectType Compare_Temp;
	int i = 0;

	//Set Gain/Phase to right setting
	R912_I2C.RegAddr = 0x10;	// R16[5:0]  
	R912_I2C.Data = IQ_Point->Gain_X; 
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	R912_I2C.RegAddr = 0x11;	// R17[5:0]  
	R912_I2C.Data = IQ_Point->Phase_Y;
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	//try iqcap
	for(i=0; i<3; i++)	
	{
		Compare_Temp.Iqcap = (u8) i;  
		R912_I2C.RegAddr = 0x0B;		// R11[1:0] 
		R912_Array[3] = (R912_Array[3] & 0xFC) | Compare_Temp.Iqcap;  
		R912_I2C.Data = R912_Array[3];  
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			   return RT_Fail;

		if(R912_Muti_Read(priv,&(Compare_Temp.Value)) != RT_Success)
			   return RT_Fail;

		if(Compare_Temp.Value < IQ_Point->Value)
		{
			IQ_Point->Value = Compare_Temp.Value; 
			IQ_Point->Iqcap = Compare_Temp.Iqcap;
		}
	}

    return RT_Success;
}


R912_ErrCode R912_GPO(struct r912_priv *priv, R912_GPO_Type R912_GPO_Conrl)
{
	if(R912_GPO_Conrl == HI_SIG)	//  R23[0]  
		R912_Array[15] |= 0x01;   //high
	else
		R912_Array[15] &= 0xFE;  //low
	R912_I2C.RegAddr = 0x17;
	R912_I2C.Data = R912_Array[15];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	return RT_Success;
}


R912_ErrCode R912_SetStandard(struct r912_priv *priv, R912_Standard_Type RT_Standard)
{	 
	u8 u1FilCalGap = 8;
	//printf("RT_Standard = %d\n",RT_Standard);
	if(RT_Standard != R912_pre_standard)
	{
		 if(R912_InitReg(priv,RT_Standard) != RT_Success)      
		     return RT_Fail;
	}
    R912_pre_standard = RT_Standard;


	Sys_Info1 = R912_Sys_Sel(priv,RT_Standard);

	// Filter Calibration

	if(RT_Standard<R912_ATV_SIZE)    //ATV
	    u1FilCalGap = R912_Fil_Cal_Gap;
	else
	    u1FilCalGap = 8;

    if(R912_Fil_Cal_flag[RT_Standard] == R912_FALSE)
	{
		R912_Fil_Cal_code[RT_Standard] = R912_Filt_Cal_ADC(priv,Sys_Info1.FILT_CAL_IF, Sys_Info1.BW, u1FilCalGap);
		R912_Fil_Cal_BW[RT_Standard] = R912_Bandwidth;
        R912_Fil_Cal_flag[RT_Standard] = R912_TRUE;

	    //Reset register and Array 
	    if(R912_InitReg(priv,RT_Standard) != RT_Success)        
		   return RT_Fail;
	}

	// Set Filter Auto Ext
	R912_I2C.RegAddr = 0x13;	// R19[4]  
	R912_Array[11] = (R912_Array[11] & 0xEF) | Sys_Info1.FILT_EXT_ENA;  
	R912_I2C.Data = R912_Array[11];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;


	if(Sys_Info1.FILT_EXT_ENA==0x10)  //(%)
	{
		
			 if(R912_Fil_Cal_code[RT_Standard]< 2)  
			   R912_Fil_Cal_code[RT_Standard] = 2;
		
			 if((Sys_Info1.FILT_EXT_POINT & 0x02)==0x02)  //HPF+3
			 {
				  if(Sys_Info1.HPF_COR>12)  
				  {    Sys_Info1.HPF_COR = 12; }
			 }
			 else  //HPF+1
			 {
				  if(Sys_Info1.HPF_COR>14)  
				  {    Sys_Info1.HPF_COR = 15; }		 
			 }		  			
	}


	// Set LPF fine code
	R912_I2C.RegAddr = 0x12;	//  R912:R18[3:0]  
	R912_Array[10] = (R912_Array[10] & 0xF0) | R912_Fil_Cal_code[RT_Standard];  
	R912_I2C.Data = R912_Array[10];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set LPF coarse BW
	R912_I2C.RegAddr = 0x13;	// R912:R19[6:5]  
	R912_Array[11] = (R912_Array[11] & 0x9F) | R912_Fil_Cal_BW[RT_Standard];
	R912_I2C.Data = R912_Array[11];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
	
	// Set HPF corner & 1.7M mode
	R912_I2C.RegAddr = 0x13;	//R912:R19[7 & 3:0]  
	R912_Array[11] = (R912_Array[11] & 0x70) | Sys_Info1.HPF_COR | Sys_Info1.V17M;
	R912_I2C.Data = R912_Array[11];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set TF current 
	R912_I2C.RegAddr = 0x0B;	//  R912:R11[6]  
	R912_Array[3] = (R912_Array[3] & 0xBF) | Sys_Info1.TF_CUR;  
	R912_I2C.Data = R912_Array[3];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set Filter current 
	R912_I2C.RegAddr = 0x12;	//  R912:R18[6:5]  
	R912_Array[10] = (R912_Array[10] & 0x9F) | Sys_Info1.FILT_CUR;  
	//R912_Array[10] = (R912_Array[10] & 0x9F) | 0x60;   //lowest
	R912_I2C.Data = R912_Array[10];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set Switch Buffer current 
	R912_I2C.RegAddr = 0x0C;	//  R912:R12[2] 
	R912_Array[4] = (R912_Array[4] & 0xFB) | Sys_Info1.SWBUF_CUR;   
	R912_I2C.Data = R912_Array[4];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set Filter Comp 
	R912_I2C.RegAddr = 0x26;	//  R912:R38[6:5]  
	R912_Array[30] = (R912_Array[30] & 0x9F) | Sys_Info1.FILT_COMP;  
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

   // Set Filter 3dB
	R912_I2C.RegAddr = 0x26;	// R912:R38[7]  
	R912_Array[30] = (R912_Array[30] & 0xF7) | Sys_Info1.FILT_3DB;  
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set Filter Ext Condition (%)
	R912_I2C.RegAddr = 0x26;	//  R912:R38[2:0] 
    R912_Array[30] = (R912_Array[30] & 0xF8) | 0x04 | Sys_Info1.FILT_EXT_POINT;   //ext both HPF/LPF
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
/*
	// Set Inductor Bias
	R912_I2C.RegAddr = 0x04;
	R912_Array[4] = (R912_Array[4] & 0xFE) | Sys_Info1.INDUC_BIAS; 
	R912_I2C.Data = R912_Array[4];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
*/
	// Set sw cap clk
	R912_I2C.RegAddr = 0x1A;	//  R912:R26[1:0]  
	R912_Array[18] = (R912_Array[18] & 0xFC) | Sys_Info1.SWCAP_CLK; 
	R912_I2C.Data = R912_Array[18];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	// Set NA det power
	R912_I2C.RegAddr = 0x26;	// R912:R38[7] 
	R912_Array[30] = (R912_Array[30] & 0x7F) | Sys_Info1.NA_PWR_DET; 
	R912_I2C.Data = R912_Array[30];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

/*
	// Set AGC clk 
	R912_I2C.RegAddr = 0x1A;	//  R912:R26[4:2] 
	R912_Array[18] = (R912_Array[18] & 0xE3) | Sys_Info1.AGC_CLK;  
	R912_I2C.Data = R912_Array[18];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
*/

	//Set GPO High
	R912_I2C.RegAddr = 0x17;	// R912:R23[4:2]  
	R912_Array[15] = (R912_Array[15] & 0xFE) | 0x01;
	R912_I2C.Data = R912_Array[15];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;


	return RT_Success;
}


u8  R912_Filt_Cal_ADC(struct r912_priv *priv, u32 IF_Freq, u8 R912_BW, u8 FilCal_Gap)
{
	 u8     u1FilterCodeResult = 0;
	 u8     u1FilterCode = 0;
	 u32   u4RingFreq = 72000;
	 u8     u1FilterCalValue = 0;
	 u8     u1FilterCalValuePre = 0;
	 u8     initial_cnt = 0;
	 u8     i = 0;
	 u32   RingVCO = 0;
	 u32   RingRef = R912_Xtal;
	 u8     divnum_ring = 0;
	 R912_Standard_Type	R912_Standard;
	 u8   VGA_Count = 0;
	 u8   VGA_Read = 0;

	
	if(R912_Xtal==16000)  //16M
	{
         divnum_ring = 11;
	}
	else if (R912_Xtal==24000)  //24M
	{
		 divnum_ring = 2;
	}
	else
	{
		divnum_ring = 0;
	}
	 RingVCO = (16+divnum_ring)* 8 * RingRef;
	 u4RingFreq = RingVCO/48;



	 R912_Standard=R912_ATSC; //no set R912_DVB_S

	 //Write initial reg before doing calibration 
	 if(R912_InitReg(priv,R912_Standard) != RT_Success)        
		return RT_Fail;

	 if(R912_Cal_Prepare(priv,R912_LPF_CAL) != RT_Success)      
	      return RT_Fail;


	 //Set Ring PLL(72MHz)	 	
	 /*R912_I2C.RegAddr = 0x18;	//  R912:R22[7:2]  
	 R912_Array[24] = (R912_Array[24] & 0x00) | 0x8B;  //div=11, pre div=6
	 R912_I2C.Data = R912_Array[24];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;*/

	 R912_I2C.RegAddr = 0x27;	//  R912:R39[5:2]  //div
	 R912_Array[31] = (R912_Array[31] & 0xC3) | (divnum_ring<<2);  
	 R912_I2C.Data = R912_Array[31];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	 R912_I2C.RegAddr = 0x12;	//R912:R18[4]  
	 R912_Array[10] = (R912_Array[10] & 0xEF) | 0x00; 
	 R912_I2C.Data = R912_Array[10];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	 R912_I2C.RegAddr = 0x25;	//  R912:R37[7]  
	 R912_Array[29] = (R912_Array[29] & 0x7F) | 0x00; 
	 R912_I2C.Data = R912_Array[29];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	
	 R912_I2C.RegAddr = 0x27;	//  R912:R39[7:6]  
	 R912_Array[31] = (R912_Array[31] & 0x3F) | 0x80;  
	 R912_I2C.Data = R912_Array[31];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	 R912_Array[25] = (R912_Array[25] & 0x00) | 0x8B;   //out div=8, RF poly=low band, power=min_lp
	 if(RingVCO>=3200000)
	{
		R912_Array[25] &= 0xDF;   //clear vco_band, R25[5]		//R912:R33[5]    33-8=25  33(0x21) is addr ; [25] is data 
	}
	else
	{
		R912_Array[25] |= 0x20;   //clear vco_band, R25[5]		//R912:R33[5]    33-8=25  33(0x21) is addr ; [25] is data 
	}
	 R912_I2C.RegAddr = 0x21;	//  R912:R33[7:0]  
	 R912_I2C.Data = R912_Array[25];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;	

     //Must do before PLL() 
	 if(R912_MUX(priv,u4RingFreq + IF_Freq, u4RingFreq, R912_STD_SIZE) != RT_Success)     //FilCal MUX (LO_Freq, RF_Freq)
	     return RT_Fail;

	 //Set PLL
	 if(R912_PLL(priv,(u4RingFreq + IF_Freq), R912_STD_SIZE) != RT_Success)   //FilCal PLL
	       return RT_Fail;

	 //-----below must set after R912_MUX()-------//
	 //Set LNA TF for RF=72MHz. no use
	 if((R912_SetTfType==R912_TF_NARROW) || (R912_SetTfType==R912_TF_NARROW_LIN))   //UL use 270n setting
	 {
	    R912_I2C.RegAddr = 0x08;	//  R912:R8[6:0]  
        R912_Array[0] = (R912_Array[0] & 0x80) | 0x1F;  
	 }
	 else
	 {
	    R912_I2C.RegAddr = 0x08;	//  R912:R8[6:0]  
        R912_Array[0] = (R912_Array[0] & 0x80) | 0x00;  
	 }
     R912_I2C.Data = R912_Array[0];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;

	 //Adc=on set 0;
	 R912_I2C.RegAddr = 0x0F;		//  R912:R15[7]  
     R912_Array[7] = (R912_Array[7] & 0x7F);
     R912_I2C.Data = R912_Array[7];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;



	//pwd_vga  vga power on set 0;
	 R912_I2C.RegAddr = 0x12;	//  R912:R18[7] 
     R912_Array[10] = (R912_Array[10] & 0x7F);  
     R912_I2C.Data = R912_Array[10];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;




	 //vga6db normal set 0;
	 R912_I2C.RegAddr = 0x0B;		// R912:R11[3]  
     R912_Array[3] = (R912_Array[3] & 0xF7);
     R912_I2C.Data = R912_Array[3];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

 	 //Vga Gain = -12dB 
	 R912_I2C.RegAddr = 0x14;		//  R912:R20[3:0]  
     R912_Array[12] = (R912_Array[12] & 0xF0);
     R912_I2C.Data = R912_Array[12];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
           return RT_Fail;

	
	 // vcomp = 0
	 R912_I2C.RegAddr = 0x26;	//  R912:R38[6:5]  
	 R912_Array[30] = (R912_Array[30] & 0x9F);	
	 R912_I2C.Data = R912_Array[30];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
	
	 //Set BW=8M, HPF corner narrowest; 1.7M disable
     R912_I2C.RegAddr = 0x13;	//  R912:R19[7:0]  
	 R912_Array[11] = (R912_Array[11] & 0x00);	  
     R912_I2C.Data = R912_Array[11];		
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;	

	 //------- increase VGA power to let ADC read value significant ---------//

	 R912_I2C.RegAddr = 0x12;	//  R912:R18[3:0]  
     R912_Array[10] = (R912_Array[10] & 0xF0) | 0;  //filter code=0
     R912_I2C.Data = R912_Array[10];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
          return RT_Fail;

	 for(VGA_Count=0; VGA_Count < 16; VGA_Count ++)
	 {
		R912_I2C.RegAddr = 0x14;	//  R912:R20[3:0]  
		R912_I2C.Data    = (R912_Array[12] & 0xF0) + VGA_Count;  
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

		R912_Delay_MS(VGA_DELAY); //
		
		if(R912_Muti_Read(priv,&VGA_Read) != RT_Success)
			return RT_Fail;

		if(VGA_Read > 40*ADC_READ_COUNT)
			break;
	 }

	 //------- Try suitable BW --------//

	 if(R912_BW==0x60) //6M
         initial_cnt = 1;  //try 7M first
	 else
		 initial_cnt = 0;  //try 8M first

	 for(i=initial_cnt; i<3; i++)
	 {
         if(i==0)
             R912_Bandwidth = 0x00; //8M
		 else if(i==1)
			 R912_Bandwidth = 0x40; //7M
		 else
			 R912_Bandwidth = 0x60; //6M

		 R912_I2C.RegAddr = 0x13;	//  R912:R19[7:0]  
	     R912_Array[11] = (R912_Array[11] & 0x00) | R912_Bandwidth;	  
         R912_I2C.Data = R912_Array[11];		
         if(I2C_Write(priv, &R912_I2C) != RT_Success)
		      return RT_Fail;	

		 // read code 0
		 R912_I2C.RegAddr = 0x12;	//  R912:R18[3:0]  
		 R912_Array[10] = (R912_Array[10] & 0xF0) | 0;  //code 0
		 R912_I2C.Data = R912_Array[10];
		 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			  return RT_Fail;

		 R912_Delay_MS(FILTER_DELAY); //delay ms
	     
		 if(R912_Muti_Read(priv,&u1FilterCalValuePre) != RT_Success)
			  return RT_Fail;

		 //read code 13
		 R912_I2C.RegAddr = 0x12;	// R912:R18[3:0]  
		 R912_Array[10] = (R912_Array[10] & 0xF0) | 13;  //code 13
		 R912_I2C.Data = R912_Array[10];
		 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			  return RT_Fail;

		 R912_Delay_MS(FILTER_DELAY); //delay ms
	     
		 if(R912_Muti_Read(priv,&u1FilterCalValue) != RT_Success)
			  return RT_Fail;

		 if(u1FilterCalValuePre > (u1FilterCalValue+8))  //suitable BW found
			 break;
	 }

     //-------- Try LPF filter code ---------//
	 u1FilterCalValuePre = 0;
	 for(u1FilterCode=0; u1FilterCode<16; u1FilterCode++)
	 {
         R912_I2C.RegAddr = 0x12;	//  R912:R18[3:0]  
         R912_Array[10] = (R912_Array[10] & 0xF0) | u1FilterCode;
         R912_I2C.Data = R912_Array[10];
         if(I2C_Write(priv, &R912_I2C) != RT_Success)
              return RT_Fail;

		 R912_Delay_MS(FILTER_DELAY); //delay ms

		 if(R912_Muti_Read(priv,&u1FilterCalValue) != RT_Success)
		      return RT_Fail;

		 if(u1FilterCode==0)
              u1FilterCalValuePre = u1FilterCalValue;

		 if((u1FilterCalValue+FilCal_Gap*ADC_READ_COUNT) < u1FilterCalValuePre)
		 {
			 u1FilterCodeResult = u1FilterCode;
			  break;
		 }

	 }

	 if (u1FilterCode == 16)
         	u1FilterCodeResult = 15;

	 return u1FilterCodeResult;

}
R912_ErrCode R912_SetFrequency(struct r912_priv *priv, R912_Set_Info R912_INFO)
{

	 u32	LO_KHz;
	 u8 RegArrayCunt = 0;

	 if(R912_INFO.R912_Standard!=R912_DVB_S)
	 {
		 // Check Input Frequency Range
		 if((R912_INFO.RF_KHz<40000) || (R912_INFO.RF_KHz>1002000))
		 {
				  return RT_Fail;
		 }
	 }
	 else
	 {
		// Check Input Frequency Range
		 if((R912_INFO.RF_KHz<40000) || (R912_INFO.RF_KHz>2400000))
		 {
				  return RT_Fail;
		 }
	 }

	      LO_KHz = R912_INFO.RF_KHz + Sys_Info1.IF_KHz;

	 //Set MUX dependent var. Must do before PLL( ) 
     if(R912_MUX(priv,LO_KHz, R912_INFO.RF_KHz, R912_INFO.R912_Standard) != RT_Success)   //normal MUX
        return RT_Fail;


     //Set PLL
     if(R912_PLL(priv,LO_KHz, R912_INFO.R912_Standard) != RT_Success) //noraml PLL
        return RT_Fail;

	 //Set TF
	 if(R912_SetTF(priv,R912_INFO.RF_KHz, R912_SetTfType) != RT_Success)
		return RT_Fail;



	 #if(FOR_KOREA_CTMR==R912_TRUE)  //Q_CTRL OFF
		 if((R912_INFO.RF_KHz>700000) && (R912_INFO.R912_Standard<R912_ATV_SIZE))
		 {
			R912_I2C.RegAddr = 0x08;
			R912_Array[0] = (R912_Array[0] & 0x80) | 0x00;
			R912_I2C.Data = R912_Array[0];
			if(I2C_Write(priv, &R912_I2C) != RT_Success)
				return RT_Fail;
		 }
	 #endif




     R912_IMR_point_num = Freq_Info1.IMR_MEM;

	 //Q1.5K   Q_ctrl  R912:R14[4]
	 //if(R912_INFO.RF_KHz<R912_LNA_MID_LOW[R912_TF_NARROW]) //<300MHz
     //     R912_Array[6] = R912_Array[6] | 0x10;	
	 //else
	 //	  R912_Array[6] = R912_Array[6] & 0xEF;		 

	if(R912_INFO.RF_KHz<=472000) //<472MHz
		R912_Array[6] = R912_Array[6] | 0x10;	
	else
	 	R912_Array[6] = R912_Array[6] & 0xEF;

	//medQctrl 1.5K
	if((R912_INFO.RF_KHz>=300000)&&(R912_INFO.RF_KHz<=472000)) //<473MHz and >299MHz
		R912_Array[6] = R912_Array[6] | 0x01;	
	else
	 	R912_Array[6] = R912_Array[6] & 0xFE;

	
	 //3~6 shrink
	if((R912_INFO.RF_KHz>=300000)&&(R912_INFO.RF_KHz<=550000)) //<551MHz and >299MHz
		R912_Array[3] = R912_Array[3] & 0xFB;	
	else
	 	R912_Array[3] = R912_Array[3] | 0x04;


	 //set Xtal AAC on=1 ;off=0
	 //R912_I2C.RegAddr = 0x18;	//  R912:R24[1]  
	 R912_Array[16] = R912_Array[16] & 0xFD;

     //Get Sys-Freq parameter	
     SysFreq_Info1 = R912_SysFreq_Sel(priv,R912_INFO.R912_Standard, R912_INFO.RF_KHz);

	
     // Set LNA_TOP
	 //R912_I2C.RegAddr = 0x23;	//  R912:R35[2:0]  
     R912_Array[27] = (R912_Array[27] & 0xF8) | (SysFreq_Info1.LNA_TOP);

	  // Set LNA VTHL
	 //R912_I2C.RegAddr = 0x1F;	// R912:R31[7:0]  
     R912_Array[23] = (R912_Array[23] & 0x00) | SysFreq_Info1.LNA_VTH_L;

     // Set MIXER TOP
	 //R912_I2C.RegAddr = 0x24;	// R912:R36[7:0]   
     R912_Array[28] = (R912_Array[28] & 0xF0) | (SysFreq_Info1.MIXER_TOP); 

     // Set MIXER VTHL
	 //R912_I2C.RegAddr = 0x20;	//  R912:R32[7:0]  
     R912_Array[24] = (R912_Array[24] & 0x00) | SysFreq_Info1.MIXER_VTH_L;

	 // Set RF TOP
	 //R912_I2C.RegAddr = 0x22;	// R912:R34[7:0]  
	 R912_Array[26] = (R912_Array[26] & 0x1F) | SysFreq_Info1.RF_TOP;

	 // Set Nrb TOP
	 //R912_I2C.RegAddr = 0x24;	//R912:R36[7:4]   
	 R912_Array[28] = (R912_Array[28] & 0x0F) | SysFreq_Info1.NRB_TOP;

	 // Set Nrb BW
	 //R912_I2C.RegAddr = 0x23;	//  R912:R35[7:6]  
	 R912_Array[27] = (R912_Array[27] & 0x3F) | SysFreq_Info1.NRB_BW;

     //printf("in set fre wwr\n");
    // R912_ReadReg();

 	 // Set TF LPF
	// R912_I2C.RegAddr = 0x0C;	// R912:R12[6]  
	 R912_Array[4] = (R912_Array[4] & 0xBF) | SysFreq_Info1.BYP_LPF;

	//R912_I2C.RegAddr = 0x2E;	//0 	 R912:R46[3:1]=0'b000	
	R912_Array[38] = (R912_Array[38] & 0xF1) | SysFreq_Info1.RF_FAST_DISCHARGE;  

	//R912_I2C.RegAddr = 0x16;	//4   R912:R22[7:5]=2'b010	
	R912_Array[14] = (R912_Array[14] & 0x1F) | SysFreq_Info1.RF_SLOW_DISCHARGE;  

	//R912_I2C.RegAddr = 0x26;	//1   R912:R38[4]=1	
	R912_Array[30] = (R912_Array[30] & 0xEF) | SysFreq_Info1.RFPD_PLUSE_ENA;  

	//R912_I2C.RegAddr = 0x2B;	//10  R912:R43[4:0]=5'b01010	
	R912_Array[35] = (R912_Array[35] & 0xE0) | SysFreq_Info1.LNA_FAST_DISCHARGE;  

	//R912_I2C.RegAddr = 0x16;	//0  R912:R22[4:2]=3'b000
	R912_Array[14] = (R912_Array[14] & 0xE3) | SysFreq_Info1.LNA_SLOW_DISCHARGE;

	//R912_I2C.RegAddr = 0x11;	//1  R912:R17[7]=0
	R912_Array[9] = (R912_Array[9] & 0x7F) | SysFreq_Info1.LNAPD_PLUSE_ENA;


	// Set AGC clk 
	//R912_I2C.RegAddr = 0x1A;	//  R912:R26[4:2] 
	R912_Array[18] = (R912_Array[18] & 0xE3) | SysFreq_Info1.AGC_CLK;  

	 //no clk out
	// R912_I2C.RegAddr = 0x19;
	 R912_Array[17] = (R912_Array[17] | 0x80);   //no clk out // R912:R25[7]  

	 if(R912_INFO.R912_Standard<R912_ATV_SIZE)   //ATV
	 {
		 if(R912_XtalDiv == XTAL_DIV1)
		 {
			 //AGC CLK to 500hz		// R912:R26[4:2]   26-8=18   26(0x1A) is addr ; [18] is data
			 R912_Array[18] = (R912_Array[18] & 0xE3) | 0x14;  //[4:2]=101
		 }
		 else
		 {
			 //AGC CLK to 1khz
			 R912_Array[18] = (R912_Array[18] & 0xE3) | 0x00;  //[4:2]=000
		 }
		 //R912_I2C.RegAddr = 0x1A;						
		 //R912_I2C.Data = R912_Array[18];
		 //if(I2C_Write(priv, &R912_I2C) != RT_Success)
		//	 return RT_Fail;
	 }
	 else
	 {
		 //for DVB-T2
         switch(R912_INFO.R912_Standard)
		 {
		    case R912_DVB_T2_6M:
			case R912_DVB_T2_7M:
			case R912_DVB_T2_8M:
			case R912_DVB_T2_1_7M:
			case R912_DVB_T2_10M:
			case R912_DVB_T2_6M_IF_5M:
			case R912_DVB_T2_7M_IF_5M:
			case R912_DVB_T2_8M_IF_5M:
			case R912_DVB_T2_1_7M_IF_5M:

                 //R912_Delay_MS(100);
				 //force plain mode
				 //R912_I2C.RegAddr = 0x0B;	// R912:R11[7]   		
				 R912_Array[3] = (R912_Array[3] | 0x80);  //[7]=1

				 //R912_I2C.RegAddr = 0x0A;	// R912:R10[5]   		
				 R912_Array[2] = (R912_Array[2] & 0xDF);  //[5]=0

				 //LPF
				 if((R912_INFO.R912_Standard==R912_DVB_T_8M_IF_5M) || (R912_INFO.R912_Standard==R912_DVB_T2_8M_IF_5M))
				 {
					  if(R912_INFO.RF_KHz>=800000) 
					  {
						  R912_Array[10] = (R912_Array[10] & 0xF0) | (R912_Fil_Cal_code[R912_INFO.R912_Standard]-2);  
					  }
					  else
					  {
						  R912_Array[10] = (R912_Array[10] & 0xF0) | (R912_Fil_Cal_code[R912_INFO.R912_Standard]);  
					  }		 			  	
						//R912_I2C.RegAddr = 0x12;		//  R912:R18[3:0]  		
						//R912_I2C.Data = R912_Array[10];
						//if(I2C_Write(priv, &R912_I2C) != RT_Success)
						//	return RT_Fail;
				 }

				 break;

		     default:		 
                 break;
		 }//end switch
	 }

	R912_I2C_Len.RegAddr = 0x08;   //  R912:0x08
	R912_I2C_Len.Len = R912_REG_NUM;
	for(RegArrayCunt = 0; RegArrayCunt<R912_REG_NUM; RegArrayCunt ++)
	{
		R912_I2C_Len.Data[RegArrayCunt] = R912_Array[RegArrayCunt];
	}
	if(I2C_Write_Len(priv,&R912_I2C_Len) != RT_Success)
		return RT_Fail;


	 return RT_Success;
}




R912_ErrCode R912_DVBS_Setting(struct r912_priv *priv, R912_Set_Info R912_INFO)
{
    u8 fine_tune=0;
    u8 Coarse=0;

    if(R912_INFO.R912_Standard != R912_pre_standard)
    {
	if(R912_InitReg(priv,R912_INFO.R912_Standard) != RT_Success)      
	     return RT_Fail;

	R912_pre_DVBS_bw = 0; //initial bw value


	//Output Signal Mode    (  O is diff  ; 1 is single  )
	if(R912_INFO.R912_DVBS_OutputSignal_Mode != SINGLEOUT)
	{
		R912_Array[35] &=0x7F;
	}
	else
	{
		R912_Array[35] |=0x80;  //R710 R11[4]    R912:R43[7]   43-8=35   43(0x2B) is addr ; [35] is data
	}

	R912_I2C.RegAddr = 0x2B;
	R912_I2C.Data = R912_Array[35];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;



	//AGC Type  //R13[4] Negative=0 ; Positive=1;
	if(R912_INFO.R912_DVBS_AGC_Mode != AGC_POSITIVE)
	{
		R912_Array[37] &= 0xF7;
	}
	else
	{
		R912_Array[37] |= 0x08;  //R710 R13[4]    R912:R45[3]   45-8=37   45(0x2D) is addr ; [37] is data
	}
	R912_I2C.RegAddr = 0x2D;
	R912_I2C.Data = R912_Array[37];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;


	//RT710_Vga_Sttenuator_Type
	if(R912_INFO.R912_DVBS_AttenVga_Mode != ATTENVGAON)
	{
		R912_Array[34] &= 0x7F;
	}
	else
	{
		R912_Array[34] |= 0x80;
	}
	R912_I2C.RegAddr = 0x2A;
	R912_I2C.Data = R912_Array[34];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;


	//R710_Fine_Gain_Type
	switch(R912_INFO.R912_DVBS_FineGain)
	{
		case FINEGAIN_3DB:  
			 R912_Array[38] = (R912_Array[38] & 0x3F) | 0x00;
		break;
		case FINEGAIN_2DB:  
			 R912_Array[38] = (R912_Array[38] & 0x3F) | 0x40;
		break;
		case FINEGAIN_1DB:  
			 R912_Array[38] = (R912_Array[38] & 0x3F) | 0x80;
		break;
		case FINEGAIN_0DB:  
			 R912_Array[38] = (R912_Array[38] & 0x3F) | 0xC0;
		break;
		default:		
			R912_Array[38] = (R912_Array[38] & 0x3F) | 0x00;
		break;
	}
	R912_I2C.RegAddr = 0x2E;
	R912_I2C.Data = R912_Array[38];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;

		//Set GPO Low
		R912_I2C.RegAddr = 0x17;	// R912:R23[4:2]  23-8=15  23(0x17) is addr ; [15] is data
		R912_Array[15] = (R912_Array[15] & 0xFE);
		R912_I2C.Data = R912_Array[15];
		if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;
		
		R912_pre_standard = R912_INFO.R912_Standard;
	}

	if(R912_PLL(priv,R912_INFO.RF_KHz, R912_INFO.R912_Standard)!= RT_Success)
	{
		return RT_Fail;
	}
	
	//VTH/VTL
	if((R912_INFO.RF_KHz >= 1200000)&&(R912_INFO.RF_KHz <= 1750000))
	{
		R912_Array[23]=(R912_Array[23] & 0x00) | 0x93;			//R912:R31[7:0]    1.24/0.64
	}
	else
	{	
		R912_Array[23]=(R912_Array[23] & 0x00) | 0x83;			//R912:R31[7:0]   1.14/0.64
	}
	R912_I2C.RegAddr = 0x1F;
	R912_I2C.Data = R912_Array[23];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	if(R912_INFO.RF_KHz >= 2000000) 
	{
		R912_Array[38]=(R912_Array[38] & 0xCF) | 0x20;			//R912:R46[4:5]
	}
	else
	{
		R912_Array[38]=(R912_Array[38] & 0xCF) | 0x30;			//R912:R46[4:5]
	}
	R912_I2C.RegAddr = 0x2E;
	R912_I2C.Data = R912_Array[38];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	if((R912_INFO.RF_KHz >= 1600000) && (R912_INFO.RF_KHz <= 1950000))
	{
		R912_Array[35] |= 0x20; //LNA Mode with att   //R710 R2[6]    R912:R43[5]   43-8=35   43(0x2B) is addr ; [35] is data
	}
	else
	{
		R912_Array[35] &= 0xDF; //LNA Mode no att
	}

	R912_I2C.RegAddr = 0x2B;
	R912_I2C.Data = R912_Array[35];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
	return RT_Fail;



	if(R912_INFO.DVBS_BW != R912_pre_DVBS_bw)
	{
		if (R912_INFO.DVBS_BW >67400)
		{
			fine_tune=1;
			Coarse =(u8)(( R912_INFO.DVBS_BW -67400) /1600)+31;
			if((( R912_INFO.DVBS_BW -67400) % 1600) > 0)
			Coarse+=1;
		}

	else if((R912_INFO.DVBS_BW >62360)&&(R912_INFO.DVBS_BW<=67400))
	{
		Coarse=31;
		fine_tune=1;		
	}
	else if((R912_INFO.DVBS_BW >38000)&&(R912_INFO.DVBS_BW<=62360))
	{
		fine_tune=1;	
		Coarse =(( R912_INFO.DVBS_BW -38000) /1740)+16;
		if((( R912_INFO.DVBS_BW -38000) % 1740) > 0)
		Coarse+=1;
				
	}
	else if(R912_INFO.DVBS_BW<=5000)
	{	
		Coarse=0;
		fine_tune=0;
	}
	else if((R912_INFO.DVBS_BW>5000) && (R912_INFO.DVBS_BW<=8000))
	{
		Coarse=0;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>8000) && (R912_INFO.DVBS_BW<=10000))
	{
		Coarse=1;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>10000) && (R912_INFO.DVBS_BW<=12000))
	{
		Coarse=2;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>12000) && (R912_INFO.DVBS_BW<=14200))
	{
		Coarse=3;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>14200) && (R912_INFO.DVBS_BW<=16000))
	{
		Coarse=4;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>16000) && (R912_INFO.DVBS_BW<=17800))
	{
		Coarse=5;
		fine_tune=0;
	}
	else if((R912_INFO.DVBS_BW>17800) && (R912_INFO.DVBS_BW<=18600))
	{
		Coarse=5;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>18600) && (R912_INFO.DVBS_BW<=20200))
	{
		Coarse=6;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>20200) && (R912_INFO.DVBS_BW<=22400))
	{
		Coarse=7;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>22400) && (R912_INFO.DVBS_BW<=24600))
	{
		Coarse=9;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>24600) && (R912_INFO.DVBS_BW<=25400))
	{
		Coarse=10;
		fine_tune=0;
	}
	else if((R912_INFO.DVBS_BW>25400) && (R912_INFO.DVBS_BW<=26000))
	{
		Coarse=10;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>26000) && (R912_INFO.DVBS_BW<=27200))
	{
		Coarse=11;
		fine_tune=0;
	}
	else if((R912_INFO.DVBS_BW>27200) && (R912_INFO.DVBS_BW<=27800))
	{
		Coarse=11;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>27800) && (R912_INFO.DVBS_BW<=30200))
	{
		Coarse=12;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>30200) && (R912_INFO.DVBS_BW<=32600))
	{
		Coarse=13;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>32600) && (R912_INFO.DVBS_BW<=33800))
	{
		Coarse=14;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>33800) && (R912_INFO.DVBS_BW<=36800))
	{
		Coarse=15;
		fine_tune=1;
	}
	else if((R912_INFO.DVBS_BW>36800) && (R912_INFO.DVBS_BW<=38000))
	{
		Coarse=16;
		fine_tune=1;
	}

		//Coarse_tune = (unsigned char) Coarse;//coras filter value

	//fine tune and coras filter write		
	R912_I2C.RegAddr = 0x2F;
	R912_Array[39] &= 0x00;		//47-8=39
	R912_Array[39] = ((R912_Array[39] | ( fine_tune<< 6 ) ) | ( Coarse));
	R912_I2C.Data = R912_Array[39];
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		return RT_Fail;

	R912_pre_DVBS_bw = R912_INFO.DVBS_BW;
   }

   return RT_Success;
}
R912_ErrCode R912_SetPllData(struct r912_priv *priv, R912_Set_Info R912_INFO)
{	  
	if(R912_Initial_done_flag==R912_FALSE)
	{
		R912_Init(priv);
	}

	if(R912_INFO.R912_Standard!=R912_DVB_S)
	{
      if(R912_SetStandard(priv,R912_INFO.R912_Standard) != RT_Success)
	  {		
		  return RT_Fail;
	  }

      if(R912_SetFrequency(priv,R912_INFO) != RT_Success)
	  {       
		  return RT_Fail;
	  }
	  R912_SATELLITE_FLAG = 0;
	}
	else
	{
		 if(R912_DVBS_Setting(priv,R912_INFO) != RT_Success)

		  return RT_Fail;
		 R912_SATELLITE_FLAG = 1;
	}

      return RT_Success;
}


R912_ErrCode R912_Standby(struct r912_priv *priv)
{
	 // R912:R8[7]   
	 R912_Array[0] = (R912_Array[0] | 0x80);         //LNA off   All / LNA / Buffer PW off
	//  R912:R37[3]  	
	 R912_Array[31] = R912_Array[31] | 0x08;        //LNA det off


	 R912_I2C.RegAddr = 0x08;	//  R912:R8[7]   
	 R912_I2C.Data = R912_Array[0];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //LT PW off
	 R912_I2C.RegAddr = 0x09;	// R912:R9[0]   
	 R912_Array[1] = R912_Array[1] | 0x01;
	 R912_I2C.Data = R912_Array[1];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;


	 //RF Buffer PW off
	 R912_I2C.RegAddr = 0x09;	// R912:R9[0] 	
	 R912_Array[1] = R912_Array[1] | 0x04;
	 R912_I2C.Data = R912_Array[1];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //RF, Nrb Det PW off
	 R912_I2C.RegAddr = 0x25;	// R912:R37[3] 
	 R912_Array[31] = R912_Array[31] | 0x05;
	 R912_I2C.Data = R912_Array[31];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //LNA current = lowest, R6[1:0]=11   
	 R912_I2C.RegAddr = 0x0E;	// R912:R14[1:0] 
	 R912_Array[6] = R912_Array[6] | 0x03;
	 R912_I2C.Data = R912_Array[6];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;


	 //NAT PW off
	 R912_I2C.RegAddr = 0x26;	// R912:R38[7]   
	 R912_Array[30] = R912_Array[30] | 0x80;
	 R912_I2C.Data = R912_Array[30];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //Mixer PW off
	 R912_I2C.RegAddr = 0x0C;	// R912:R12[3]  
	 R912_Array[4] = R912_Array[4] | 0x08;
	 R912_I2C.Data = R912_Array[4];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;


	 //Filter PW off    //pwd_filt / all / vga / poly / amp
	 R912_I2C.RegAddr = 0x12;	//  R912:R18[7]   
	 R912_Array[10] = R912_Array[10] | 0x80;
	 R912_I2C.Data = R912_Array[10];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //ADC PW off
	 R912_I2C.RegAddr = 0x0F;	// R912:R15[7]   
	 R912_Array[7] = R912_Array[7] | 0x80;
	 R912_I2C.Data = R912_Array[7];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;


     //PLL LDO A off
	 R912_I2C.RegAddr = 0x15;	//  R912:R21[5:4]   
	 R912_Array[13] = R912_Array[13] | 0x30;
	 R912_I2C.Data = R912_Array[13];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //PLL DLDO 1& 2 off
	 R912_I2C.RegAddr = 0x15;	//  R912:R21[3:0]   
	 R912_Array[13] = R912_Array[13] | 0x0F;
	 R912_I2C.Data = R912_Array[13];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	 //PLL SELS & SELT off
	 R912_I2C.RegAddr = 0x18;	//  R912:R24[3]  
	 R912_Array[16] = R912_Array[16] & 0xE7;
	 R912_I2C.Data = R912_Array[16];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //AGC off
	 R912_I2C.RegAddr = 0x15;	//R912:R21[6]  
	 R912_Array[13] = R912_Array[13] | 0x40;
	 R912_I2C.Data = R912_Array[13];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	//RF_DET_POWER
	 R912_I2C.RegAddr = 0x28;	//R912:R40[0]   
	 R912_Array[32] = R912_Array[32] | 0x01;
	 R912_I2C.Data = R912_Array[32];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	 //NET_DET PW
	 R912_I2C.RegAddr = 0x28;	//R912:R40[3]   40-8=32   
	 R912_Array[32] = R912_Array[32] | 0x08;
	 R912_I2C.Data = R912_Array[32];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	 //ALL_LNA_BUF PW
	 R912_I2C.RegAddr = 0x28;	//R912:R40[3]   40-8=32  
	 R912_Array[32] = R912_Array[32] | 0x10;
	 R912_I2C.Data = R912_Array[32];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	 //AMP_FILT PW
	 R912_I2C.RegAddr = 0x28;	//R912:R40[6]   40-8=32  
	 R912_Array[32] = R912_Array[32] | 0x40;
	 R912_I2C.Data = R912_Array[32];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;

	//LNA PW
	 R912_I2C.RegAddr = 0x28;	//R912:R40[6]   40-8=32   
	 R912_Array[32] = R912_Array[32] | 0x80;
	 R912_I2C.Data = R912_Array[32];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	//BYP_LPF
	 R912_I2C.RegAddr = 0x0C;	//R912:R12[6]   12-8=4   
	 R912_Array[4] = R912_Array[4] & 0xBF;
	 R912_I2C.Data = R912_Array[4];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	 //XTAL LDO PW
	 R912_I2C.RegAddr = 0x17;	//R912:R23[2]   23-8=15  
	 R912_Array[15] = R912_Array[15] | 0x04;
	 R912_I2C.Data = R912_Array[15];
	 if(I2C_Write(priv, &R912_I2C) != RT_Success)
			return RT_Fail;
	
	return RT_Success;
}



R912_ErrCode R912_GetRfGain(struct r912_priv *priv, R912_RF_Gain_Info *pR912_rf_gain)
{

	R912_I2C_Len.RegAddr = 0x00;
	R912_I2C_Len.Len =0x05;
	if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
	{
		I2C_Read_Len(priv,&R912_I2C_Len);
	}
	pR912_rf_gain->RF_gain1 = (R912_I2C_Len.Data[4] & 0x1F);            //lna    //848:4[4:0]  
	pR912_rf_gain->RF_gain2 = ((R912_I2C_Len.Data[3] & 0xF0) >> 4);     //rf	 //848:3[4:0]  
	pR912_rf_gain->RF_gain3 = (R912_I2C_Len.Data[3] & 0x0F);             //mixer //848:3[4:0]  	
/*
	Test_Len.Data[0]=0;
	Test_Len.Data[1]=0;
	Test_Len.RegAddr = 0x03;
	Test_Len.Len =2;
	if(I2C_Read_Len(&Test_Len) != RT_Success)
		return RT_Fail;

	pR912_rf_gain->RF_gain1 = (Test_Len.Data[1] & 0x1F);            //lna    //848:4[4:0]  
	pR912_rf_gain->RF_gain2 = ((Test_Len.Data[0] & 0xF0) >> 4);  //rf		 //848:3[4:0]  
	pR912_rf_gain->RF_gain3 = (Test_Len.Data[0] & 0x0F);             //mixer //848:3[4:0]  */
	


	if(R912_SATELLITE_FLAG==0)
	{
		if(pR912_rf_gain->RF_gain1 > 22) 
			pR912_rf_gain->RF_gain1 = 22;  //LNA gain max is 22

		if(pR912_rf_gain->RF_gain3 > 10)
			pR912_rf_gain->RF_gain3 = 10;  //MixerAmp gain max is 10

		pR912_rf_gain->RF_gain_comb = (pR912_rf_gain->RF_gain1*14 + pR912_rf_gain->RF_gain2*12 + pR912_rf_gain->RF_gain3*12);
	}
	else
	{
		if (pR912_rf_gain->RF_gain1 <= 2)
		{
			pR912_rf_gain->RF_gain1=0;
		}
		else if(pR912_rf_gain->RF_gain1 > 2 && pR912_rf_gain->RF_gain1 <= 9) 
		{
			pR912_rf_gain->RF_gain1 -=2;
		}
		else if(pR912_rf_gain->RF_gain1 >9 && pR912_rf_gain->RF_gain1 <=12)
		{
			pR912_rf_gain->RF_gain1 = 8;
		}
		else if(pR912_rf_gain->RF_gain1 >12 && pR912_rf_gain->RF_gain1 <= 20)
		{
			pR912_rf_gain->RF_gain1 -= 4;
		}
		else if(pR912_rf_gain->RF_gain1 > 20 && pR912_rf_gain->RF_gain1 <= 22 )
		{
			pR912_rf_gain->RF_gain1 = 17;
		}
		else if(pR912_rf_gain->RF_gain1 > 22 )
		{
			pR912_rf_gain->RF_gain1 = 18;
		}
	}

    return RT_Success;
}


R912_ErrCode R912_RfGainMode(struct r912_priv *priv, R912_RF_Gain_TYPE R912_RfGainType)
{
    u8 MixerGain = 0;
	u8 RfGain = 0;
	u8 LnaGain = 0;

	if(R912_RfGainType==RF_MANUAL)
	{
		//LNA auto off
	     R912_I2C.RegAddr = 0x0D;
	     R912_Array[5] = R912_Array[5] | 0x80;   // 848:13[7:0]   
		 R912_I2C.Data = R912_Array[5];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //Mixer buffer off
	     R912_I2C.RegAddr = 0x22;
	     R912_Array[26] = R912_Array[26] | 0x10;  // 848:34[7:0]   
		 R912_I2C.Data = R912_Array[26];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //Mixer auto off
	     R912_I2C.RegAddr = 0x0F;
	     R912_Array[7] = R912_Array[7] & 0xEF;  //848:15[6:0]
		 R912_I2C.Data = R912_Array[7];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		R912_I2C_Len.RegAddr = 0x00;
		R912_I2C_Len.Len     = 5; 
		if(I2C_Read_Len(priv,&R912_I2C_Len) != RT_Success)
		{
			I2C_Read_Len(priv,&R912_I2C_Len);
		}

		//MixerGain = (((R912_I2C_Len.Data[1] & 0x40) >> 6)<<3)+((R912_I2C_Len.Data[3] & 0xE0)>>5);   //?
		MixerGain = (R912_I2C_Len.Data[3] & 0x0F); //mixer // 848:3[4:0]
		RfGain = ((R912_I2C_Len.Data[3] & 0xF0) >> 4);   //rf		 // 848:3[4:0] 
		LnaGain = R912_I2C_Len.Data[4] & 0x1F;  //lna    // 848:4[4:0]  

		//set LNA gain
	     R912_I2C.RegAddr = 0x0D;
	     R912_Array[5] = (R912_Array[5] & 0xE0) | LnaGain;  // 848:13[7:0]  
		 R912_I2C.Data = R912_Array[5];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //set Mixer Buffer gain
	     R912_I2C.RegAddr = 0x22;
	     R912_Array[26] = (R912_Array[26] & 0xF0) | RfGain;  //848:34[7:0] 
		 R912_I2C.Data = R912_Array[26];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //set Mixer gain
	     R912_I2C.RegAddr = 0x0F;
	     R912_Array[7] = (R912_Array[7] & 0xF0) | MixerGain; // 848:15[6:0]  
		 R912_I2C.Data = R912_Array[7];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;
	}
	else
	{
	    //LNA auto on
	     R912_I2C.RegAddr = 0x0D;
	     R912_Array[5] = R912_Array[5] & 0x7F;  // 848:13[7:0]  
		 R912_I2C.Data = R912_Array[5];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //Mixer buffer on
	     R912_I2C.RegAddr = 0x22;
	     R912_Array[26] = R912_Array[26] & 0xEF;	// 848:34[7:0]  
		 R912_I2C.Data = R912_Array[26];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;

		 //Mixer auto on
	     R912_I2C.RegAddr = 0x0F;
	     R912_Array[7] = R912_Array[7] | 0x10;	// 848:15[6:0]  
		 R912_I2C.Data = R912_Array[7];
	     if(I2C_Write(priv, &R912_I2C) != RT_Success)
		       return RT_Fail;
	}

    return RT_Success;
}




//--------- Set IC_Internal_cap------------------------------------//
//   Xtal loading match formula: 
//   Xtal_cap_load*2 = IC_Internal_cap + External_cap
//   (default is for Xtal 16pF load)
//--------------------------------------------------------------------//
R912_ErrCode R912_SetXtalIntCap(struct r912_priv *priv, R912_Xtal_Cap_TYPE R912_XtalCapType)
{
	 //Set internal Xtal cap 
	if(R912_XtalCapType==XTAL_CAP_LARGE)  //for Xtal CL >16pF
		R912_Array[19] = R912_Array[19] & 0xDF;
	else       //for Xtal CL < 16pF
		R912_Array[19] = R912_Array[19] | 0x20;

     R912_I2C.RegAddr = 0x1B;
	 R912_I2C.Data = R912_Array[19];
     if(I2C_Write(priv, &R912_I2C) != RT_Success)
	       return RT_Fail;

	return RT_Success;
}

//-------------------------------------------------------------------------------------------

void r912_release(struct dvb_frontend *fe)
{

	kfree(fe->tuner_priv);
	fe->tuner_priv = NULL;
	return;
}

static int r912_init(struct dvb_frontend *fe)
{
	struct r912_priv *priv = fe->tuner_priv;
	int ret = 0;
#if 0
	u8 i;

	

	//if (priv->inited == 1)
	//	return 0;
	

	//priv->inited = 1;

	

	if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)
		return RT_Fail;
	
	

	if(R912_TF_Check(priv) != RT_Success)
		return RT_Fail;
	
	

	//start IMR calibration
	if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)        //write initial reg before doing IMR Cal
		return RT_Fail;
		
	

	if(R912_Cal_Prepare(priv,R912_IMR_CAL) != RT_Success)
		return RT_Fail;
	
	

	if(R912_IMR(priv,3, 0) != RT_Success)       //Full K node 3
		return RT_Fail;
		
	

	if(R912_IMR(priv,0, 1) != RT_Success)
		return RT_Fail;
	
	

	if(R912_IMR(priv,1, 1) != RT_Success)
		return RT_Fail;
	
	

	if(R912_IMR(priv,2, 1) != RT_Success)
		return RT_Fail;
	
	

	if(R912_IMR(priv,4, 0) != RT_Success)   //Full K node 4
		return RT_Fail;
		
	

	//do Xtal check
	if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)
		return RT_Fail;
	
	

	//priv->cfg->R912_Xtal_Pwr = XTAL_SMALL_LOWEST;
	//priv->cfg->R912_Xtal_Pwr_tmp = XTAL_SMALL_LOWEST;
	
	priv->cfg->R912_Xtal_Pwr = XTAL_SMALL_HIGHEST;
	priv->cfg->R912_Xtal_Pwr_tmp = XTAL_LARGE_HIGHEST;

	for (i=0; i<3; i++) {
		if(R912_Xtal_Check(priv) != RT_Success)
			return RT_Fail;

		if(priv->cfg->R912_Xtal_Pwr_tmp > priv->cfg->R912_Xtal_Pwr)
			priv->cfg->R912_Xtal_Pwr = priv->cfg->R912_Xtal_Pwr_tmp;
	}
	
	

	//write initial reg
	if(R912_InitReg(priv,R912_STD_SIZE) != RT_Success)
		return RT_Fail;
	
	

	priv->cfg->R912_pre_standard = R912_STD_SIZE;
#endif

	//ret = R912_Init(priv);
#if 1
	if (R912_Init(priv) != RT_Success)
		dev_dbg("init tuner failed!!!\n");
//	else
		//priv->inited = 1;//vit
		
#endif
		
#if 0		
	//if (R912_GPO(priv, HI_SIG) != RT_Success)
	if (R912_GPO(priv, LO_SIG) != RT_Success)
		dev_dbg("R912_GPO failed!!!\n");
		
#endif

#if 0 //debug regs
	//R912_Initial_done_flag = R912_TRUE;
	
	//test write
	R912_I2C.RegAddr = 0x0A;
	R912_I2C.Data = 0x77;
	if(I2C_Write(priv, &R912_I2C) != RT_Success)
		dev_dbg("I2C_Write failed!!!\n");
	else
		dev_dbg("I2C_Write OK!!!\n");
	
	//read all regs 
	if (R912_ReadReg(priv) != RT_Success)
		dev_dbg("R912_ReadReg failed!!!\n");
		
#endif
	
	return ret;
}

static int r912_sleep(struct dvb_frontend *fe)
{
//	struct r912_priv *priv = fe->tuner_priv;
	int ret = 0;
	
	//R912_Initial_done_flag = R912_FALSE;//vit test
	//R912_IMR_done_flag = R912_FALSE;//vit test

	return ret;
}

static int r912_set_params(struct dvb_frontend *fe)
{
	struct r912_priv *priv = fe->tuner_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int ret = 0, i;
	//u8 tuner_lock;

	R912_Set_Info R912_INFO;

	dev_dbg("delivery_system=%d frequency=%d " \
			"symbol_rate=%d bandwidth_hz=%d\n",
			c->delivery_system, c->frequency, c->symbol_rate,
			c->bandwidth_hz);


	/* failsafe */
	//R912_INFO.R912_Standard = R912_DVB_T2_8M_IF_5M;
	R912_INFO.R912_Standard = R912_DVB_S;

	switch (c->delivery_system) {
	case SYS_DVBC_ANNEX_A:
	case SYS_DVBC_ANNEX_C:
		R912_INFO.RF_KHz = c->frequency / 1000;
		if(c->bandwidth_hz <= 6000000) {
        		R912_INFO.R912_Standard = R912_DVB_C_6M_IF_5M;
		} else if (c->bandwidth_hz <= 8000000) {
			R912_INFO.R912_Standard = R912_DVB_C_8M_IF_5M;
		}
#if 0
		/* set pll data */
		if(R912_SetStandard(priv, R912_INFO.R912_Standard) != RT_Success) {
			return RT_Fail;
		}
		if(R912_SetFrequency(priv, R912_INFO) != RT_Success) {
			return RT_Fail;
		}
#endif
		if(R912_SetPllData(priv, R912_INFO) != RT_Success) {
			dev_dbg("R912_SetPllData failed!!!\n");
			return RT_Fail;
		}
		dev_dbg("R912_SetPllData for DVB-C\n");
		dev_dbg("R912_SetStandard=%d\n", R912_INFO.R912_Standard);
		dev_dbg("R912_SetFrequency=%d KHz\n", R912_INFO.RF_KHz);
		break;
	case SYS_DVBC_ANNEX_B:
		R912_INFO.RF_KHz = c->frequency / 1000;
       		R912_INFO.R912_Standard = R912_J83B_IF_5M;
#if 0
		/* set pll data */
		if(R912_SetStandard(priv, R912_INFO.R912_Standard) != RT_Success) {
			return RT_Fail;
		}
		if(R912_SetFrequency(priv, R912_INFO) != RT_Success) {
			return RT_Fail;
		}
#endif
		if(R912_SetPllData(priv, R912_INFO) != RT_Success) {
			dev_dbg("R912_SetPllData failed!!!\n");
			return RT_Fail;
		}
			dev_dbg("R912_SetPllData for DVB-C Annex B\n");
		dev_dbg("R912_SetStandard=%d\n", R912_INFO.R912_Standard);
		dev_dbg("R912_SetFrequency=%d KHz\n", R912_INFO.RF_KHz);
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
		R912_INFO.RF_KHz = c->frequency / 1000;
		if(c->bandwidth_hz <= 1700000) {
			R912_INFO.R912_Standard = R912_DVB_T2_1_7M_IF_5M;
		} else if (c->bandwidth_hz <= 6000000) {
			R912_INFO.R912_Standard = R912_DVB_T2_6M_IF_5M;
		} else if (c->bandwidth_hz <= 7000000) {
			R912_INFO.R912_Standard = R912_DVB_T2_7M_IF_5M;
		} else { //if(c->bandwidth_hz == 8000000) {
			R912_INFO.R912_Standard = R912_DVB_T2_8M_IF_5M;
		}
#if 0
		/* set pll data */
		if(R912_SetStandard(priv, R912_INFO.R912_Standard) != RT_Success) {
			return RT_Fail;
		}
		if(R912_SetFrequency(priv, R912_INFO) != RT_Success) {
			return RT_Fail;
		}
#endif
		if(R912_SetPllData(priv, R912_INFO) != RT_Success) {
			dev_dbg("R912_SetPllData failed!!!\n");
			return RT_Fail;
		}
		dev_dbg("R912_SetPllData for DVB-T\n");
		dev_dbg("R912_SetStandard=%d\n", R912_INFO.R912_Standard);
		dev_dbg("R912_SetFrequency=%d KHz\n", R912_INFO.RF_KHz);
		dev_dbg("c->bandwidth_hz=%d\n", c->bandwidth_hz);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		R912_INFO.RF_KHz = c->frequency;
		R912_INFO.R912_Standard = R912_DVB_S;
		R912_INFO.DVBS_BW = (c->symbol_rate/200*135+2000000)/1000*2;//unit KHz
		R912_INFO.R912_DVBS_OutputSignal_Mode = DIFFERENTIALOUT;
		R912_INFO.R912_DVBS_AGC_Mode = AGC_NEGATIVE;
#if 0
		/* set pll data */
		if(R912_DVBS_Setting(priv,R912_INFO) != RT_Success)
			return RT_Fail;
#endif
		if(R912_SetPllData(priv, R912_INFO) != RT_Success) {
			dev_dbg("R912_SetPllData failed!!!\n");
			return RT_Fail;
		}
		dev_dbg("R912_SetPllData for DVB-S\n");
		dev_dbg("R912_SetStandard=%d\n", R912_INFO.R912_Standard);
		dev_dbg("R912_SetFrequency=%d KHz\n", R912_INFO.RF_KHz);
		break;
	}

#if 0
	if (ret) {
		printk("[r912_lock_n_wait] Tuner lock function Failed!\n");
		goto exit;
	}
#endif
	for (i = 0; i < 5; i++) {
	    ret = R912_GetLockStatus(priv);
	    if(!ret)  {
		dev_dbg("Tuner Locked.\n");
		break;
	    }
	    dev_dbg("Tuner not Locked!\n");
	    msleep(20);  
	}
//exit:

	return ret;
}

static const struct dvb_tuner_ops r912_tuner_ops = {
	.info = {
	.name           = "Rafael R912",

		.frequency_min_hz  = 175 * MHz,
		.frequency_max_hz  = 2150 * MHz,
		.frequency_step_hz = 200 * kHz,
	},

	.release = r912_release,

	.init = r912_init,
	.sleep = r912_sleep,
	.set_params = r912_set_params,
};

struct dvb_frontend *r912_attach(struct dvb_frontend *fe,
		struct r912_config *cfg, struct i2c_adapter *i2c)
{
	struct r912_priv *priv = NULL;

	priv = kzalloc(sizeof(struct r912_priv), GFP_KERNEL);
	if (priv == NULL) {
		printk( "R912: attach failed\n");
		return NULL;
	}

	priv->cfg = cfg;
	priv->i2c = i2c;
	priv->inited = 0;

	dev_info(&priv->i2c->dev, "%s: Rafael R912 successfully attached!\n", KBUILD_MODNAME);

	memcpy(&fe->ops.tuner_ops, &r912_tuner_ops, sizeof(struct dvb_tuner_ops));

	fe->tuner_priv = priv;
	return fe;
}
EXPORT_SYMBOL(r912_attach);

MODULE_DESCRIPTION("Rafael R912 silicon tuner driver");
MODULE_AUTHOR("Luis Alves <ljalvs@gmail.com>");
MODULE_LICENSE("GPL");

