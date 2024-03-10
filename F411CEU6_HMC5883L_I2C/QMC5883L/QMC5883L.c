
#include "QMC5883L.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t QMC5883L_Read_Reg(uint8_t reg)
{
	uint8_t Buffer[1];
	HAL_I2C_Mem_Read(QMC5883L_I2C_PORT,QMC5883L_ADDRESS,reg,1,Buffer,1,10);
	return Buffer[0];
}

void QMC5883L_Write_Reg(uint8_t reg, uint8_t data)
{
	uint8_t Buffer[2]={reg,data};
	HAL_I2C_Master_Transmit(QMC5883L_I2C_PORT,QMC5883L_ADDRESS,Buffer,2,10);
}


void QMC5883L_Read_Data(int16_t *MagX,int16_t *MagY,int16_t *MagZ) // (-32768 / +32768)
{
	*MagX=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_X_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_X_MSB))<<8));
	*MagY=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Y_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Y_MSB))<<8));
	*MagZ=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Z_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Z_MSB))<<8));
}


int16_t QMC5883L_Read_Temperature()
{
	return (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_LSB)) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_MSB))<<8))/100;
}


void QMC5883L_Initialize(_qmc5883l_MODE MODE,_qmc5883l_ODR ODR,_qmc5883l_RNG RNG,_qmc5883l_OSR OSR)
{
	QMC5883L_Write_Reg(QMC5883L_CONFIG_3,0x01);
	QMC5883L_Write_Reg(QMC5883L_CONFIG_1,MODE | ODR | RNG | OSR);
}

void QMC5883L_Reset()
{
	QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x81);
}

void QMC5883L_InterruptConfig(_qmc5883l_INT INT)
{
	if(INT==INTERRUPT_ENABLE){QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x00);}
	else {QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x01);}
}


_qmc5883l_status QMC5883L_DataIsReady()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0x00)	  {return NO_NEW_DATA;}
	else if((Buffer&0x01)==0X01){return NEW_DATA_IS_READY;}
	return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsSkipped()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x04)==0X04){return DATA_SKIPPED_FOR_READING;}
		return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsOverflow()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x02)==0X02){return DATA_OVERFLOW;}
		return NORMAL;
}


void QMC5883L_ResetCalibration() 
{ 
	Xmin=Xmax=Ymin=Ymax=0;
} 

 
float QMC5883L_Heading(int16_t Xraw,int16_t Yraw,int16_t Zraw) 
{ 
   	float X=Xraw,Y=Yraw,Z=Zraw; 
   	float Heading;

  	if(X<Xmin) {Xmin = X;} 
    	  else if(X>Xmax) {Xmax = X;} 

  	if(Y<Ymin) {Ymin = Y;} 
    	  else if(Y>Ymax) {Ymax = Y;} 

  
  	if( Xmin==Xmax || Ymin==Ymax ) {return 0.0;} 

 
 	  X -= (Xmax+Xmin)/2; 
  	Y -= (Ymax+Ymin)/2; 
    
  	X = X/(Xmax-Xmin); 
  	Y = Y/(Ymax-Ymin); 

  	Heading = atan2(Y,X); 
		//EAST
	Heading += QMC5883L_DECLINATION_ANGLE;
	//WEST
	//Heading -= QMC5883L_DECLINATION_ANGLE;
		
	if(Heading <0)
   	  {Heading += 2*M_PI;}
	else if(Heading > 2*M_PI)
   	  {Heading -= 2*M_PI;}
 
 return Heading; 
} 


void QMC5883L_Scale(int16_t *X,int16_t *Y,int16_t *Z)
{
	*X*=QMC5883L_SCALE_FACTOR;
	*Y*=QMC5883L_SCALE_FACTOR;
	*Z*=QMC5883L_SCALE_FACTOR;
}





