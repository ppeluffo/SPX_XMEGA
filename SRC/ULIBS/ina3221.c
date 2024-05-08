/*
 * l_ina3232.c
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#include "ina3221.h"

//------------------------------------------------------------------------------
void INA_sleep(void)
{
    INA_config(INA_A, CONF_INA_SLEEP);
    INA_config(INA_B, CONF_INA_SLEEP);
}
//------------------------------------------------------------------------------
void INA_awake(void)
{
    INA_config(INA_A, CONF_INA_AVG128);
    INA_config(INA_B, CONF_INA_AVG128);
}
//------------------------------------------------------------------------------
uint8_t INA_id2busaddr( uint8_t ina_id )
{
	switch(ina_id) {
	case INA_A:
		// ina_U1 en SPX_5CH. Canales 0,1,2
		return(BUSADDR_INA_A);
		break;
	case INA_B:
		// ina_U2 en SPX_5CH. Canales 3,4,5
		return(BUSADDR_INA_B);
		break;
	default:
		return(99);
		break;
	}

	return(99);

}
//------------------------------------------------------------------------------------
void INA_config( uint8_t ina_id, uint16_t conf_reg_value )
{
char res[3] = { 0 };
int16_t xBytes = 0;

	res[0] = ( conf_reg_value & 0xFF00 ) >> 8;
	res[1] = ( conf_reg_value & 0x00FF );

	xBytes = INA_write( ina_id, INA3231_CONF, res, 2 );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:INA_config\r\n\0"));

}
//------------------------------------------------------------------------------------
int16_t INA_test_write ( char *ina_id_str, char *rconf_val_str )
{

	// Escribe en el registro de configuracion de un INA ( 0, 1, 2)

uint16_t val = 0;
uint8_t ina_id = 0;
char data[3] = { 0 };
int16_t xBytes = 0;

	ina_id = atoi(ina_id_str);
	val = atoi( rconf_val_str);
	data[0] = ( val & 0xFF00 ) >> 8;
	data[1] = ( val & 0x00FF );
	xBytes = INA_write( ina_id, INA3231_CONF, data, 2 );
	if ( xBytes == -1 )
		xprintf_P(PSTR("ERROR: I2C:INA_test_write\r\n\0"));

	return (xBytes);
}
//------------------------------------------------------------------------------------
int16_t INA_test_read ( char *ina_id_str, char *regs )
{

uint16_t val = 0;
uint8_t ina_id = 0;
char data[3] = { ' ' };
int16_t xBytes = 0;
char l_data[10] = { ' ' };

	memcpy(l_data, regs, sizeof(l_data));
	strupr(l_data);

	// read ina id {conf|chxshv|chxbusv|mfid|dieid}
	ina_id = atoi(ina_id_str);

//	xprintf_P( PSTR("in->DEBUG=[%02x][%02x]\r\n\0"), data[0], data[1]);

	if (strcmp_P( l_data, PSTR("CONF\0")) == 0 ) {
		xBytes = INA_read(  ina_id, INA3231_CONF, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH1SHV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH1_SHV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH1BUSV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH1_BUSV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH2SHV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH2_SHV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH2BUSV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH2_BUSV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH3SHV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH3_SHV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("CH3BUSV\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_CH3_BUSV, data, 2 );
	} else if (strcmp_P( l_data, PSTR("MFID\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_MFID, data, 2 );
	} else if (strcmp_P( l_data, PSTR("DIEID\0")) == 0) {
		xBytes = INA_read(  ina_id, INA3221_DIEID, data, 2 );
	} else {
		xBytes = -1;
	}

	if ( xBytes == -1 ) {
		xprintf_P(PSTR("ERROR: I2C:INA_test_read\r\n\0"));

	} else {

		val = ( data[0]<< 8 ) + data[1];
		xprintf_P( PSTR("INAID=%d\r\n\0"), ina_id);
		xprintf_P( PSTR("VAL=0x%04x\r\n\0"), val);
//		xprintf_P( PSTR("out->DEBUG=[%02x][%02x]\r\n\0"), data[0], data[1]);
	}

	return(xBytes);

}
//------------------------------------------------------------------------------------
int16_t INA_read( uint8_t dev_id, uint16_t rdAddress, char *data, uint8_t length )
{

int16_t rcode = 0;

    rcode =  I2C_read( fdI2C0, INA_id2busaddr(dev_id), rdAddress, 0x01, data, length );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf_P(PSTR("ERROR: INA(%d)_read recovering i2c bus\r\n"), dev_id );
		goto quit;
	}

quit:

	return( rcode );

}
//------------------------------------------------------------------------------------
int16_t INA_write( uint8_t dev_id, uint16_t wrAddress, char *data, uint8_t length )
{

int16_t rcode = 0;

    rcode =  I2C_write ( fdI2C0, INA_id2busaddr(dev_id), wrAddress, 0x01, data, length );
	if ( rcode == -1 ) {
		// Hubo error: trato de reparar el bus y reintentar la operacion
		// Espero 1s que se termine la fuente de ruido.
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		// Reconfiguro los dispositivos I2C del bus que pueden haberse afectado
		xprintf_P(PSTR("ERROR: INA(%d)_write recovering i2c bus\r\n"), dev_id );
	}

	return( rcode );

}
//------------------------------------------------------------------------------------
