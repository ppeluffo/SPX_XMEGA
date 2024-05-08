/*
 * l_iopines.h
 *
 *  Created on: 8 dic. 2018
 *      Author: pablo
 */

#ifndef SRC_SPX_LIBS_L_IOPINES_H_
#define SRC_SPX_LIBS_L_IOPINES_H_

#include <avr/io.h>

//------------------------------------------------------------------------------------
// TWI PE0(SDA)/PE1(SCL)
#define SCL_PORT        PORTE
#define SCL             1
#define SCL_PIN_bm      PIN1_bm
#define SCL_PIN_bp      PIN1_bp

#define CONFIG_SCL()        SCL_PORT.DIR |= SCL_PIN_bm;
#define SET_SCL()        ( SCL_PORT.OUT |= SCL_PIN_bm )
#define CLEAR_SCL()		 ( SCL_PORT.OUT &= ~SCL_PIN_bm )

//------------------------------------------------------------------------------
// ANALOG_IN: SENSOR VCC CONTROL

// Salida de prender/apagar sensores 4-20
#define VSENSORS420_PORT         PORTA
#define VSENSORS420              1
#define VSENSORS420_PIN_bm       PIN1_bm
#define VSENSORS420_PIN_bp       PIN1_bp

#define CONFIG_VSENSORS420()    ( VSENSORS420_PORT.DIR |= VSENSORS420_PIN_bm )	
#define SET_VSENSORS420()       ( VSENSORS420_PORT.OUT |= VSENSORS420_PIN_bm )
#define CLEAR_VSENSORS420()     ( VSENSORS420_PORT.OUT &= ~VSENSORS420_PIN_bm )

void VSENSORS420_init(void);
//------------------------------------------------------------------------------------
// RELES

#define RELE_K1_PORT         PORTD
#define RELE_K1              5
#define RELE_K1_PIN_bm       PIN4_bm
#define RELE_K1_PIN_bp       PIN4_bp

#define CONFIG_RELE_K1()    ( RELE_K1_PORT.DIR |= RELE_K1_PIN_bm )
#define SET_RELE_K1()       ( RELE_K1_PORT.OUT |= RELE_K1_PIN_bm )
#define CLEAR_RELE_K1()     ( RELE_K1_PORT.OUT &= ~RELE_K1_PIN_bm )
#define TOGGLE_RELE_K1()    ( RELE_K1_PORT.OUT ^= 1UL << RELE_K1_PIN_bp);
    
#define RELE_K1_OPEN()      SET_RELE_K1() 
#define RELE_K1_CLOSE()     CLEAR_RELE_K1()

#define MODEM_PRENDER()     SET_RELE_K1() 
#define MODEM_APAGAR()      CLEAR_RELE_K1()

void RELE_K1_init(void);
    
#define RELE_K2_PORT         PORTD
#define RELE_K2              4 
#define RELE_K2_PIN_bm       PIN5_bm
#define RELE_K2_PIN_bp       PIN5_bp

#define CONFIG_RELE_K2()    ( RELE_K2_PORT.DIR |= RELE_K2_PIN_bm )
#define SET_RELE_K2()       ( RELE_K2_PORT.OUT |= RELE_K2_PIN_bm )
#define CLEAR_RELE_K2()     ( RELE_K2_PORT.OUT &= ~RELE_K2_PIN_bm )
#define TOGGLE_RELE_K2()    ( RELE_K2_PORT.OUT ^= 1UL << RELE_K2_PIN_bp);
    
#define RELE_K2_OPEN()      SET_RELE_K2() 
#define RELE_K2_CLOSE()     CLEAR_RELE_K2()

void RELE_K2_init(void);

//------------------------------------------------------------------------------
// RTS485A

#define RTS_RS485A_PORT         PORTE
#define RTS_RS485A              5
#define RTS_RS485A_PIN_bm       PIN5_bm
#define RTS_RS485A_PIN_bp       PIN5_bp

#define CONFIG_RTS_485A()       RTS_RS485A_PORT.DIR |= RTS_RS485A_PIN_bm;
#define SET_RTS_RS485A()        ( RTS_RS485A_PORT.OUT |= RTS_RS485A_PIN_bm )
#define CLEAR_RTS_RS485A()      ( RTS_RS485A_PORT.OUT &= ~RTS_RS485A_PIN_bm )

//------------------------------------------------------------------------------
// DRV8814:

#define DRV8814_PWR_PORT        PORTA
#define DRV8814_PWR             3
#define DRV8814_PWR_PIN_bm      PIN3_bm
#define DRV8814_PWR_PIN_bp      PIN3_bp
#define SET_DRV8814_PWR         ( DRV8814_PWR_PORT.OUT |= DRV8814_PWR_PIN_bm )
#define CLEAR_DRV8814_PWR       ( DRV8814_PWR_PORT.OUT &= ~DRV8814_PWR_PIN_bm )
#define CONFIG_DRV8814_PWR      ( DRV8814_PWR_PORT.DIR |= DRV8814_PWR_PIN_bm )

#define DRV8814_RESET_PORT      PORTA      
#define DRV8814_RESET           4
#define DRV8814_RESET_PIN_bm    PIN4_bm
#define DRV8814_RESET_PIN_bp    PIN4_bp
#define SET_DRV8814_RESET       ( DRV8814_RESET_PORT.OUT |= DRV8814_RESET_PIN_bm )
#define CLEAR_DRV8814_RESET     ( DRV8814_RESET_PORT.OUT &= ~DRV8814_RESET_PIN_bm )
#define CONFIG_DRV8814_RESET    ( DRV8814_RESET_PORT.DIR |= DRV8814_RESET_PIN_bm )

#define DRV8814_SLEEP_PORT      PORTA     
#define DRV8814_SLEEP           5
#define DRV8814_SLEEP_PIN_bm    PIN5_bm
#define DRV8814_SLEEP_PIN_bp    PIN5_bp
#define SET_DRV8814_SLEEP       ( DRV8814_SLEEP_PORT.OUT |= DRV8814_SLEEP_PIN_bm )
#define CLEAR_DRV8814_SLEEP     ( DRV8814_SLEEP_PORT.OUT &= ~DRV8814_SLEEP_PIN_bm )
#define CONFIG_DRV8814_SLEEP    ( DRV8814_SLEEP_PORT.DIR |= DRV8814_SLEEP_PIN_bm )

#define DRV8814_AEN_PORT      PORTA    
#define DRV8814_AEN           7
#define DRV8814_AEN_PIN_bm    PIN7_bm
#define DRV8814_AEN_PIN_bp    PIN7_bp
#define SET_DRV8814_AEN       ( DRV8814_AEN_PORT.OUT |= DRV8814_AEN_PIN_bm )
#define CLEAR_DRV8814_AEN     ( DRV8814_AEN_PORT.OUT &= ~DRV8814_AEN_PIN_bm )
#define CONFIG_DRV8814_AEN    ( DRV8814_AEN_PORT.DIR |= DRV8814_AEN_PIN_bm )

#define DRV8814_BEN_PORT      PORTB      
#define DRV8814_BEN           0
#define DRV8814_BEN_PIN_bm    PIN0_bm
#define DRV8814_BEN_PIN_bp    PIN0_bp
#define SET_DRV8814_BEN       ( DRV8814_BEN_PORT.OUT |= DRV8814_BEN_PIN_bm )
#define CLEAR_DRV8814_BEN     ( DRV8814_BEN_PORT.OUT &= ~DRV8814_BEN_PIN_bm )
#define CONFIG_DRV8814_BEN    ( DRV8814_BEN_PORT.DIR |= DRV8814_BEN_PIN_bm )

#define DRV8814_APH_PORT      PORTA      
#define DRV8814_APH           6
#define DRV8814_APH_PIN_bm    PIN6_bm
#define DRV8814_APH_PIN_bp    PIN6_bp
#define SET_DRV8814_APH       ( DRV8814_APH_PORT.OUT |= DRV8814_APH_PIN_bm )
#define CLEAR_DRV8814_APH     ( DRV8814_APH_PORT.OUT &= ~DRV8814_APH_PIN_bm )
#define CONFIG_DRV8814_APH    ( DRV8814_APH_PORT.DIR |= DRV8814_APH_PIN_bm )

#define DRV8814_BPH_PORT      PORTB      
#define DRV8814_BPH           1
#define DRV8814_BPH_PIN_bm    PIN1_bm
#define DRV8814_BPH_PIN_bp    PIN1_bp
#define SET_DRV8814_BPH       ( DRV8814_BPH_PORT.OUT |= DRV8814_BPH_PIN_bm )
#define CLEAR_DRV8814_BPH     ( DRV8814_BPH_PORT.OUT &= ~DRV8814_BPH_PIN_bm )
#define CONFIG_DRV8814_BPH    ( DRV8814_BPH_PORT.DIR |= DRV8814_BPH_PIN_bm )

//------------------------------------------------------------------------------
// Los pines de FinCarrera son entradas
// ENTRADAS DIGITALES ( SOLO EN SPX_5CH ya que el otro usa el MCP )
// Solo niveles logicos

// ROJO / NEGRO
#define FC1_PORT      PORTA    
#define FC1           0
#define FC1_PIN_bm    PIN0_bm
#define FC1_PIN_bp    PIN0_bp
#define CONFIG_FC1()    ( FC1_PORT.DIR &= ~FC1_PIN_bm )

// AZUL / BLANCO
#define FC2_PORT      PORTB     
#define FC2           7
#define FC2_PIN_bm    PIN7_bm
#define FC2_PIN_bp    PIN7_bp
#define CONFIG_FC2()    ( FC2_PORT.DIR &= ~FC2_PIN_bm )

uint8_t FC1_read(void);
uint8_t FC2_read(void);
#define FC_alta_read() FC1_read()
#define FC_baja_read() FC2_read()
void FCx_init(void);

//------------------------------------------------------------------------------
// SOLO USAMOS EL CONTADOR 0 ( Negro / Azul )!!
#define CNT0_PORT	PORTA
#define CNT0_PIN    2
#define CNT0_PIN_bm	PIN2_bm
#define CNT0_PIN_bp	PIN2_bp
    
// EL CONTADOR 1 NO SE USA PORQUE LOS PINES DEL CONECTOR SE USAN PARA
// ENTRADAS 4-20 DE CAUDALIMETROS
#define CNT1_PORT	PORTB
#define CNT1_PIN    2
#define CNT1_PIN_bm	PIN2_bm
#define CNT1_PIN_bp	PIN2_bp
    
// Los CNTx son inputs
#define CNT0_CONFIG()    ( CNT0_PORT.DIR &= ~CNT0_PIN_bm )
#define CNT1_CONFIG()    ( CNT1_PORT.DIR &= ~CNT1_PIN_bm )
//------------------------------------------------------------------------------


#endif /* SRC_SPX_LIBS_L_IOPINES_H_ */
