
#include "valves.h"


//------------------------------------------------------------------------------
void valve_A_open(void)
{
    /*
     Genero un pulso A+A- para abrir la valvula A
     */
    DRV8814_awake();
    DRV8814_pulse_Amas_Amenos(10);
    DRV8814_sleep();
}
//------------------------------------------------------------------------------
void valve_A_close(void)
{
    /*
     Genero un pulso A-A+ para cerrar la valvula A
     */
    DRV8814_awake();
    DRV8814_pulse_Amenos_Amas(10);
    DRV8814_sleep();
}
//------------------------------------------------------------------------------
void valve_B_open(void)
{
    /*
     Genero un pulso B+B- para abrir la valvula B
     */
    DRV8814_awake();
    DRV8814_pulse_Bmas_Bmenos(10);
    DRV8814_sleep();
}
//------------------------------------------------------------------------------
void valve_B_close(void)
{
    /*
     Genero un pulso B-B+ para cerrar la valvula B
     */
    DRV8814_awake();
    DRV8814_pulse_Bmenos_Bmas(10);
    DRV8814_sleep();
}
//------------------------------------------------------------------------------
bool valve_test( char *s_valveId, char *s_action)
{
    
    if (!strcmp_P( strupr(s_valveId), PSTR("A"))) {
        
        if (!strcmp_P( strupr(s_action), PSTR("OPEN"))) {
            valve_A_open();
            return(true);
        } else if (!strcmp_P( strupr(s_action), PSTR("CLOSE"))) {
            valve_A_close();
            return(true);
        } else {
            return(false);  
        }
        
    } else if (!strcmp_P( strupr(s_valveId), PSTR("B"))) {
        
        if (!strcmp_P( strupr(s_action), PSTR("OPEN"))) {
            valve_B_open();
            return(true);
        } else if (!strcmp_P( strupr(s_action), PSTR("CLOSE"))) {
            valve_B_close();
            return(true);
        } else {
            return(false);  
        } 
    }
    
    return(false);
}
//------------------------------------------------------------------------------
