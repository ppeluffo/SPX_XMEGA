/* 
 * File:   xgetc.h
 * Author: pablo
 *
 * Created on August 30, 2023, 10:36 AM
 */

#ifndef XGETC_H
#define	XGETC_H

#ifdef	__cplusplus
extern "C" {
#endif


#include "frtos-io.h"
    
int xgetc( char *c );
int xfgetc( int fd, char *c );

#ifdef	__cplusplus
}
#endif

#endif	/* XGETC_H */

