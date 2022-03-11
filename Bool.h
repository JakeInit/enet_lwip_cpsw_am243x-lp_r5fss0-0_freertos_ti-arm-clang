/*
 * Bool.h
 *
 *  Created on: Mar 11, 2022
 *      Author: root
 */

#ifndef BOOL_H_
#define BOOL_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#ifndef Bool
typedef uint8_t      Bool;       /* boolean */
#endif

/* Define TRUE/FALSE to go with Bool */
#ifndef TRUE

#define TRUE        ((Bool) 0xFF)
#define FALSE       ((Bool) 0x00)

#endif /* TRUE */



#endif /* BOOL_H_ */
