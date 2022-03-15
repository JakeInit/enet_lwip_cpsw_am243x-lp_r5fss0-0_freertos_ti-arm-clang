/*
 * Bool.h
 *
 *  Created on: Mar 11, 2022
 *      Author: root
 */

#ifndef SRC_BOOL_H_
#define SRC_BOOL_H_

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#ifndef bool
typedef uint8_t      bool;       /* boolean */
#endif

/* Define TRUE/FALSE to go with bool */
#ifndef true

#define true        ((bool) 0xFF)
#define false       ((bool) 0x00)

#endif /* TRUE */



#endif /* SRC_BOOL_H_ */
