/*
 * parser.h
 *
 *  Created on: Mar 11, 2022
 *      Author: Jacob Morgan
 */

#ifndef SRC_PARSER_H_
#define SRC_PARSER_H_

#include <stdio.h>
#include <stdarg.h>

#include "Shared.h"

void initParser(void (*message_callback)(struct CompleteLidarMessage*));

uint32_t parse(const struct BufferData* bufferData);

void resetParser();

#endif /* SRC_PARSER_H_ */
