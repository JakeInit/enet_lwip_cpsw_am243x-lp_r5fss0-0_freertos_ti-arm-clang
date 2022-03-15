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

// Points to function passed in that has a reference to a complete lidar message as input
// Returns nothing
void (*onCompleteLidarMessageCallback)(struct CompleteLidarMessage* message);

int parse(const struct BufferData* bufferData);

void reset();

#endif /* SRC_PARSER_H_ */
