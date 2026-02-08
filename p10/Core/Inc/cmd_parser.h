/*
 * cmd_parser.h
 *
 *  Created on: Feb 8, 2026
 *      Author: Nuria
 */

#ifndef INC_CMD_PARSER_H_
#define INC_CMD_PARSER_H_

#pragma once
#include "common_def.h"
#include <stdint.h>

typedef void (*CmdEnqueueFn)(const SystemCommandMsg_t *cmd);

/**
 * Parsea una línea de comando (ASCII) y genera un SystemCommandMsg_t.
 * Devuelve 1 si comando válido, 0 si no reconocido / error de formato.
 */
int Cmd_ParseLine(const char *line, SystemCommandSource_t src, SystemCommandMsg_t *out);




#endif /* INC_CMD_PARSER_H_ */
