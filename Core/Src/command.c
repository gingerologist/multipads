/*
 * command.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ma
 */
/* Includes ------------------------------------------------------------------*/
#include <profile.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "queue.h"

#include "command.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static char long_buf[256];

/* Private function prototypes -----------------------------------------------*/
static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context);
static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context);

/* Private user code ---------------------------------------------------------*/

static void CLI_CMD_List(EmbeddedCli *cli, char *args, void *context)
{
	for (int i = 0; i < 9; i++)
	{
		print_profile(i);
	}
}

CliCommandBinding cli_cmd_list_binding = {
		"list",
		"Print profile 1 to 9",
		false,
		NULL,
		CLI_CMD_List
};

static bool cfg_str_is_valid(const char* str)
{
	if (strlen(str) != 9) return false;
	for (int i = 0; i < 9; i++)
	{
		char c = str[i];
		if (c != '0' && c != '1' && c != '2')
		{
			return false;
		}
	}
	return true;
}

static bool parse_config(const char* str, uint32_t *config)
{
	if (!cfg_str_is_valid(str))
	{
		return false;
	}

	if (config != NULL)
	{
		*config = 0;
		for (int i = 0; i < 9; i++)
		{
			switch(str[i])
			{
			case '1':
				*config |= 0x01 << (i * 2);
				break;
			case '2':
				*config |= 0x02 << (i * 2);
				break;
			default:
				break;
			}
		}
	}

	return true;
}

// strictly 0 to 9, no leading zero, not larger than 3600 * 1000
static bool nat_is_valid(const char* str)
{
	size_t len = strlen(str);
	if (len == 0)
	{
		return false;
	}

	for (int i = 0; i < len; i++)
	{
		if (str[i] < '0' || str[i] > '9') return false;
	}

	if (str[0] == '0' && len > 1) return false;

	return true;
}

static void CLI_CMD_Define(EmbeddedCli *cli, char *args, void *context)
{
	uint8_t count = embeddedCliGetTokenCount(args);

	if (count != 6)
	{
		print_line("error: define command requires exact 6 arguments.");
		return;
	}

	const char *p = embeddedCliGetToken(args, 1);
	if (strlen(p) != 2 || p[0] < '1' || p[0] > '9' || (p[1] != 'a' && p[1] != 'b'))
	{
		print_line("error: first argument is invalid.");
		return;
	}

	int profile_index = p[0] - '1';
	int profile_phase = (p[1] == 'a') ? 0 : 1;
	uint32_t config[4];

	for (int j = 2; j < 6; j++)
	{
		p = embeddedCliGetToken(args, j);
		if (!parse_config(p, &config[j - 2]))
		{
			snprintf(long_buf, 256, "error: argument %d \"%s\" is invalid.", j, p);
			print_line(long_buf);
			return;
		}
	}

	const int max_str_len = strlen("3600");
	const uint32_t max_dur_value = 3600;

	p = embeddedCliGetToken(args, 6);
	if (!nat_is_valid(p))
	{
		print_line("error: duration is not a valid number.");
		return;
	}

	if (strlen(p) > max_str_len)
	{
		print_line("error: duration value is too large (max 3600)");
		return;
	}

	long int nat = strtol(p, NULL, 10);
	if (nat > max_dur_value)
	{
		print_line("error: duration value is too large (max 3600)");
		return;
	}

	uint32_t duration = (uint32_t)nat;

	if (profile_phase == 0)
	{
		set_profile(profile_index, config, &duration, NULL, NULL);
	}
	else
	{
		set_profile(profile_index, NULL, NULL, config, &duration);
	}

	print_profile(profile_index);
}

CliCommandBinding cli_cmd_define_binding = {
		"define",
		"Define a profile",
		true,
		NULL,
		CLI_CMD_Define
};



