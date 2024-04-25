/*
 * control.h
 *
 *  Created on: Apr 23, 2024
 *      Author: ma
 */

#ifndef INC_PROFILE_H_
#define INC_PROFILE_H_

#include <stdint.h>
#include <assert.h>

typedef struct {
	uint32_t pgcfg_a[4];
	uint32_t pgcfg_b[4];
	uint32_t duration_a_sec;	// only meaningful when mode is 2
	uint32_t duration_b_sec;	// only meaningful when mode is 2
} profile_t;

// suppress the IDE syntax error (yellow mark) by
// Project --> Properties --> C/C++ General --> Preprocessor Include Paths, Macros etc.
// On the Entries tab, select CDT User Setting Entries, then click Add
//
//   _Static_assert(a,b)
//
static_assert(sizeof(profile_t) == 40, "profile_t size not 40");

void print_profile(int index);
profile_t get_profile(int index);
void set_profile(int index,
				uint32_t pgcfg_a[4],
				uint32_t *duration_a_sec,
				uint32_t pgcfg_b[4],
				uint32_t *duration_b_sec);

#endif /* INC_PROFILE_H_ */
