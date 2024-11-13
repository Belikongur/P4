#ifndef GUARD_UTILITY_H
#define GUARD_UTILITY_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "lownet.h"

// Usage: HEXDUMP(x)
// Pre:   x is a valid operand for the addressof operator (&)
// Post:  A hexadecimal representation of x has been written to
//        the serial port
#define HEXDUMP(x) do {\
	for (int i = 0; i < sizeof x; ++i)\
	printf("%02x%c", ((uint8_t*)&x)[i], (i + 1) % 16 == 0 ? '\n' : ' ');\
	putchar('\n');\
	} while (0)

int util_printable(char c);

// Usage: min(A, B)
// Pre:   None, other than those imposed by the type system
// Value: The smaller of A and B
uint8_t min(uint8_t a, uint8_t b);

uint32_t hex_to_dec(const char* hex_digits);

// Usage: time_to_milliseconds(TIME)
// Pre:   TIME != NULL
// Value: The number of milliseconds represented by TIME
uint32_t time_to_milliseconds(const lownet_time_t* time);

// Usage: time_from_milliseconds(MILLIS)
// Pre:   None
// Value: The time value represented by MILLIS
lownet_time_t time_from_milliseconds(uint32_t millis);

// Usage: compare_time(LHS, RHS)
// Pre:   LSH != NULL, RHS != NULL
// Value: -1 if LSH is smaller than RHS
//         0 if LSH is equal to RHS
//         1 if LSH is greater than RHS
int compare_time(const lownet_time_t* lhs, const lownet_time_t* rhs);

// Usage: time_diff(A, B)
// Pre:   A != NULL, B != NULL,
//        B must be greater than A as defined by compare_time(A, B).
// Value: The difference between A and B
lownet_time_t time_diff(const lownet_time_t* a, const lownet_time_t* b);

// Usage: buffers_compare(A, B, SIZE)
// Pre:   A != NULL, B != NULL
//        A and B are buffers of size SIZE
// Value: -1 if A is less than B,
//         0 if A and B are equal,
//         1 if A is greater than B
int buffers_compare(const uint8_t* a, const uint8_t* b, size_t size);

// Usage: buffers_equal(A, B, SIZE)
// Pre:   A != NULL, B != NULL
//        A and B are buffers of size SIZE
// Value: true if A and B are equal, false otherwise
bool buffers_equal(const uint8_t* a, const uint8_t* b, size_t size);

// uint32 + '.' + uint32 + 's'
#define TIME_WIDTH (11 + 1 + 11 + 1)

// Usage: format_time(BUFFER, TIME)
// Pre:   BUFFER != NULL, TIME != NULL
//        sizeof BUFFER >= TIME_WIDTH
// Post:  TIME has been formatted into buffer
// Value: The number of characters written to BUFFER
int format_time(char* buffer, lownet_time_t* time);

#define ID_WIDTH 4
// Usage: format_id(BUFFER, ID)
// Pre:   BUFFER != NULL, sizeof BUFFER >= ID_WIDTH
// Post:  ID has been formatted into buffer
// Value: The number of characters written to BUFFER
int format_id(char* buffer, uint8_t id);

#endif
