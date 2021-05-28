// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// util.h: misc utilities prototypes
//
// Copyright (c) 2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef DUMP1090_UTIL_H
#define DUMP1090_UTIL_H

#include <stdint.h>

/* Returns system time in milliseconds */
uint64_t mstime(void);

/* Returns the time for the current message we're dealing with */
extern uint64_t _messageNow;
static inline uint64_t messageNow() {
    return _messageNow;
}

/* Returns the time elapsed, in nanoseconds, from t1 to t2,
 * where t1 and t2 are 12MHz counters.
 */
int64_t receiveclock_ns_elapsed(uint64_t t1, uint64_t t2);

/* Same, in milliseconds */
int64_t receiveclock_ms_elapsed(uint64_t t1, uint64_t t2);

/* Normalize the value in ts so that ts->nsec lies in
 * [0,999999999]
 */
struct timespec;
void normalize_timespec(struct timespec *ts);

/* Find the absolute system time that is `timeout_ms` milliseconds in the future, and store that in *ts */
void get_deadline(uint32_t timeout_ms, struct timespec *ts);

/* set current thread name, if supported */
void set_thread_name(const char *name);

#endif
