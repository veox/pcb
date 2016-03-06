/* Provide a backtrace with line numbers, and an assert()-like routine that
 * prints a backtrace on failure.  */

/*
 *                            COPYRIGHT
 *
 *  PCB, interactive printed circuit board design
 *  Copyright (C) 1994,1995,1996 Thomas Nau
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Contact addresses for paper mail and Email:
 *  Thomas Nau, Schlehenweg 15, 88471 Baustetten, Germany
 *  Thomas.Nau@rz.uni-ulm.de
 *
 */

#include <errno.h>
#include <stdlib.h>

#ifndef NDEBUG

/* Return a new multi-line string containing a backtrace of the call point,
 * with line numbers.  Caveats:
 *
 *   * This uses the GNU libc backtrace() function, all caveats related to
 *     that function apply here as well.  In particular, the GCC -g compile
 *     time and -rdynamic link-time options are required, and line numbers
 *     might be wrong if the -O0 (disable all optimization) isn't used.
 *     There might be other issues as well.  To avoid these problems build
 *     pcb like this:
 *
 *       make CFLAGS='-Wall -g -O0'
 *
 *   * FIXME: as of this writing, --disable-nls is also required at
 *     ./configure-time to avoid bugs in NLS that trigger with -O0.
 *
 *   * Flex and Bison still seem to cause slight line number error.
 *     
 *   * It uses malloc(), so cannot reliably be used when malloc() might fail
 *     (e.g. after memory corruption).
 *
 *   * It probably isn't reentrant.
 * */
char *
backtrace_with_line_numbers (void);

/* Like assert(), but prints a full backtrace (if it does anything).
 * All caveats of backtrace_with_line_numbers() apply.  */
#  define ASSERT_BT(COND)                                                    \
    do {                                                                     \
      if ( __builtin_expect (!(COND), 0) ) {                                 \
        fprintf (                                                            \
            stderr,                                                          \
            "%s: %s:%u: %s: Assertion ` " #COND "' failed.  Backtrace:\n%s", \
            program_invocation_short_name,                                   \
            __FILE__,                                                        \
            __LINE__,                                                        \
            __func__,                                                        \
            backtrace_with_line_numbers() );                                 \
        abort ();                                                            \
      }                                                                      \
    } while ( 0 )

#else

#  define ASSERT_BT(COND)

#endif
