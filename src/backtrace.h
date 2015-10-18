// Provide a backtrace with line numbers, and an assert()-like routine that
// prints a backtrace on failure.

#include <stdlib.h>

// Return a new multi-line string containing a backtrace of the call point,
// with line numbers.  Caveats:
//
//   * This uses the GNU libc backtrace() function, all caveats related to
//     that function apply here as well.  In particular, the GCC -g compile
//     time and -rdynamic link-time options are required, and line numbers
//     might be wrong if the -O0 (disable all optimization) isn't used.
//     There might be other issues as well.  To avoid these problems build
//     pcb like this:
//
//       make CFLAGS='-Wall -g -O0'
//
//   * FIXME: as of this writing, --disable-nls is also required at
//     ./configure-time to avoid bugs in NLS that trigger with -O0
//     
//   * It uses malloc(), so cannot reliably be used when malloc() might fail
//     (e.g. after memory corruption).
//
//   * It probably isn't reentrant.
//
char *
backtrace_with_line_numbers (void);

// FIXME: so how does libc assert get the program name? test all this

// Like assert(), but prints a full backtrace (if it does anything).
// All caveats of backtrace_with_line_numbers() apply.
#ifdef NDEBUG
#  define ASSERT_BT(CONDITION)
#else
#  define ASSERT_BT(CONDITION)                                             \
    do {                                                                   \
      if ( __builtin_expect (!(CONDITION), 0) ) {                          \
        fprintf (                                                          \
            stderr,                                                        \
            "Assertion ` " #CONDITION "' failed.  Backtrace:\n%s",         \
            backtrace_with_line_numbers() );                               \
        abort ();                                                          \
      }                                                                    \
    } while ( 0 )
#endif
