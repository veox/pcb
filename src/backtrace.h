// Provide a backtrace with line numbers, and assertions that print this
// backtrace when they fail.

#include <stdlib.h>

// Return a new multi-line string containing a backtrace of the call point,
// with line numbers.  Caveats:
//
//   * This uses the GNU libc backtrace() function, all caveats related to
//     that function apply here as well.
//
//   * It uses malloc(), so cannot reliably be used when malloc() might fail
//     (e.g. after memory corruption).
//
//   * It probably isn't reentrant.
//
char *
backtrace_with_line_numbers (void);

// FIXME: so how does libc assert get the program name? test all this

// Like assert(), but prints a full backtrace.  All caveats of
// backtrace_with_line_numbers() apply.
#ifdef NDEBUG
#  define ASSERT_BT(CONDITION)
#else
#  define ASSERT_BT(CONDITION)                                             \
    do {                                                                   \
      if ( __builtin_expect (!(CONDITION), 0) ) {                          \
        fprintf (                                                          \
            stderr, "Assertion ` " #CONDITION "' failed.  Backtrace:\n%s", \
            backtrace_with_line_numbers() );                               \
        abort ();                                                          \
      }                                                                    \
    } while ( 0 )
#endif
