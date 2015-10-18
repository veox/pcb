// Implementation of interface described in backtrace.h.

#include <assert.h>
#include <execinfo.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "backtrace.h"

char *
backtrace_with_line_numbers (void)
{
  char executable_name[PATH_MAX + 1];
  ssize_t bytes_read;
  char const *mrba = "/tmp/mrba";   // Most Recent Backtrace Addresses
  char const *mrbt = "/tmp/mrbt";   // Most Recent Backtrace Text
  FILE *btfp;   // Backtrace File FILE Pointer
#define BT_MAX_STACK 142
  void *btrace_array[BT_MAX_STACK];   // Actual addressess
  size_t btrace_size;                 // Number or addresses
  int ii;   //  Index variable
#define ADDR2LINE_COMMAND_MAX_LENGTH 242
  char addr2line_command[ADDR2LINE_COMMAND_MAX_LENGTH];
  int return_code;
  struct stat stat_buf;
  char *result;

  // Use /proc magic to find which binary we are
  bytes_read = readlink ("/proc/self/exe", executable_name, PATH_MAX + 1);
  assert (bytes_read != -1);
  assert (bytes_read <= PATH_MAX);      // Systems don't always honor PATH_MAX
  executable_name[bytes_read] = '\0';   // Readlink doesn't do this for us
  
  // Get the actual backtrace
  btrace_size = backtrace (btrace_array, BT_MAX_STACK);
  assert (btrace_size < BT_MAX_STACK);

  // Print the addresses to the address file
  btfp = fopen (mrba, "w");
  assert (btfp != NULL);
  for ( ii = 0 ; ii < btrace_size ; ii++ ) {
    fprintf (btfp, "%p\n", btrace_array[ii]);
  } 
  return_code = fclose (btfp);
  assert (return_code == 0);

  // Run addr2line to convert addresses to show func, file, line
  sprintf (
      addr2line_command,
      "cat %s | addr2line --exe %s -f -i >%s",
      mrba,
      executable_name,
      mrbt );
  return_code = system (addr2line_command);
  assert (return_code == 0);

  // Get the size of the result
  return_code = stat (mrbt, &stat_buf);
  assert (return_code == 0);
  
  // Allocate storage for result
  result = malloc (stat_buf.st_size + 1);   // +1 for trailing null byte
  assert (result != NULL);

  // Read the func, file, line form back in
  btfp = fopen (mrbt, "r");
  assert (btfp != NULL);
  bytes_read = fread (result, 1, stat_buf.st_size, btfp);
  assert (bytes_read == stat_buf.st_size);
  result[stat_buf.st_size] = '\0';

  return result;
}
