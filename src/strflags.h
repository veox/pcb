/*
 *                            COPYRIGHT
 *
 *  PCB, interactive printed circuit board design
 *  Copyright (C) 2005 DJ Delorie
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
 *  DJ Delorie, 334 North Road, Deerfield NH 03037-1110, USA
 *  dj@delorie.com
 *
 */

#ifndef PCB_STRFLAGS_H
#define PCB_STRFLAGS_H

/* The purpose of this interface is to make the file format able to handle more
 * than 32 flags, hide the internal details of flags from the file format, and
 * make it possible to give warnings if we get flags that aren't supported on
 * the associated object type or pcb environment.  */

/* When passed a string, parse it and return an appropriate set of
   flags.  Errors cause error() to be called with a suitable message;
   if error is NULL, errors are ignored.  */
FlagType string_to_flags (const char *flagstring,
			  int (*error) (const char *msg));

/* Given a set of flags for a given object type, return a string which
   can be output to a file.  The returned pointer must not be
   freed.  */
char *flags_to_string (FlagType flags, int object_type);

/* Same as above, but for pcb flags.  */
FlagType string_to_pcbflags (const char *flagstring,
			  int (*error) (const char *msg));
char *pcbflags_to_string (FlagType flags);

// If flagstring is non-NULL, return a new NULL-terminated list of new
// strings, each of which consists of exactly one of the comma-separated
// sub-strings of flagstring, otherwise return an empty list of strings.
// Note that an empty list of strings is also returned for an empty
// flagstring.
//
// Amusingly, flags fields in the datafile can contain things like
// "thermal(0X,1S)", which aren't really flags and don't result in actual
// flag bits being set when encountered, but instead add a thermal elsewhere
// as a side effect.  This function is only concerned with actual flags,
// and ignores these thermal declarations.
char **
string_to_flag_names (char const *flagstring);

// Free a NULL-terminated list of strings (both the strings and the list
// of pointers are freed).
void
free_null_terminated_string_list (char **string_list);

// Ensure (via assertions) that for each flag in flag_names which is mentioned
// in the object flags table, a matching bit the the numeric flags argument
// is set.  This is used to to help ensure parser correctness.  See the
// object_flagbits definitions in strflags.c for the table specifying which
// flags are supported where.
void
ensure_flags_set_for_flag_names (char const *const *flag_names, FlagType flags);

// If any flag_names not supported for Type are found in flags, call log_error
// as appropriate.  See the object_flagbits definitions in strflags.c for
// the table specifying which flags are supported where.
void
warn_about_invalid_object_flags (
    int Type,
    char const *const *flag_names,
    int (*log_warning) (char const *msg) );

#endif /* PCB_STRFLAGS_H */
