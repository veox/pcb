/*!
 * \file src/fileformats/template.c
 *
 * \brief template file for pluggable file formats.
 *
 * <hr>
 *
 * <h1><b>Copyright.</b></h1>\n
 *
 * PCB, interactive printed circuit board design
 *
 * Copyright (C) 1994,1995,1996,1997,1998,2005,2006 Thomas Nau
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Contact addresses for paper mail and Email:
 * Thomas Nau, Schlehenweg 15, 88471 Baustetten, Germany
 * Thomas.Nau@rz.uni-ulm.de
 *
 * \note How to compile (include paths can differ, depending on your
 *       system):\n
 *
 *       gcc -I<pcbroot>/src -I<pcbroot> -I/usr/include/glib-2.0 -I/usr/lib64/glib-2.0/include -DHAVE_CONFIG_H -fPIC -shared fftemplate.c -o fftemplate.so
 *
 *       - <pcbroot> is the location of *configured* PCB souce tree
 *         (config.h is available).
 *       - copy the .so file into one of plugin directories
 *         (e.g. ~/.pcb/plugins).
 *       - set executable flag.
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h>
#endif

#include "global.h"

#include <dirent.h>
#ifdef HAVE_PWD_H
#include <pwd.h>
#endif
#include <time.h>

#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif

#include <sys/stat.h>

#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif

#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif

#include <stdio.h>

#ifdef HAVE_STDLIB_H
#include <stdlib.h>
#endif

#ifdef HAVE_STRING_H
#include <string.h>
#endif

#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif


#include "buffer.h"
#include "change.h"
#include "create.h"
#include "crosshair.h"
#include "data.h"
#include "edif_parse.h"
#include "error.h"
#include "file.h"
#include "hid.h"
#include "layerflags.h"
#include "misc.h"
#include "mymem.h"
#include "parse_l.h"
#include "pcb-printf.h"
#include "polygon.h"
#include "rats.h"
#include "remove.h"
#include "set.h"
#include "strflags.h"

#ifdef HAVE_LIBDMALLOC
#include <dmalloc.h>
#endif


#define DEMO_FILE_VERSION_IMPLEMENTED 20110603


/*!
 * \brief Function to check file format.
 *
 * Function should return 0, the file provided is of specified format,
 * otherwise non-zero value.
 * This function is optional; if it is not provided, the system tries to
 * load the file.
 * If loading fails, it is supposed that file format is not correct.
 */
int
CheckDemoFile (char *filename)
{
  return 1;
}


/*!
 * \brief Function to check supported version.
 *
 * \return Function should return 0, if supports the provided data
 * structure version (at least minimal one).
 * \return Function should return non-zero value, if it does not support
 * the version of data structures.
 */
int
CheckDemoVersion (unsigned long current, unsigned long minimal)
{
  return (DEMO_FILE_VERSION_IMPLEMENTED >= minimal) ? 0 : 1;
}


/*!
 * \brief Function to save layout to specified file.
 *
 * Functions saves contens of PCBtype structure to specified file.
 * No user interactions are necessary.
 *
 * \return Returns 0, if succeeded, other value indicates error.
 */
int
SaveDemo (PCBType *pcb, char *filename)
{
  FILE *f;

  f = fopen (filename, "w");
  fprintf (f,"Demo PCB Format\n\n");
  fclose (f);

  return 0;
}


/*!
 * \brief Function to load layout from specified file.
 *
 * Functions fills provided PCBtype structure with data
 * No user interactions are necessary.
 *
 * \return Returns 0, if succeeded, other value indicates error.
 */
int
ParseDemo (PCBType *pcb, char *filename)
{
  /*! \todo Follofing function call is placeholder, which creates empty
   * board */

  CreateNewPCBPost (pcb, 1); /* REMOVE ME!!!" */

  return 0;
}


static char *demo_format_list_patterns[] = {"*.pcb", "*.PCB", 0};


/*!
 * Array of file format registration structures:
 * - unique identifiction string
 * - name shown in Open/Save dialogs
 * - pointer to NULL-terminated list of file patterns
 * - mime-type
 * - file format is default format (1)
 * - function to check format version
 * - function to check if file is of acceptable format
 * - function to load layout
 * - function to save layout
 * - function to load element(s) - not implemented yet
 * - function to save element(s) - not implemented yet
 */
static HID_Format demo_format_list[] =
{
  {"dmo","Demo PCB format", demo_format_list_patterns, "application/x-pcb-layout", 0, (void *)CheckDemoVersion, (void *)CheckDemoFile, (void *)ParseDemo, (void *)SaveDemo, 0, 0}
};

REGISTER_FORMATS (demo_format_list)

/*!
 * \brief Plugin registration. Rename it to hid_<basename>_init.
 */
void
hid_fftemplate_init ()
{
  register_demo_format_list ();
}
