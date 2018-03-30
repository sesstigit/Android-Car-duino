/**
*    This file is part of Autodrive.
*
*    Autodrive is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    Autodrive is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with Autodrive.  If not, see <http://www.gnu.org/licenses/>.
**/

#pragma once
#ifndef ANDROIDCARDUINO_AUTODRIVE_GETOPT_H_
#define ANDROIDCARDUINO_AUTODRIVE_GETOPT_H_

#include <string.h>
#include <stdio.h>

extern int opterr;             /* if error message should be printed */
extern int optind;             /* index into parent argv vector */
extern int optopt;                 /* character checked for validity */
extern int optreset;               /* reset getopt */
extern char    *optarg;                /* argument associated with option */

#define BADCH   (int)'?'
#define BADARG  (int)':'
#define EMSG    ""

int getopt(int nargc, char * const nargv[], const char *ostr);  //no getopt in Microsoft Visual Studio

#endif // ANDROIDCARDUINO_AUTODRIVE_GETOPT_H_