#!/usr/bin/env bash
#
#
# Version 0.1.6
#
# This script was writen for running LaTeX multiply times. It'll check whenever it's
# indicated to rerun Latex.
#
# Extra features are ps, pdf, and HTML output from Latex source.
# You need latex2html and ps2pdf to do that so.
#
# Usage.
#
# cd to the directory, where you have your LaTeX file.
#
# run_latex.sh [-p|P|h] filename
#
# -p is for postscript
# -P is for pdf
# -h for html
#
# filename is the proper filename of the tex file.
#
# Without switch it'll generate dvi only.
#
# Return value
#
# 0 on succes, 
# 1 otherwise.
#
# Send comments, and bug reports to levente.kovacs@cern.ch or
# lekovacs@interware.hu

maxcount=5
LATEX=latex
DVIPS="dvips -Ppdf -G0"
PS2PDF="ps2pdf -sPAPERSIZE=a4 -dMaxSubsetPct=100 -dCompatibilityLevel=1.2 -dSubsetFonts=true -dEmbedAllFonts=true"
LATEX2HTML="latex2html -split 4 -show_section_numbers -mkdir -dir html"
MAKEINDEX=makeindex

if [ $# = 0 ]; then
	echo "run_latex: too few arguments"
	exit 1
elif [ $# = 1 ]; then
	file=$1
elif [ $# = 2 ]; then
	file=$2
else
	echo "run_latex: too many arguments"
	exit 1
fi

if [ ! -e $file ]; then
	echo "run_latex: $file: no such file exists"
	exit 1
fi

filebase=${file%.*}

for ((i=1; i <= $maxcount; i++)); do
	if [ -e $filebase.aux ]; then
		cp $filebase.aux $filebase.aux.tmp
	fi

	#See if there is an index file	
	if [ -e $filebase.idx ]; then
		cp $filebase.idx $filebase.idx.tmp
		$MAKEINDEX $filebase.idx
	fi

	$LATEX $filebase.tex
	retval=$?

	if [ $retval = 0 ]; then
		if test ! -e $filebase.aux.tmp ; then
			continue
		fi
		if ! diff $filebase.aux $filebase.aux.tmp ; then
			continue
		fi
		grep "No file" $filebase.log
		if [ $? = 0 ]; then
			continue
		fi

		if [ -e $filebase.idx ]; then
			if ! diff $filebase.idx $filebase.idx.tmp ; then
			continue
			fi
		fi

		while getopts ":pPh" Options
			do
				case $Options in
					p ) $DVIPS $filebase.dvi;;
					P ) $DVIPS $filebase.dvi; $PS2PDF $filebase.ps;;
					h ) $LATEX2HTML $filebase.tex;;
					* ) break;;
				esac
			done
		echo "run_latex: total latex run: $i"
#This is ugly.... should have make functions... laizy I am...
		if test -e $filebase.aux.tmp ; then
			rm $filebase.aux.tmp
		fi
		if [ -e $filebase.idx.tmp ]; then
			rm $filebase.idx.tmp
		fi
			exit 0
	else
		echo "run_latex: there were errors running latex. Error code: $retval"
		if test -e $filebase.aux.tmp ; then
			rm $filebase.aux.tmp
		fi
		if [ -e $filebase.idx.tmp ]; then
			rm $filebase.idx.tmp
		fi
		exit 1
	fi
done
echo "run_latex: latex could not work some error out. Giving up. Total latex run: $i"
if test -e $filebase.aux.tmp ; then
	rm $filebase.aux.tmp
fi
if [ -e $filebase.idx.tmp ]; then
	rm $filebase.idx.tmp
fi
exit 1

