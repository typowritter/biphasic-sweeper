#!/bin/bash

tmp="tmp"
flags="-output-directory=$tmp -shell-escape"

if [[ "$#" == "0" ]];then
    echo "No input. Stop."
    exit
elif [[ "$1" == "clean" ]]; then
    rm -rf $tmp
    exit
else
    filename="$1"
fi

filename=${filename/.tex}

mkdir -p $tmp

if [[ -f "$tmp/$filename.bbl" ]];then
    xelatex $flags $filename || exit
else
    export TEXINPUTS=".//:$TEXINPUTS"
    export BIBINPUTS=".//:$BIBINPUTS"
    export BSTINPUTS=".//:$BSTINPUTS"

    xelatex $flags $filename || exit
    bibtex ./$tmp/$filename
    xelatex $flags $filename || exit
    xelatex $flags $filename || exit
fi

System_Name=`uname`
if [[ $System_Name == "Linux" ]]; then
    PDFviewer="evince"
elif [[ $System_Name == "Darwin" ]]; then
    PDFviewer="open"
else
    PDFviewer="open"
fi

mv -f ./$tmp/"$filename".pdf ./"$filename".pdf || exit

$PDFviewer ./"$filename".pdf &