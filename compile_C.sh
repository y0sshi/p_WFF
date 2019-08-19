#!/bin/zsh

TARGET="./initialize"

rm -f g.log
gcc ${TARGET}.c -o $TARGET -lm

if [ $# -eq 3 ] ; then
  $TARGET $1 $2 $3 > g.log
  $TARGET $1 $2 $3
else
  $TARGET 
fi
