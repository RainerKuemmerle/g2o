#!/bin/sh

# wrapper script to generate the parser by flex / bison
# Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard

flex -o flex_scanner.cpp -i scanner.l

cp /usr/include/FlexLexer.h .

bison -o bison_parser.cpp --defines=bison_parser.h parser.yy

for f in *.cpp; do
  clang-format -i "$f"
done

for f in *.h; do
  clang-format -i "$f"
done

for f in *.hh; do
  clang-format -i "$f"
done
