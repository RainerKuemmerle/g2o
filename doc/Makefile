all: g2o.pdf

%.pdf:	%.dvi
	dvipdf $<

g2o.dvi: g2o.tex
	latex g2o.tex
	bibtex g2o
	latex g2o.tex
	latex g2o.tex
