all: g2o.pdf

%.pdf:	%.dvi
	dvipdf $<

%.dvi: bbs %.tex
	latex g2o.tex
	bibtex g2o
	latex g2o.tex
	latex g2o.tex

bbs:
	cd pics && ebb *.png *.eps
 
