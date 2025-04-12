all: g2o.pdf

pics/%.svg: pics/%.fig
	fig2dev -L svg $< $@
pics/%.pdf: pics/%.fig
	fig2dev -L pdf $< $@

g2o.pdf: g2o.tex pics/classes.pdf pics/hgraph.pdf pics/slam.pdf
	pdflatex g2o.tex
	bibtex g2o
	pdflatex g2o.tex
	pdflatex g2o.tex

clean:
	rm -rf pics/*.svg pics/*.pdf *.log
.PHONY: clean
