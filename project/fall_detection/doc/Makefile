DOC ?= template_RNS
SOURCES = Makefile $(DOC).tex

.PHONY: simple
simple :
	@echo "Simple build ..."
	make DOC=$(DOC) tex
	make DOC=$(DOC) tex
	make DOC=$(DOC) clean

.PHONY: full
full: 
	@echo "Full build with bibtex ..."
	make DOC=$(DOC) tex
	make DOC=$(DOC) bib
	make DOC=$(DOC) simple

tex: 	$(SOURCES)
	@echo "Running pdflatex ..."
	pdflatex  $(DOC)

.PHONY: bib
bib: 
	@echo "Running bibtex ..."
	bibtex  $(DOC)

.PHONY: clean
clean:
	rm -f  *~  *.toc  *.aux  *.blg  *.bbl  *.log  *.out  *.dvi  */*.aux  */*~

.PHONY: fullclean
fullclean:
	make clean
	rm  $(DOCNAME).pdf
