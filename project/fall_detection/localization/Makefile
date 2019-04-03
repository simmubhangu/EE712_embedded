.PHONY : all
all : documentation executable
DOC ?= fileName
SOURCES = $(DOC).c.nw
ROOT ?= includes

all : documentation tangle clean

weave: $(SOURCES)
	noweave -delay -index $(SOURCES) > $(DOC).tex
tangle: $(SOURCES)
	notangle -R$(ROOT) $(SOURCES) > $(DOC).c
documentation: weave
	pdflatex $(DOC).tex; pdflatex $(DOC).tex; pdflatex $(DOC).tex
executable: tangle
	gcc $(DOC).c -lm -g -o $(DOC)

.PHONY: clean
clean:
	rm -f *.log *.aux
full_clean:
	rm -f *.log *.aux $(DOC).tex $(DOC).c $(DOC).pdf
