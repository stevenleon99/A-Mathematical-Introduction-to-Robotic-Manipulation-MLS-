# Makefile for screws package
# RMM, Dec 92
#

# Files which are part of the basic distribution
FILES = Screws.m RobotLinks.m scara.m elbow.m \
	screws.txi screws.ps

# Location of the distribution directory
DISTDIR = $(HOME)/www/books/MLS94/ftp
TARFILE = screws.tar
MATHDIR = /opt/tools/lib/math
INFODIR  = /opt/tools/info

screws.info: screws.txi
	makeinfo screws.txi

screws.ps: screws.txi
	tex screws.txi
	texindex screws.cp
	tex screws.txi
	dvips -o screws.ps screws

$(TARFILE): $(FILES)
	tar cf $@ $(FILES)

screws.shar: $(FILES)
	shar $(FILES) > screws.shar

install: 
	cp screws.info $(INFODIR)
	cp Screws.m RobotLinks.m $(MATHDIR)

distribution: $(FILES) $(TARFILE)
	cp -p $(FILES) $(TARFILE) $(DISTDIR)
