DESTDIR = /home/ftp/pub/MLS

EXMPS =	exmp2-1.m exmp2-2.m exmp2-3.m exmp2-4.m exmp2-5.m exmp2-6.m \
	exmp3-1.m exmp3-10.m exmp3-17.m exmp3-2.m exmp3-3.m exmp3-8.m \
	exmp3-9.m ch3sec5-3.m \
	exmp4-1.m exmp4-3.m exmp4-4.m exmp4-8.m ch4sec2-3.m ch4sec6-3.m \
	ch5sec5-3.m exmp6-1.m \
	exmp7-10.m exmp7-11.m exmp7-2.m exmp7-3.m exmp7-4.m exmp7-5.m \
	exmp7-6.m exmp7-7.m exmp7-8.m exmp7-9.m \
	Jac.m

all: examples.tar

install: examples.tar
	cp -p examples.tar $(EXMPS) $(SCREWS) $(DESTDIR)

examples.tar: $(EXMPS)
	tar cfh examples.tar $(EXMPS)

clean:
	/bin/rm -f *~ *.tar core *.[cho] *.cfg *.d 
	/bin/rm -f joint work
