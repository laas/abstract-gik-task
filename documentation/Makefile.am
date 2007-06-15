# Research carried out within the scope of the Associated
# International Laboratory: Joint Japanese-French Robotics Laboratory
# (JRL)
#
# Author: Florent Lamiraux
#

gikTaskDocdir = $(datadir)/doc/abstractGikTask

EXTRA_DIST = \
	header.html \
	footer.html \
	hpp.css \
	additionalDoc/gikTask.h \
	pictures/HRP2.jpg 

install-data-hook: html
	mkdir -p $(gikTaskDocdir)
	$(INSTALL_DATA) html/* $(gikTaskDocdir)
	mkdir -p $(gikTaskDocdir)/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(gikTaskDocdir)/pictures

html: gikTask.dox
	doxygen $<
