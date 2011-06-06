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
	mkdir -p $(gikTaskDocdir)/html
	$(INSTALL_DATA) html/* $(gikTaskDocdir)/html
	mkdir -p $(gikTaskDocdir)/html/pictures
	$(INSTALL_DATA) $(srcdir)/pictures/* $(gikTaskDocdir)/html/pictures
	$(INSTALL_DATA) abstractGikTask.doxytag $(gikTaskDocdir)/html/.

html: gikTask.dox
	doxygen $<
