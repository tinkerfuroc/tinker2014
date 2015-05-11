# Makefile of tinker2014
# created by bss at 2014-12-28
# Last modified: 2015-04-29, 19:51:53

CD=cd
PWD=pwd
CATKIN=catkin_make
CLEAN=catkin_make clean
FLGS=
PKGS=

all: dev downloaded driver logic decision navigation

.PHONY: all dev downloaded driver logic decision navigation clean

pkgs:
ifneq "$(strip $(pkg))" ""
PKGS=--pkg
PKGS+=$(pkg)
endif
FLGS+=$(PKGS)

dev: pkgs
	$(CD) ./dev && $(PWD) && $(CATKIN)$ $(FLGS)

downloaded: pkgs
	$(CD) ./downloaded && $(PWD) && $(CATKIN)$ $(FLGS)

driver: pkgs
	$(CD) ./driver && $(PWD) && $(CATKIN)$ $(FLGS)

logic: pkgs
	$(CD) ./logic && $(PWD) && $(CATKIN)$ $(FLGS)

decision: pkgs
	$(CD) ./decision && $(PWD) && $(CATKIN)$ $(FLGS)

navigation: pkgs
	$(CD) ./navigation && $(PWD) && $(CATKIN)$ $(FLGS)

clean_dev:
	$(CD) ./dev && $(PWD) && $(CLEAN)
	
clean_downloaded:
	$(CD) ./downloaded && $(PWD) && $(CLEAN)
	
clean_driver:
	$(CD) ./driver && $(PWD) && $(CLEAN)
	
clean_logic:
	$(CD) ./logic && $(PWD) && $(CLEAN)
	
clean_decision:
	$(CD) ./decision && $(PWD) && $(CLEAN)
	
clean_navigation:
	$(CD) ./navigation && $(PWD) && $(CLEAN)
	
clean: clean_dev clean_downloaded clean_driver clean_logic clean_decision clean_decision

