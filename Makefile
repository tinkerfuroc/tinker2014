# Makefile of tinker2014
# created by bss at 2014-12-28
# Last modified: 2014-12-28, 19:58:20

CD=cd
PWD=pwd
CATKIN=catkin_make
CLEAN=catkin_make clean

all: dev downloaded driver logic decision

.PHONY: dev downloaded driver logic decision

dev:
	$(CD) ./dev && $(PWD) && $(CATKIN)

downloaded:
	$(CD) ./downloaded && $(PWD) && $(CATKIN)

driver:
	$(CD) ./driver && $(PWD) && $(CATKIN)

logic:
	$(CD) ./logic && $(PWD) && $(CATKIN)

decision:
	$(CD) ./decision && $(PWD) && $(CATKIN)

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
	
clean: clean_dev clean_downloaded clean_driver clean_logic clean_decision
