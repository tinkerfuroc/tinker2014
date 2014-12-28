# Makefile of tinker2014
# created by bss at 2014-12-28
# Last modified: 2014-12-28, 19:19:13

CD=cd
PWD=pwd
CATKIN=catkin_make

all:
	$(CD) ./dev && $(PWD) && $(CATKIN)
	$(CD) ./downloaded && $(PWD) && $(CATKIN)
	$(CD) ./driver && $(PWD) && $(CATKIN)
	$(CD) ./logic && $(PWD) && $(CATKIN)
	$(CD) ./decision && $(PWD) && $(CATKIN)

