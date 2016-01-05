sv_driver_bin.o_shipped is renamed from sv_driver.o generated in the ../driver/ directory from the sv_driver.c source file. 

renaming the created .o file and using the Makefile in this directory allows us to distribute the binary blob instead of the source file
and link to a different kernel by running make.
