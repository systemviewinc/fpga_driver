
obj-m += vsi_driver.o

# vsi_driver-y += xdma/sv_xdma.o
# vsi_driver-y += xdma/xdma-core.o
# vsi_driver-y += xdma/xdma-sgm.o
vsi_driver-y += support_funcs.o
vsi_driver-y += sv_streaming_funcs.o
vsi_driver-y += sv_driver.o

#CFLAGS_xdma-core.o := -DDEBUG
#CFLAGS_xdma-sgm.o := -DDEBUG
#CFLAGS_sv_xdma.o := -DDEBUG
#CFLAGS_support_funcs.o := -DDEBUG

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
# Get head commit
# GIT_EXEC=git
# CURRENT_DIR = $(shell pwd)
# PROJECT_ROOT = ${CURRENT_DIR}/..
# GIT_DIR = ${PROJECT_ROOT}/.git
# ifeq (, $(shell which ${GIT_EXEC}))
# 	HEAD_COMMIT = `echo Undefined_${GIT_EXEC}_not_found.`
# 	COMMIT_STATUS = ""
# 	DEATTACH_STATUS = `echo `
# else
# 	ifneq ($(wildcard ${GIT_DIR}),)
# 		git_location_args = --work-tree=${PROJECT_ROOT} --git-dir=${GIT_DIR}
# 		ifeq (, $(${GIT_EXEC} ${git_location_args} status --porcelain))
# 			# No uncommitted changes
# 			# Get current branch
# 			DEATTACH_STATUS = `${GIT_EXEC} branch`
# 		else
# 			DEATTACH_STATUS = `echo You are in \'detached HEAD\'`
# 			# Uncommited changes
# 		endif
# 		# Get hash
# 		HEAD_COMMIT += `${GIT_EXEC} ${git_location_args}  rev-parse --short HEAD`
# 	else
# 		HEAD_COMMIT = `echo Not_a_git_repository.`
# 	endif
# endif
# CUSTOMDEFINES+=-DHEAD_COMMIT=\"$(HEAD_COMMIT)\"

all:
	@echo ${CUSTOMDEFINES}
	make CFLAGS_MODULE='$(CUSTOMDEFINES)'  -C ${KERNEL_SRC} M=$(PWD) modules
modules_install:
	make -C ${KERNEL_SRC} ${CUSTOMDEFINES} M=$(PWD) modules_install
clean:
	make -C ${KERNEL_SRC} M=$(PWD) clean
