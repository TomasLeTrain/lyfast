################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp cc
CXXEXTSNOLTO:=c++

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-Wunused-variable -Wall

LIBDIR=$(INCDIR)/$(LIBNAME)

MAKEFLAGS=-j

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:=$(FWDIR)/float_units.a $(FWDIR)/blazing.a $(FWDIR)/Eigen.a
EXCLUDE_COLD_LIBRARIES:=$(FWDIR)/float_units.a $(FWDIR)/blazing.a

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1

# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=lyfast
VERSION:=0.0.7
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
# EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))
EXCLUDE_SRC_FROM_LIB+=$(SRCDIR)/main.cpp

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
TEMPLATE_FILES+=$(LIBDIR)/**/**

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
