#############################################################################
#
# Makefile
# 
# Application Makefile
#
#############################################################################

include $(VCTL_ARCHIVE)/scripts/Makefile.defines

# special options ==>
# USER_DEFINES = -g
USER_DEFINES =

#
# Program object and library dependencies
#
BIN_OBJ = $(BINNAME).o

BIN_EXTRA_LIBS = -lstdc++ -lpthread -ltcl -lppm -lrt -lX11 -lm
BIN_EXTRA_LIB_DIRS = $(LIB_DIRS)

#
# Compile application, copy and create
#
all: $(BINNAME) copy

#examples: $(basename $(notdir $(shell ls ../examples/*)))

#
# Compile application
#
$(BINNAME): $(addprefix $(OBJSUB)/,$(BIN_OBJ)) $(BINFULLLIBS)
	$(ECHO) linking program
	$(SILENT) $(CC) -o $(BINSUB)/$(BINNAME) \
	  $(addprefix $(OBJSUB)/,$(BIN_OBJ)) \
	  $(BINLIBDIRS) $(BINLIBS) $(BINLIBS) $(BINLIBS) \
	  $(BIN_EXTRA_LIBS) $(BIN_EXTRA_LIB_DIRS)

#
#
#
include $(VCTL_ARCHIVE)/scripts/Makefile.rules
