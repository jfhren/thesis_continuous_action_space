#Uncomment to build with debug symbols
#export CC_OPTIONS := -g
#Uncomment to build without SDL (and thus without viewer)
#export USE_SDL := 

#The list of problems found in the problems directory (swimmers are a special case so we left them out)
export PROBLEMS := $(filter-out swimmer,$(shell ls -d problems/*/ | cut -f 2 -d '/'))
BIN_DIR := ./bin
OBJ_DIR := ./obj

.PHONY: make_directories clean 

all: make_directories
	$(MAKE) -C problems -f problems.mk -e
	$(MAKE) -C algorithms -f lipschitzian.mk -e
	$(MAKE) -C algorithms -f sequential_direct.mk -e
	$(MAKE) -C algorithms -f sequential_soo.mk -e
	$(MAKE) -C algorithms -f random_search.mk -e

tools: all
	$(MAKE) -C tools -f tools.mk -e

make_directories:
	mkdir -p $(BIN_DIR)
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(BIN_DIR)
	rm -rf $(OBJ_DIR)

