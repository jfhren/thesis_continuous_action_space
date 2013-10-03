CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
DIRECT_FLAGS := -W -Wall -g -O1 -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL) #It's somewhat not working with my version of GCC with -O2 or -O3
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/sequential_direct_,$(PROBLEMS)) $(foreach i,2 3 4 5,$(BIN_DIR)/sequential_direct_$i_swimmer)

$(BIN_DIR)/sequential_direct_double_cart_pole: $(OBJ_DIR)/direct_2.o $(OBJ_DIR)/sequential_direct_2.o $(OBJ_DIR)/main_sequential_direct_2.o $(OBJ_DIR)/double_cart_pole.o $(if $(USE_SDL),$(OBJ_DIR)/viewer_double_cart_pole.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

.SECONDARY: #Marked as secondary file that should not be deleted because of the chaining of implicit rule
$(OBJ_DIR)/sequential_direct_%.o: sequential_direct/sequential_direct.c sequential_direct/sequential_direct.h
	$(CC) -c $(FLAGS) -D NUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/main_sequential_direct_%.o: sequential_direct/main_sequential_direct.c
	$(CC) -c $(FLAGS) -D NUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/direct_%.o: sequential_direct/direct.c sequential_direct/direct.h
	$(CC) -c $(DIRECT_FLAGS) -D NUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/sequential_direct_%_swimmer: $(OBJ_DIR)/direct_$$*.o $(OBJ_DIR)/sequential_direct_$$*.o $(OBJ_DIR)/main_sequential_direct_$$*.o $(OBJ_DIR)/swimmer_$$*.o $$(if $(USE_SDL),$(OBJ_DIR)/viewer_swimmer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_direct_%: $(OBJ_DIR)/direct_1.o $(OBJ_DIR)/sequential_direct_1.o $(OBJ_DIR)/main_sequential_direct_1.o $(OBJ_DIR)/$$*.o $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
