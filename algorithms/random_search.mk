CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror$(if $(USE_SDL), -DUSE_SDL)
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all:  $(addprefix $(BIN_DIR)/random_search_,$(PROBLEMS)) $(foreach i,2 3 4 5,$(BIN_DIR)/random_search_$i_swimmer)

$(BIN_DIR)/random_search_double_cart_pole: $(OBJ_DIR)/random_search_2.o $(OBJ_DIR)/main_random_search_2.o $(OBJ_DIR)/double_cart_pole.o $(if $(USE_SDL),$(OBJ_DIR)/viewer_double_cart_pole.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

.SECONDARY: #Marked as secondary file that should not be deleted because of the chaining of implicit rule
$(OBJ_DIR)/random_search_%.o: random_search/random_search.c random_search/random_search.h
	$(CC) -c $(FLAGS) -D NUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/main_random_search_%.o: random_search/main_random_search.c
	$(CC) -c $(FLAGS) -D NUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/random_search_%_swimmer: $(OBJ_DIR)/random_search_$$*.o $(OBJ_DIR)/main_random_search_$$*.o $(OBJ_DIR)/swimmer_$$*.o $$(if $(USE_SDL),$(OBJ_DIR)/viewer_swimmer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/random_search_%: $(OBJ_DIR)/random_search_1.o $(OBJ_DIR)/main_random_search_1.o $(OBJ_DIR)/$$*.o $$(if $(USE_SDL),$(OBJ_DIR)/viewer_$$*.o)
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
