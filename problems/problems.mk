CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror
LIBS := -lm$(if $(USE_SDL), -lSDL -lSDLmain /usr/lib/libSDL_gfx.so) -largtable2
OBJ_DIR := ../obj

all: $(addsuffix .o,$(addprefix $(OBJ_DIR)/,$(PROBLEMS))$(if $(USE_SDL), $(addprefix $(OBJ_DIR)/viewer_,$(PROBLEMS)))) $(foreach i,2 3 4 5,$(OBJ_DIR)/swimmer_$i.o$(if $(USE_SDL), $(OBJ_DIR)/viewer_swimmer_$i.o))

$(OBJ_DIR)/swimmer_%.o: swimmer/swimmer.c swimmer/swimmer.h generative_model.h
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/viewer_swimmer_%.o: swimmer/viewer_swimmer.c viewer.h
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

.SECONDEXPANSION:
$(OBJ_DIR)/%.o: $$*/$$*.c $$*/$$*.h generative_model.h
	$(CC) -c $(FLAGS) $< -o $@

$(OBJ_DIR)/viewer_%.o: $$*/viewer_$$*.c viewer.h
	$(CC) -c $(FLAGS) $< -o $@
