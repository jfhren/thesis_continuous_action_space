CC_OPTIONS := -O3
USE_SDL := 1
CC := gcc
FLAGS := -W -Wall $(CC_OPTIONS) -ansi -std=c99 -pedantic -Werror
LIBS := -lm$(if $(USE_SDL), /usr/lib/libSDL_gfx.so -lSDL -lSDLmain) -largtable2 -lgsl -lgslcblas
BIN_DIR := ../bin
OBJ_DIR := ../obj

all: $(addprefix $(BIN_DIR)/lipschitzian_xp_sum_,$(PROBLEMS)) $(addprefix $(BIN_DIR)/sequential_xp_sum_,$(PROBLEMS)) $(addprefix $(BIN_DIR)/sequential_soo_xp_sum_,$(PROBLEMS)) $(addprefix $(BIN_DIR)/random_search_xp_sum_,$(PROBLEMS)) $(foreach i,2 3 4 5,$(BIN_DIR)/lipschitzian_xp_sum_$i_swimmer $(BIN_DIR)/sequential_xp_sum_$i_swimmer $(BIN_DIR)/sequential_soo_xp_sum_$i_swimmer $(BIN_DIR)/random_search_xp_sum_$i_swimmer) $(BIN_DIR)/problems_xp_initial_states

$(BIN_DIR)/problems_xp_initial_states: $(OBJ_DIR)/problems_xp_initial_states.o
	$(CC) $(FLAGS) $(LIBS) $< -o $@

$(OBJ_DIR)/problems_xp_initial_states.o: problems_xp_initial_states.c
	$(CC) -c $(FLAGS) $< -o $@

$(BIN_DIR)/lipschitzian_xp_sum_double_cart_pole: $(OBJ_DIR)/lipschitzian_xp_sum_double_cart_pole.o $(OBJ_DIR)/lipschitzian_2.o $(OBJ_DIR)/double_cart_pole.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/random_search_xp_sum_double_cart_pole: $(OBJ_DIR)/random_search_xp_sum_double_cart_pole.o $(OBJ_DIR)/random_search_2.o $(OBJ_DIR)/double_cart_pole.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_xp_sum_double_cart_pole: $(OBJ_DIR)/sequential_xp_sum_double_cart_pole.o $(OBJ_DIR)/soo_2.o $(OBJ_DIR)/sequential_soo_2.o $(OBJ_DIR)/direct_2.o $(OBJ_DIR)/sequential_direct_2.o $(OBJ_DIR)/double_cart_pole.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_soo_xp_sum_double_cart_pole: $(OBJ_DIR)/sequential_soo_xp_sum_double_cart_pole.o $(OBJ_DIR)/soo_2.o $(OBJ_DIR)/sequential_soo_2.o $(OBJ_DIR)/double_cart_pole.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

.SECONDARY: #Marked as secondary file that should not be deleted because of the chaining of implicit rule
$(OBJ_DIR)/lipschitzian_xp_sum_levitation.o: lipschitzian_xp_sum_levitation.c
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/random_search_xp_sum_levitation.o: random_search_xp_sum_levitation.c
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/sequential_xp_sum_levitation.o: sequential_xp_sum_levitation.c
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/sequential_soo_xp_sum_levitation.o: sequential_soo_xp_sum_levitation.c
	$(CC) -c $(FLAGS) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/lipschitzian_xp_sum_double_cart_pole.o: lipschitzian_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DDOUBLE_CART_POLE -DNUMBER_OF_DIMENSIONS_OF_ACTION=2 $< -o $@

$(OBJ_DIR)/random_search_xp_sum_double_cart_pole.o: random_search_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DDOUBLE_CART_POLE -DNUMBER_OF_DIMENSIONS_OF_ACTION=2 $< -o $@

$(OBJ_DIR)/sequential_xp_sum_double_cart_pole.o: sequential_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DDOUBLE_CART_POLE -DNUMBER_OF_DIMENSIONS_OF_ACTION=2 $< -o $@

$(OBJ_DIR)/sequential_soo_xp_sum_double_cart_pole.o: sequential_soo_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DDOUBLE_CART_POLE -DNUMBER_OF_DIMENSIONS_OF_ACTION=2 $< -o $@

$(OBJ_DIR)/lipschitzian_xp_sum_swimmer_%.o: lipschitzian_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DSWIMMER -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/random_search_xp_sum_swimmer_%.o: random_search_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DSWIMMER -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/sequential_xp_sum_swimmer_%.o: sequential_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DSWIMMER -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/sequential_soo_xp_sum_swimmer_%.o: sequential_soo_xp_sum_problems.c
	$(CC) -c $(FLAGS) -DSWIMMER -DNUMBER_OF_DIMENSIONS_OF_ACTION=$* $< -o $@

$(OBJ_DIR)/lipschitzian_xp_sum_%.o: lipschitzian_xp_sum_problems.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/random_search_xp_sum_%.o: random_search_xp_sum_problems.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/sequential_xp_sum_%.o: sequential_xp_sum_problems.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

$(OBJ_DIR)/sequential_soo_xp_sum_%.o: sequential_soo_xp_sum_problems.c
	$(CC) -c $(FLAGS) -D$(shell echo $* | tr a-z A-Z) -DNUMBER_OF_DIMENSIONS_OF_ACTION=1 $< -o $@

.SECONDEXPANSION:
$(BIN_DIR)/lipschitzian_xp_sum_%_swimmer: $(OBJ_DIR)/lipschitzian_xp_sum_swimmer_$$*.o $(OBJ_DIR)/lipschitzian_%.o $(OBJ_DIR)/swimmer_$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/random_search_xp_sum_%_swimmer: $(OBJ_DIR)/random_search_xp_sum_swimmer_$$*.o $(OBJ_DIR)/random_search_%.o $(OBJ_DIR)/swimmer_$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_xp_sum_%_swimmer: $(OBJ_DIR)/sequential_xp_sum_swimmer_$$*.o $(OBJ_DIR)/soo_%.o $(OBJ_DIR)/sequential_soo_%.o $(OBJ_DIR)/direct_%.o $(OBJ_DIR)/sequential_direct_%.o $(OBJ_DIR)/swimmer_$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_soo_xp_sum_%_swimmer: $(OBJ_DIR)/sequential_soo_xp_sum_swimmer_$$*.o $(OBJ_DIR)/soo_%.o $(OBJ_DIR)/sequential_soo_%.o $(OBJ_DIR)/swimmer_$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/lipschitzian_xp_sum_%: $(OBJ_DIR)/lipschitzian_xp_sum_$$*.o $(OBJ_DIR)/lipschitzian_1.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/random_search_xp_sum_%: $(OBJ_DIR)/random_search_xp_sum_$$*.o $(OBJ_DIR)/random_search_1.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_xp_sum_%: $(OBJ_DIR)/sequential_xp_sum_$$*.o $(OBJ_DIR)/soo_1.o $(OBJ_DIR)/sequential_soo_1.o $(OBJ_DIR)/direct_1.o $(OBJ_DIR)/sequential_direct_1.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@

$(BIN_DIR)/sequential_soo_xp_sum_%: $(OBJ_DIR)/sequential_soo_xp_sum_$$*.o $(OBJ_DIR)/soo_1.o $(OBJ_DIR)/sequential_soo_1.o $(OBJ_DIR)/$$*.o
	$(CC) $(FLAGS) $(LIBS) $^ -o $@
