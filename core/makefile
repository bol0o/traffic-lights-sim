CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I. -Ilib -Itests

BIN_DIR = bin
LIB_DIR = lib
TEST_DIR = tests

EXEC_TEST_QUEUE = $(BIN_DIR)/test_queue
EXEC_TEST_FSM   = $(BIN_DIR)/test_fsm
EXEC_APP        = $(BIN_DIR)/traffic_sim

SRC_QUEUE = $(LIB_DIR)/traffic_queue.c
SRC_FSM   = traffic_fsm.c
SRC_MAIN  = main_pc.c

all: $(EXEC_TEST_QUEUE) $(EXEC_TEST_FSM) $(EXEC_APP)

$(EXEC_APP): $(SRC_MAIN) $(SRC_FSM) $(SRC_QUEUE)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^

$(EXEC_TEST_QUEUE): $(TEST_DIR)/test_queue.c $(SRC_QUEUE)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^

$(EXEC_TEST_FSM): $(TEST_DIR)/test_fsm.c $(SRC_FSM) $(SRC_QUEUE)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^

test_queue: $(EXEC_TEST_QUEUE)
	@./$(EXEC_TEST_QUEUE)

test_fsm: $(EXEC_TEST_FSM)
	@./$(EXEC_TEST_FSM)

test: test_queue test_fsm

clean:
	rm -rf $(BIN_DIR)/*

.PHONY: all test test_queue test_fsm clean