CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I. -Ilib

BIN_DIR = bin
LIB_DIR = lib
TEST_DIR = tests

TEST_EXEC = $(BIN_DIR)/test_runner

TEST_SRCS = $(TEST_DIR)/test_queue.c $(LIB_DIR)/traffic_queue.c

all: $(TEST_EXEC)

$(TEST_EXEC): $(TEST_SRCS)
	@mkdir -p $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^

test: $(TEST_EXEC)
	@./$(TEST_EXEC)

check_fsm:
	$(CC) $(CFLAGS) -c traffic_fsm.c -o $(BIN_DIR)/traffic_fsm.o

clean:
	rm -rf $(BIN_DIR)/*

.PHONY: all test clean check_fsm