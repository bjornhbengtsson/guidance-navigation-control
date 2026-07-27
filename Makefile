CC ?= gcc

CFLAGS := \
    -std=c11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -Iinclude

BUILD_DIR := build
TEST_BINARY := $(BUILD_DIR)/test_math

MATH_SOURCES := \
    src/math/vector3.c \
    src/math/quaternion.c

TEST_SOURCES := \
    tests/unit/test_math.c

.PHONY: all test clean

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_BINARY): $(MATH_SOURCES) $(TEST_SOURCES) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MATH_SOURCES) $(TEST_SOURCES) -lm -o $(TEST_BINARY)

test: $(TEST_BINARY)
	./$(TEST_BINARY)

clean:
	rm -rf $(BUILD_DIR)
