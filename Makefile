CC ?= gcc

CFLAGS := \
    -std=c11 \
    -Wall \
    -Wextra \
    -Wpedantic \
    -Werror \
    -Iinclude

BUILD_DIR := build
MATH_TEST_BINARY := $(BUILD_DIR)/test_math
MAHONY_TEST_BINARY := $(BUILD_DIR)/test_mahony

MATH_SOURCES := \
    src/math/vector3.c \
    src/math/quaternion.c

MAHONY_SOURCES := \
    src/attitude/mahony.c

.PHONY: all test test-math test-mahony clean

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(MATH_TEST_BINARY): $(MATH_SOURCES) tests/unit/test_math.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MATH_SOURCES) tests/unit/test_math.c -lm -o $(MATH_TEST_BINARY)

$(MAHONY_TEST_BINARY): $(MATH_SOURCES) $(MAHONY_SOURCES) tests/unit/test_mahony.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(MATH_SOURCES) $(MAHONY_SOURCES) tests/unit/test_mahony.c -lm -o $(MAHONY_TEST_BINARY)

test-math: $(MATH_TEST_BINARY)
	./$(MATH_TEST_BINARY)

test-mahony: $(MAHONY_TEST_BINARY)
	./$(MAHONY_TEST_BINARY)

test: test-math test-mahony

clean:
	rm -rf $(BUILD_DIR)
