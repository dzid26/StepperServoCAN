#include <unity.h>
#include "utils.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}
static void test_maximum(void) {
    TEST_ASSERT_EQUAL(max(0, 0), 0);
    TEST_ASSERT_EQUAL(max(7, 5), 7);
    TEST_ASSERT_EQUAL(max(0, 100), 100);
    TEST_ASSERT_EQUAL(max(-7, -5), -5);
    TEST_ASSERT_EQUAL(max(-100, 0), 0);
    TEST_ASSERT_EQUAL(max(-20, 20), 20);
    TEST_ASSERT_EQUAL(max(10, -10), 10);
    TEST_ASSERT_EQUAL(max((uint16_t)65535, (uint8_t)255), 65535);
    TEST_ASSERT_EQUAL(max((int16_t)-32768, (int8_t)-128), -128);
    TEST_ASSERT_EQUAL_FLOAT(max(2.5f, 1.2f), 2.5f);
    TEST_ASSERT_EQUAL_FLOAT(max(-2.5f, -1.2f), -1.2f);
}

static void test_minimum(void) {
    TEST_ASSERT_EQUAL(min(0, 0), 0);
    TEST_ASSERT_EQUAL(min(7, 5), 5);
    TEST_ASSERT_EQUAL(min(0, 100), 0);
    TEST_ASSERT_EQUAL(min(-7, -5), -7);
    TEST_ASSERT_EQUAL(min(-100, 0), -100);
    TEST_ASSERT_EQUAL(min(-20, 20), -20);
    TEST_ASSERT_EQUAL(min(10, -10), -10);
    TEST_ASSERT_EQUAL(min((uint16_t)65535, (uint8_t)255), 255);
    TEST_ASSERT_EQUAL(max((int16_t)-32768, (int8_t)-128), -128);
    TEST_ASSERT_EQUAL_FLOAT(min(2.5f, 1.2f), 1.2f);
    TEST_ASSERT_EQUAL_FLOAT(min(-2.5f, -1.2f), -2.5f);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_maximum);
    RUN_TEST(test_minimum);
    return UNITY_END();
}
