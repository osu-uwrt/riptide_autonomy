#include "autonomy_test/autonomy_testing.hpp"
//will also #include whatever node is being tested, which will #include autonomy_lib.

TEST(dummy_suite, dummy_decorator_test) {
    EXPECT_EQ(2 * 2,  4);
}
