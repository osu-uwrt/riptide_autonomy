#include <gtest/gtest.h>

/**
 * @brief Initializes GTest and runs it. GTest will find and run any tests in the library that this file is compiled against.
 * 
 * @param argc arg count.
 * @param argv arg values
 * @return int exit status
 */
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
