#include "autonomy_test/autonomy_testing.hpp"

/**
 * @brief Initializes GTest and runs it. GTest will find and run any tests in the other files that this one is compiled against.
 * 
 * @param argc arg count.
 * @param argv arg values
 * @return int exit status
 */
int main(int argc, char **argv) {
    BtTest::initBtTest(argc, argv);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
