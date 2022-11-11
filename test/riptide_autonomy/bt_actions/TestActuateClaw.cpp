#include "autonomy_test/autonomy_testing.hpp"

using ActuateClaw = riptide_msgs2::action::ChangeClawState;

class ActuateClawTest : public ::testing::Test {
    public:
    static void SetUpTestSuite() { }
    static void TearDownTestSuite() { }
};

TEST(BtTest, test_ActuateClaw) {
    //TODO: define a test here and delete below line when you do
    GTEST_SKIP();
}

//TODO: define more tests here
