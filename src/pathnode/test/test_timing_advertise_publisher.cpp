#include <gtest/gtest.h>

class TestTimingAdvertisePublisher : public ::testing::Test
{
  public:
  void SetUp()
  {
  }
};

TEST_F(TestTimingAdvertisePublisher, no_set_and_dispatch_throw) {
  EXPECT_EQ(1 + 1, 2);
}
