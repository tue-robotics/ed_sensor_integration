#include <gtest/gtest.h>
#include <iostream>

// TU/e Robotics
#include <ed/world_model.h>


TEST(TestSuite, testCase)
{
    std::cout << "Running test" << "std::endl";
    ASSERT_TRUE(false);
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

//  g_argc = argc;
//  g_argv = argv;

  return RUN_ALL_TESTS();
}
