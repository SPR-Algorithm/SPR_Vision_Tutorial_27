add_test([=[MatrixTest.AddMatrix]=]  C:/Users/Lenovo/SPR_Vision_Tutorial_27/build/Debug/test_matrix.exe [==[--gtest_filter=MatrixTest.AddMatrix]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixTest.AddMatrix]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Lenovo\SPR_Vision_Tutorial_27\test\test_matrix.cpp:5]==]
    WORKING_DIRECTORY [==[C:/Users/Lenovo/SPR_Vision_Tutorial_27/build]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixTest.MultiplyMatrix]=]  C:/Users/Lenovo/SPR_Vision_Tutorial_27/build/Debug/test_matrix.exe [==[--gtest_filter=MatrixTest.MultiplyMatrix]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixTest.MultiplyMatrix]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Lenovo\SPR_Vision_Tutorial_27\test\test_matrix.cpp:20]==]
    WORKING_DIRECTORY [==[C:/Users/Lenovo/SPR_Vision_Tutorial_27/build]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[MatrixTest.DimensionCheck]=]  C:/Users/Lenovo/SPR_Vision_Tutorial_27/build/Debug/test_matrix.exe [==[--gtest_filter=MatrixTest.DimensionCheck]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[MatrixTest.DimensionCheck]=]
  PROPERTIES
    
    DEF_SOURCE_LINE [==[C:\Users\Lenovo\SPR_Vision_Tutorial_27\test\test_matrix.cpp:36]==]
    WORKING_DIRECTORY [==[C:/Users/Lenovo/SPR_Vision_Tutorial_27/build]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(test_matrix_TESTS [==[MatrixTest.AddMatrix]==] [==[MatrixTest.MultiplyMatrix]==] [==[MatrixTest.DimensionCheck]==])
