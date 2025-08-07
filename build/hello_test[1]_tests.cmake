add_test( HelloTest.BasicAssert /home/mrobins/my_repos/maniptools/build/hello_test [==[--gtest_filter=HelloTest.BasicAssert]==] --gtest_also_run_disabled_tests)
set_tests_properties( HelloTest.BasicAssert PROPERTIES WORKING_DIRECTORY /home/mrobins/my_repos/maniptools/build SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==])
set( hello_test_TESTS HelloTest.BasicAssert)
