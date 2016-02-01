// example test suite.
// if hal is needed, include in check_hal.c, else check_other.c

#ifndef _TEST_HELLO_WORLD_SUITE
#define _TEST_HELLO_WORLD_SUITE

START_TEST(test_hello_world)
{
    ck_assert_int_eq(0, 0);
}
END_TEST

START_TEST(test_goodbye_world)
{
    ck_assert_int_eq(0, 0);
}
END_TEST

Suite * hello_world_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("helloWorld");

    tc_core = tcase_create("Core");
    tcase_add_test(tc_core, test_hello_world);
    tcase_add_test(tc_core, test_goodbye_world);
    suite_add_tcase(s, tc_core);

    return s;
}


#endif /* _TEST_HELLO_WORLD_SUITE */
