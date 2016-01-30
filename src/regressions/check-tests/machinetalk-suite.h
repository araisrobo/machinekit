#ifndef _TEST_MACHINETALK_SUITE
#define _TEST_MACHINETALK_SUITE

#include <stdlib.h>

START_TEST(test_machinetalk)
{
    ck_assert_int_eq(0, 0);
}
END_TEST

Suite * machinetalk_suite(void)
{
    Suite *s;
    TCase *tc_core;
    s = suite_create("machinetalk");

    tc_core = tcase_create("machinetalk");

    tcase_add_test(tc_core, test_machinetalk);
    suite_add_tcase(s, tc_core);

    return s;
}


#endif /* _TEST_MACHINETALK_SUITE */
