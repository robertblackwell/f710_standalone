#include "unittest.h"
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define UT_MAX_TESTS 1000
#define UT_MAX_ASSERTS 3000

pthread_mutex_t ut_lock;
static int utest_count = 0;
static UTObjectRef    utest_table[UT_MAX_TESTS];
static UTAssertResultRef ut_assert_table[UT_MAX_ASSERTS];
static int ut_assert_count = 0;
static int utest_error_count;

// make a testcase from a test function
// should only be run from the main thread
void UTRegister(UTObjectRef uto)
{
	if (utest_count == 0) {
	    pthread_mutex_init(&ut_lock, NULL);
		for(int i = 0; i < UT_MAX_TESTS; i++) {
			utest_table[i] = NULL;
		}
		for(int i = 0; i < UT_MAX_ASSERTS; i++) {
		    ut_assert_table[i] = NULL;
        }
	}

	utest_table[utest_count] = uto;
	utest_count++;
    if(utest_count >= UT_MAX_TESTS) {
        printf(
                BRIGHT_RED("TESTS ABORTED - TOO Many Tests\n")
                BRIGHT_CYAN("Maximum number of tests        -- UT_MAX_TESTS   is : ")
                WHITE("%d \n")
                BRIGHT_CYAN("Maximum number of UT_ asserts  -- UT_MAX_ASSERTS is : ")
                WHITE("%d \n")
                WHITE("Change #define values in unittest.c")
                "\n", UT_MAX_TESTS, UT_MAX_ASSERTS
        );
        exit(-1);
    }

}
int UTRunOne(UTObjectRef utoref)
{
	int result = utoref->fn();
	utoref->passed = (result == 0);
	return result;
}
void UTReport()
{
    for(int i = 0; i < UT_MAX_TESTS; i++) {
        if(utest_table[i] != NULL) {
            printf("Test %s %s \n", utest_table[i]->name, (utest_table[i]->passed ? BRIGHT_GREEN("Passed") : BRIGHT_RED("Failed")));
        }
    }
    for(int i = 0; i < UT_MAX_ASSERTS; i++) {
        if(ut_assert_table[i] != NULL) {
            printf("%s", ut_assert_table[i]->msg);
        }
    }
}
int UTRun()
{
    bool all_passed = true;
	for(int i = 0; i < UT_MAX_TESTS; i++) {
	    UTObjectRef ref = utest_table[i];
		if(ref != NULL) {
		    int rc = UTRunOne(ref);
			all_passed = all_passed && (rc == 0);
		} 
	}
	UTReport();
	return (all_passed)? 0: 1;
}

void UTRecordAssertResult(const char* fn, const char* file, int line, const char* msg)
{
    // only asserts are to be running from multiple threads
    pthread_mutex_lock(&ut_lock);
    UTAssertResultRef arref = (UTAssertResultRef)malloc(sizeof(UTAssertResult));
    strcpy(arref->msg, msg);
    arref->fn_name = fn;
    arref->file_name = file;
    ut_assert_table[ut_assert_count] = arref;
    ut_assert_count++;
    if(ut_assert_count >= UT_MAX_ASSERTS) {
        printf(
                BRIGHT_RED("TESTS ABORTED - TOO Many ASSERTS\n")
                BRIGHT_CYAN("Maximum number of tests        -- UT_MAX_TESTS   is : ")
                WHITE("%d \n")
                BRIGHT_CYAN("Maximum number of UT_ asserts  -- UT_MAX_ASSERTS is : ")
                WHITE("%d \n")
                WHITE("Change #define values in unittest.c")
                "\n", UT_MAX_TESTS, UT_MAX_ASSERTS
        );
        pthread_mutex_unlock(&ut_lock);
        exit(-1);
    }

    pthread_mutex_unlock(&ut_lock);
}