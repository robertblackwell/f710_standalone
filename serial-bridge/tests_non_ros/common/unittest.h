#ifndef iracoon_unittest_h
#define iracoon_unittest_h

#ifdef __cplusplus
extern "C" {
#endif


#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <libgen.h>
#include <stdbool.h>

#define GREEN(string)   "\x1b[32m" string "\x1b[0m"
#define YELLOW(string)  "\x1b[33m" string  "\x1b[0m"
#define MAGENTA(string) "\x1b[35m"  string  "\x1b[0m"
#define CYAN(string)    "\x1b[36m"  string  "\x1b[0m"
#define WHITE(string)   "\x1b[37m" string "\x1b[0m"
#define RESET(string)   "\x1b[0m"
#define BLUE(string)    "\x1b[34m" string "\x1b[0m"
#define RED(string)     "\x1b[31m" string "\x1b[0m"


#define    BRIGHT_BLACK(string)      "\x1b[30;1m" string  "\x1b[0m"
#define    BRIGHT_RED(string)        "\x1b[31;1m" string  "\x1b[0m"
#define    BRIGHT_GREEN(string)      "\x1b[32;1m" string  "\x1b[0m"
#define    BRIGHT_YELLOW(string)     "\x1b[33;1m" string  "\x1b[0m"
#define    BRIGHT_BLUE(string)       "\x1b[34;1m" string  "\x1b[0m"
#define    BRIGHT_MAGENTA(string)    "\x1b[35;1m" string  "\x1b[0m"
#define    BRIGHT_CYAN(string)       "\x1b[36;1m" string  "\x1b[0m"
#define    BRIGHT_WHITE(string)      "\x1b[37;1m" string  "\x1b[0m"

typedef int(UTFunction)();
#define UT_MAX_MSG_SIZE 256

typedef struct {
    const char* fn_name;
    const char* file_name;
    int line;
    char msg[UT_MAX_MSG_SIZE];
} UTAssertResult, *UTAssertResultRef;

typedef struct  UTObject_s {
	const char* name;
	UTFunction* fn;
	bool passed;
	char msg[UT_MAX_MSG_SIZE];
} UTObject, *UTObjectRef;


// make a testcase from a test function
void UTRegister(UTObjectRef uto);
int UTRun();
void UTRecordAssertResult(const char* fn, const char* file, int line, const char* msg);


#define UT_EQUAL_INT(a,b) do{\
	if(a != b) { \
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %d b = %d\n"); \
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b); \
		UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);
#define UT_NOT_EQUAL_INT(a,b) do{\
	if(a == b) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %d b = %d\n"); \
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);

#define UT_NOT_EQUAL_LONG(a,b) do{\
	if(a == b) {\
	    char* msg; \
		const char fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %ld b = %ld\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);
#define UT_EQUAL_LONG(a,b) do{\
	if(a != b) {\
	    char* msg; \
		const char fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %ld b = %ld\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);

#define UT_NOT_EQUAL_DOUBLE(a,b) do{\
	if(a == b) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %f b = %f\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);
#define UT_EQUAL_DOUBLE(a,b) do{\
	if(a != b) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %f b = %f\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);


#define UT_EQUAL_PTR(a,b) do{\
	if(a != b) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %lx b = %lx\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);

#define UT_NOT_EQUAL_PTR(a,b) do{\
	if(a == b) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %lx b = %lx\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);

#define UT_EQUAL_CSTR(a,b) do{\
	if(strcmp(a,b) != 0) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %s b = %s\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);

#define UT_NOT_EQUAL_CSTR(a,b) do{\
	if(strcmp(a,b) == 0) {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" not equal a = %s b = %s\n");\
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a , b);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);


#define UT_TRUE(a) do{\
	if(!a)  {\
	    char* msg; \
		const char* fmt = BRIGHT_RED("FAILED report ") BRIGHT_CYAN("func:") BLUE(" %s") BRIGHT_CYAN(" file:") BLUE(" %s ") BRIGHT_CYAN("line:") BLUE("%d") BRIGHT_BLUE(" a = %d n"); \
		int x = asprintf(&msg, fmt, __FUNCTION__, __FILE__, __LINE__,a);\
        UTRecordAssertResult(__FUNCTION__, __FILE__, __LINE__, msg);   \
        free(msg); \
		return 1; \
	}	\
} while(0);


#define REQUIRE(expression) UT_TRUE((expression))
#define CHECK(expression) UT_TRUE((expression))
#define UT_ADD(name) UTObject uobj_ ## name = {#name, name}; UTRegister(&uobj_ ## name);
#define UT_RUN(tests) UTRun();

#ifdef __cplusplus
}
#endif



#endif