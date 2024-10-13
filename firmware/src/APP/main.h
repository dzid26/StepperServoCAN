#ifndef __MAIN_H
#define __MAIN_H

#include <stdbool.h>
#include <assert.h>

#ifdef DEBUG
# define debug_assert(__e) ((__e) ? (void)0 : debug_assert_func(__FILE__, __LINE__, \
						       __ASSERT_FUNC, #__e))

void debug_assert_func(const char *file, int line, const char *func, const char *failedexpr);
#else
# define debug_assert(__e) ((void)0)
#endif /* DEBUG */


//called from interrupt
void Motion_task(void); 
void Service_task(void);

#endif
