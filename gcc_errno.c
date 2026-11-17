/**
 * gcc_errno.c — provide __errno for newlib's libm
 * Replaces the missing __errno symbol when using -specs=nosys.specs
 */
#include <errno.h>

int *__errno(void)
{
    return &errno;
}
