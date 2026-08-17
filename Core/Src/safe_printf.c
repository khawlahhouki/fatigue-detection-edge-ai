/* safe_printf.c
 *
 * Voir safe_printf.h pour l'usage.
 */

#include "safe_printf.h"
#include <stdarg.h>
#include <stdio.h>
#include "cmsis_os.h"

/* Mutex partagé par toutes les tâches qui appellent SafePrintf().
 * NULL tant que SafePrintf_Init() n'a pas été appelée. */
static osMutexId_t MTX_PrintfHandle = NULL;

void SafePrintf_Init(void)
{
    const osMutexAttr_t attr = {
        .name = "MTX_Printf"
    };
    MTX_PrintfHandle = osMutexNew(&attr);
}

int SafePrintf(const char *fmt, ...)
{
    int ret;

    /* Sécurité : si Init n'a pas encore été appelée (ne devrait
     * jamais arriver si SafePrintf_Init() est bien placée avant
     * osKernelStart()), on imprime quand même, sans protection,
     * plutôt que de perdre le message. */
    if (MTX_PrintfHandle != NULL)
    {
        osMutexAcquire(MTX_PrintfHandle, osWaitForever);
    }

    va_list args;
    va_start(args, fmt);
    ret = vprintf(fmt, args);
    va_end(args);

    if (MTX_PrintfHandle != NULL)
    {
        osMutexRelease(MTX_PrintfHandle);
    }

    return ret;
}
