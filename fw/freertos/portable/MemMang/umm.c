#include "umm/umm_malloc.h"

#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
	extern uint8_t heap[ configTOTAL_HEAP_SIZE ];
#else
u8 heap[configTOTAL_HEAP_SIZE] __section(".heap");
#endif

#if(configUSE_MALLOC_FAILED_HOOK == 1)
    extern void vApplicationMallocFailedHook(void);
#endif

void *pvPortMalloc( size_t xWantedSize )
{
    void *pvReturn;

    pvReturn = umm_malloc(xWantedSize);
    traceMALLOC(pvReturn, xWantedSize);

    #if(configUSE_MALLOC_FAILED_HOOK == 1)
    {
        if (pvReturn == NULL) vApplicationMallocFailedHook();
    }
    #endif

    return pvReturn;
}


void vPortFree(void *pv)
{
    if (pv) {
        umm_free(pv);
        traceFREE(pv, 0);
    }
}


void *pvPortRealloc( void *pv, size_t xWantedSize )
{
    if (pv) {
        pv = umm_realloc(pv, xWantedSize);


        #if(configUSE_MALLOC_FAILED_HOOK == 1)
        {
            if (pv == NULL) vApplicationMallocFailedHook();
        }
        #endif
    }

    return pv;
}
