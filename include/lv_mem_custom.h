#ifndef LV_MEM_CUSTOM_H
#define LV_MEM_CUSTOM_H

#include "esp_heap_caps.h"

#undef LV_MEM_CUSTOM_ALLOC
#undef LV_MEM_CUSTOM_FREE
#undef LV_MEM_CUSTOM_REALLOC

#define LV_MEM_CUSTOM_ALLOC(size)          heap_caps_malloc(size, MALLOC_CAP_SPIRAM)
#define LV_MEM_CUSTOM_FREE(p)              heap_caps_free(p)
#define LV_MEM_CUSTOM_REALLOC(p, size)     heap_caps_realloc(p, size, MALLOC_CAP_SPIRAM)

#endif
