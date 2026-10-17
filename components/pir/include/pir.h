#ifndef _PIR_
#define _PIR_

#include <stdbool.h>       //bool类型

#include "esp_err.h"       //esp_err_t返回值类型

bool pir_is_active(void);
esp_err_t pir_init(void);

#endif
