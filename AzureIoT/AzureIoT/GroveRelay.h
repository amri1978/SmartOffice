#pragma once

//#include "../../applibs_versions.h"
#include <applibs/gpio.h>

void* GroveRelay_Open(GPIO_Id pinId);
void GroveRelay_On(void* inst);
void GroveRelay_Off(void* inst);
