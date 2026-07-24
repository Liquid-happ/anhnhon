#include "ctx.h"
#include "tick.h"
#include <string.h>

Bms_Ctx_t g_bms;

void Ctx_Init(void)
{
    memset(&g_bms, 0, sizeof(g_bms));
}

void Ctx_Update(void)
{
    g_bms.uptime_ms = Tick_Get();
}
