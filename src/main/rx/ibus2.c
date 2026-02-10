#include "platform.h"

#ifdef USE_SERIALRX_IBUS

#include "rx/ibus2.h"

#include "rx/ibus.h"

bool ibus2Init(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState)
{
    // Placeholder: IBUS2 currently reuses the IBUS1 parser until a dedicated
    // IBUS2 implementation (with command/response handling) is added.
    return ibusInit(rxConfig, rxRuntimeState);
}

#endif
