
#include <AP_Common/Location.h>

#include "Copter.h"
#include "AP_Rally.h"

bool AP_Rally_Copter::is_valid(const Location &rally_point) const
{
#if AC_FENCE == ENABLED
    if (!copter.fence.check_destination_within_fence(rally_point)) {
        return false;
    }
#endif
    return true;
}
