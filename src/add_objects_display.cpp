#include <curobo_rviz/add_objects_display.hpp>
#include <rviz_common/logging.hpp>

namespace add_objects
{
    AddObjectsDisplay::AddObjectsDisplay()
        : Display{}
    {
    }

    AddObjectsDisplay::~AddObjectsDisplay()
    {
        RVIZ_COMMON_LOG_INFO("AddObjectsDisplay::~AddObjectsDisplay()");
    }
} // add_objects