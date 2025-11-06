#include "curobo_rviz/arrow_interaction_display.hpp"
#include <rviz_common/logging.hpp>

namespace curobo_rviz
{
    ArrowInteractionDisplay::ArrowInteractionDisplay()
        : Display()
        , node_(nullptr)
        , arrow_interaction_(nullptr)
        , frame_id_property_(nullptr)
        , visible_property_(nullptr)
    {
        // Create properties that will appear in RViz Display panel
        frame_id_property_ = new rviz_common::properties::StringProperty(
            "Reference Frame", "base_0",
            "Frame ID for the interactive marker",
            this, SLOT(updateFrameId()));

        visible_property_ = new rviz_common::properties::BoolProperty(
            "Visible", true,
            "Show or hide the interactive marker",
            this, SLOT(updateVisibility()));
    }

    ArrowInteractionDisplay::~ArrowInteractionDisplay()
    {
    }

    void ArrowInteractionDisplay::onInitialize()
    {
        // Use RViz's main node instead of creating a new one
        // This ensures the node is already being spun by RViz
        node_ = context_->getRos2Node();

        // Create the arrow interaction instance
        arrow_interaction_ = std::make_shared<ArrowInteraction>(node_);

        // Set initial frame ID from property
        std::string frame_id = frame_id_property_->getStdString();
        if (!frame_id.empty()) {
            arrow_interaction_->setFrameId(frame_id);
        }

        RCLCPP_INFO(node_->get_logger(), "ArrowInteractionDisplay initialized with frame: %s",
                    frame_id.c_str());
    }

    void ArrowInteractionDisplay::onEnable()
    {
        if (arrow_interaction_) {
            arrow_interaction_->setVisible(true);
            visible_property_->setBool(true);
        }
    }

    void ArrowInteractionDisplay::onDisable()
    {
        if (arrow_interaction_) {
            arrow_interaction_->setVisible(false);
            visible_property_->setBool(false);
        }
    }

    void ArrowInteractionDisplay::updateFrameId()
    {
        if (arrow_interaction_) {
            std::string new_frame_id = frame_id_property_->getStdString();
            arrow_interaction_->setFrameId(new_frame_id);
            RCLCPP_INFO(node_->get_logger(), "Updated marker frame_id to: %s", new_frame_id.c_str());
        }
    }

    void ArrowInteractionDisplay::updateVisibility()
    {
        if (arrow_interaction_) {
            bool visible = visible_property_->getBool();
            arrow_interaction_->setVisible(visible);
        }
    }

} // namespace curobo_rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(curobo_rviz::ArrowInteractionDisplay, rviz_common::Display)
