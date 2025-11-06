#pragma once

#ifndef CUROBO_RVIZ__ARROW_INTERACTION_DISPLAY_HPP_
#define CUROBO_RVIZ__ARROW_INTERACTION_DISPLAY_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include "curobo_rviz/arrow_interaction.hpp"

namespace curobo_rviz
{
    class ArrowInteractionDisplay : public rviz_common::Display
    {
        Q_OBJECT
    public:
        explicit ArrowInteractionDisplay();
        ~ArrowInteractionDisplay() override;

        // Get the arrow interaction instance (for panel access)
        std::shared_ptr<ArrowInteraction> getArrowInteraction() const { return arrow_interaction_; }

    protected:
        void onInitialize() override;
        void onEnable() override;
        void onDisable() override;

    private Q_SLOTS:
        void updateFrameId();
        void updateVisibility();

    private:
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<ArrowInteraction> arrow_interaction_;

        // RViz properties
        rviz_common::properties::StringProperty* frame_id_property_;
        rviz_common::properties::BoolProperty* visible_property_;
    };
} // namespace curobo_rviz

#endif  // CUROBO_RVIZ__ARROW_INTERACTION_DISPLAY_HPP_
