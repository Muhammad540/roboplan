#pragma once

#include <string>
#include <variant>
#include <vector>

namespace roboplan {
    /// @brief Robot state expressed in RoboPlan format
    struct RoboPlanRobotState {
        double time{0.0};
        Eigen::VectorXd q;
        Eigen::VectorXd v;
        Eigen::VectorXd tau;
    };

    struct PositionCommand {
        Eigen::VectorXd q;
    };

    struct VelocityCommand {
        Eigen::VectorXd v;
    };

    struct EffortCommand {
        Eigen::VectorXd tau;
    };

    using HardwareCommand = std::variant<PositionCommand, VelocityCommand, EffortCommand>;

    class HardwareInterface {
        public:
            virtual ~HardwareInterface() = default;

            virtual tl::expected<RoboPlanRobotState, std::string> readState() = 0;
            virtual tl::expected<void, std::string> writeCommand(const HardwareCommand& command) = 0;
    };
} // namespace roboplan