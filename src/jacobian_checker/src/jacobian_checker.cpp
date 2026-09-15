#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>

#include <Eigen/Core>

#include <iostream>
#include <iomanip>
#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared(
        "jacobian_checker",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    // Cambia "manipulator" si tu grupo tiene otro nombre
    const std::string GROUP = "manipulator";

    moveit::planning_interface::MoveGroupInterface move_group(node, GROUP);

    // Leer estado actual del robot
    auto robot_state = move_group.getCurrentState(10.0);

    if (!robot_state)
    {
        RCLCPP_ERROR(node->get_logger(), "No se pudo obtener el estado actual.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    const moveit::core::JointModelGroup* joint_model_group =
        robot_state->getJointModelGroup(GROUP);

    // Mostrar q1...q6
    std::vector<double> q;
    robot_state->copyJointGroupPositions(joint_model_group, q);

    std::cout << "\nConfiguracion articular [rad]:\n";

    for (size_t i = 0; i < q.size(); i++)
    {
        std::cout << "q" << i + 1 << " = "
                  << std::fixed << std::setprecision(6)
                  << q[i] << "\n";
    }

    // Link final del grupo
    const auto& link_names = joint_model_group->getLinkModelNames();
    const std::string tip_link = link_names.back();

    const moveit::core::LinkModel* link =
        robot_state->getLinkModel(tip_link);

    // Jacobiano
    Eigen::MatrixXd J;

    bool success = robot_state->getJacobian(
        joint_model_group,
        link,
        Eigen::Vector3d::Zero(),
        J);

    if (!success)
    {
        RCLCPP_ERROR(node->get_logger(), "No se pudo calcular el Jacobiano.");
    }
    else
    {
        std::cout << "\nLink usado: " << tip_link << "\n";

        std::cout << "\nJacobiano MoveIt2:\n\n";
        std::cout << J << "\n";
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
