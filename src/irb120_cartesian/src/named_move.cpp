#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <thread>
#include <iostream>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared(
        "named_move",
        options
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread hilo([&]()
    {
        executor.spin();
    });

    std::string target;
    std::string planner;

    node->get_parameter("target", target);
    node->get_parameter("planner", planner);

    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "manipulator"
    );

    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId(planner);
    move_group.setStartStateToCurrentState();

    move_group.setNamedTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool exito = static_cast<bool>(
        move_group.plan(plan)
    );

    if (!exito)
    {
        std::cout << "No se pudo planear hacia " << target << std::endl;

        rclcpp::shutdown();
        hilo.join();

        return 1;
    }

    std::cout << "Ejecutando hacia " << target << std::endl;

    move_group.execute(plan);

    rclcpp::shutdown();
    hilo.join();

    return 0;
}
