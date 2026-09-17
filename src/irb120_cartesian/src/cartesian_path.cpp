#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit/robot_state/conversions.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

struct Punto
{
    double tiempo;
    geometry_msgs::msg::Pose pose;
};


std::vector<Punto> leerCSV(const std::string& archivo)
{
    std::vector<Punto> puntos;

    std::ifstream file(archivo);

    if (!file.is_open())
        throw std::runtime_error("No se pudo abrir el CSV");

    std::string linea;

    getline(file, linea);

    while (getline(file, linea))
    {
        std::stringstream ss(linea);
        std::string dato;
        std::vector<double> valores;

        while (getline(ss, dato, ','))
            valores.push_back(std::stod(dato));

        if (valores.size() != 8)
            continue;

        Punto p;

        p.tiempo = valores[0];

        p.pose.position.x = valores[1];
        p.pose.position.y = valores[2];
        p.pose.position.z = valores[3];

        p.pose.orientation.x = valores[4];
        p.pose.orientation.y = valores[5];
        p.pose.orientation.z = valores[6];
        p.pose.orientation.w = valores[7];

        puntos.push_back(p);
    }

    return puntos;
}


double progreso(
    double x,
    double y,
    double z,
    const Punto& inicio,
    const Punto& final)
{
    double dx = final.pose.position.x - inicio.pose.position.x;
    double dy = final.pose.position.y - inicio.pose.position.y;
    double dz = final.pose.position.z - inicio.pose.position.z;

    double px = x - inicio.pose.position.x;
    double py = y - inicio.pose.position.y;
    double pz = z - inicio.pose.position.z;

    double longitud2 = dx*dx + dy*dy + dz*dz;

    double s = (px*dx + py*dy + pz*dz) / longitud2;

    return std::clamp(s, 0.0, 1.0);
}


double obtenerTiempo(
    double s,
    const std::vector<Punto>& puntos)
{
    double distancia_total =
        std::sqrt(
            std::pow(
                puntos.back().pose.position.x -
                puntos.front().pose.position.x, 2
            ) +
            std::pow(
                puntos.back().pose.position.y -
                puntos.front().pose.position.y, 2
            ) +
            std::pow(
                puntos.back().pose.position.z -
                puntos.front().pose.position.z, 2
            )
        );

    for (size_t i = 1; i < puntos.size(); i++)
    {
        double distancia =
            std::sqrt(
                std::pow(
                    puntos[i].pose.position.x -
                    puntos.front().pose.position.x, 2
                ) +
                std::pow(
                    puntos[i].pose.position.y -
                    puntos.front().pose.position.y, 2
                ) +
                std::pow(
                    puntos[i].pose.position.z -
                    puntos.front().pose.position.z, 2
                )
            );

        double s2 = distancia / distancia_total;

        double distancia_anterior =
            std::sqrt(
                std::pow(
                    puntos[i-1].pose.position.x -
                    puntos.front().pose.position.x, 2
                ) +
                std::pow(
                    puntos[i-1].pose.position.y -
                    puntos.front().pose.position.y, 2
                ) +
                std::pow(
                    puntos[i-1].pose.position.z -
                    puntos.front().pose.position.z, 2
                )
            );

        double s1 = distancia_anterior / distancia_total;

        if (s <= s2)
        {
            double u = (s - s1) / (s2 - s1);

            return puntos[i-1].tiempo +
                   u * (
                       puntos[i].tiempo -
                       puntos[i-1].tiempo
                   );
        }
    }

    return puntos.back().tiempo;
}


void ponerTiempo(
    trajectory_msgs::msg::JointTrajectoryPoint& punto,
    double tiempo)
{
    punto.time_from_start.sec =
        static_cast<int32_t>(std::floor(tiempo));

    punto.time_from_start.nanosec =
        static_cast<uint32_t>(
            (tiempo - std::floor(tiempo)) * 1e9
        );
}


double tiempoPunto(
    const trajectory_msgs::msg::JointTrajectoryPoint& punto)
{
    return punto.time_from_start.sec +
           punto.time_from_start.nanosec * 1e-9;
}


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions opciones;
    opciones.automatically_declare_parameters_from_overrides(true);

    auto node = rclcpp::Node::make_shared(
        "cartesian_path",
        opciones
    );

    std::string archivo;

    node->get_parameter(
        "csv",
        archivo
    );

    std::cout
        << "CSV: "
        << archivo
        << std::endl;

    auto puntos_csv = leerCSV(archivo);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread hilo([&executor]()
    {
        executor.spin();
    });


    moveit::planning_interface::MoveGroupInterface move_group(
        node,
        "manipulator"
    );

    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("tool0");
    move_group.setStartStateToCurrentState();


    std::vector<geometry_msgs::msg::Pose> waypoints;

    // El primer punto ya es la posicion actual
    for (size_t i = 1; i < puntos_csv.size(); i++)
        waypoints.push_back(puntos_csv[i].pose);


    moveit_msgs::msg::RobotTrajectory trayectoria;

    double fraccion =
        move_group.computeCartesianPath(
            waypoints,
            0.005,
            trayectoria,
            true
        );


    std::cout
        << "Trayectoria completada: "
        << fraccion * 100.0
        << " %"
        << std::endl;


    if (fraccion < 0.99)
    {
        std::cout
            << "No se pudo completar la trayectoria"
            << std::endl;

        rclcpp::shutdown();
        hilo.join();

        return 1;
    }


    auto estado = move_group.getCurrentState(2.0);

    auto modelo = move_group.getRobotModel();

    const auto* grupo =
        modelo->getJointModelGroup("manipulator");

    moveit::core::RobotState estado_fk(*estado);


    auto& puntos =
        trayectoria.joint_trajectory.points;


    // Asignar los tiempos del perfil a los puntos de MoveIt
    for (auto& punto : puntos)
    {
        estado_fk.setJointGroupPositions(
            grupo,
            punto.positions
        );

        estado_fk.update();

        const auto& T =
            estado_fk.getGlobalLinkTransform("tool0");

        double s = progreso(
            T.translation().x(),
            T.translation().y(),
            T.translation().z(),
            puntos_csv.front(),
            puntos_csv.back()
        );

        double tiempo =
            obtenerTiempo(
                s,
                puntos_csv
            );

        ponerTiempo(
            punto,
            tiempo
        );
    }


    // Evita tiempos repetidos por redondeo
    for (size_t i = 1; i < puntos.size(); i++)
    {
        double t0 = tiempoPunto(puntos[i-1]);
        double t1 = tiempoPunto(puntos[i]);

        if (t1 <= t0)
            ponerTiempo(
                puntos[i],
                t0 + 0.000001
            );
    }


    size_t n_joints =
        trayectoria.joint_trajectory.joint_names.size();


    // Velocidades
    for (auto& punto : puntos)
        punto.velocities.resize(n_joints, 0.0);


    for (size_t i = 1; i + 1 < puntos.size(); i++)
    {
        double t0 = tiempoPunto(puntos[i-1]);
        double t2 = tiempoPunto(puntos[i+1]);

        double dt = t2 - t0;

        for (size_t j = 0; j < n_joints; j++)
        {
            puntos[i].velocities[j] =
                (
                    puntos[i+1].positions[j] -
                    puntos[i-1].positions[j]
                ) / dt;
        }
    }

// =====================================================
// Guardar posiciones y velocidades articulares en CSV
// =====================================================

// Genera el nombre automaticamente a partir del CSV usado
std::string archivo_salida = archivo;

size_t posicion_extension =
    archivo_salida.rfind(".csv");

if (posicion_extension != std::string::npos)
{
    archivo_salida.replace(
        posicion_extension,
        4,
        "_articulares.csv"
    );
}
else
{
    archivo_salida += "_articulares.csv";
}


std::ofstream csv_salida(archivo_salida);

if (!csv_salida.is_open())
{
    std::cerr
        << "No se pudo crear el CSV"
        << std::endl;
}
else
{
    // Encabezado
    csv_salida
        << "tiempo,"
        << "q1,q2,q3,q4,q5,q6,"
        << "qdot1,qdot2,qdot3,qdot4,qdot5,qdot6\n";


    // Una fila por cada punto de la trayectoria
    for (size_t i = 0; i < puntos.size(); i++)
    {
        double t =
            tiempoPunto(puntos[i]);

        csv_salida << t;


        // Posiciones articulares [rad]
        for (size_t j = 0; j < n_joints; j++)
        {
            csv_salida
                << ","
                << puntos[i].positions[j];
        }


        // Velocidades articulares [rad/s]
        for (size_t j = 0; j < n_joints; j++)
        {
            csv_salida
                << ","
                << puntos[i].velocities[j];
        }

        csv_salida << "\n";
    }


    csv_salida.close();

    std::cout
        << "\nPosiciones y velocidades guardadas en:\n"
        << archivo_salida
        << std::endl;
}

    // Aceleraciones
    for (auto& punto : puntos)
        punto.accelerations.resize(n_joints, 0.0);


    for (size_t i = 1; i + 1 < puntos.size(); i++)
    {
        double t0 = tiempoPunto(puntos[i-1]);
        double t2 = tiempoPunto(puntos[i+1]);

        double dt = t2 - t0;

        for (size_t j = 0; j < n_joints; j++)
        {
            puntos[i].accelerations[j] =
                (
                    puntos[i+1].velocities[j] -
                    puntos[i-1].velocities[j]
                ) / dt;
        }
    }


    std::cout
        << "Tiempo del perfil: "
        << puntos_csv.back().tiempo
        << " s"
        << std::endl;

    std::cout
        << "Tiempo aplicado en MoveIt: "
        << tiempoPunto(puntos.back())
        << " s"
        << std::endl;
    std::cout
        << "\nVelocidad angular maxima por articulacion:\n";

    for (size_t j = 0; j < n_joints; j++)
    {
        double maxima = 0.0;

        for (const auto& punto : puntos)
        {
             maxima = std::max(
             	maxima,
                std::abs(punto.velocities[j])
             );
        }

    std::cout
        << trayectoria.joint_trajectory.joint_names[j]
        << ": "
        << maxima
        << " rad/s"
        << std::endl;
    }

    std::cout
        << "\nAceleracion maxima por articulacion:\n";


    for (size_t j = 0; j < n_joints; j++)
    {
        double maxima = 0.0;

        for (const auto& punto : puntos)
        {
            maxima = std::max(
                maxima,
                std::abs(
                    punto.accelerations[j]
                )
            );
        }

        std::cout
            << trayectoria.joint_trajectory.joint_names[j]
            << ": "
            << maxima
            << " rad/s2"
            << std::endl;
    }


    auto publisher =
        node->create_publisher<
            moveit_msgs::msg::DisplayTrajectory
        >(
            "/display_planned_path",
            10
        );


    moveit_msgs::msg::DisplayTrajectory display;

    moveit::core::robotStateToRobotStateMsg(
        *estado,
        display.trajectory_start
    );

    display.trajectory.push_back(
        trayectoria
    );


    std::this_thread::sleep_for(
        std::chrono::seconds(1)
    );

    publisher->publish(display);


    std::cout
        << "\nTrayectoria enviada a RViz"
        << std::endl;
    std::cout << "Ejecutando trayectoria..." << std::endl;

    move_group.execute(trayectoria);

    std::this_thread::sleep_for(
        std::chrono::seconds(5)
    );


    rclcpp::shutdown();
    hilo.join();

    return 0;
}
