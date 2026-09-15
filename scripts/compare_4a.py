#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint


# Nombres de las articulaciones
JOINTS = [
    'joint_1',
    'joint_2',
    'joint_3',
    'joint_4',
    'joint_5',
    'joint_6'
]

# Pose HOME
HOME = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0
]

# Pose PRE-PICK
PRE_PICK = [
    -0.8314709566249745,
     1.0682651987297038,
    -0.5169242849147218,
    -0.07499808730959998,
     0.9855451188798028,
     0.0035715753131571703
]


class PlannerTest(Node):

    def __init__(self):

        super().__init__('planner_test')

        self.client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path'
        )

        print("Esperando servicio de MoveIt...")

        if not self.client.wait_for_service(timeout_sec=5.0):
            print("ERROR: No se encontró /plan_kinematic_path")
            raise RuntimeError("MoveIt no está disponible")

        print("Servicio encontrado.")


    def test(self, planner):

        print()
        print("--------------------------------")
        print("Probando:", planner)
        print("--------------------------------")

        req = GetMotionPlan.Request()

        request = req.motion_plan_request

        # Grupo y planner
        request.group_name = 'manipulator'
        request.pipeline_id = 'ompl'
        request.planner_id = planner

        request.allowed_planning_time = 5.0
        request.num_planning_attempts = 1

        # Estado inicial HOME
        request.start_state.joint_state.name = JOINTS
        request.start_state.joint_state.position = HOME

        # Objetivo PRE-PICK
        goal = Constraints()

        for joint_name, joint_position in zip(JOINTS, PRE_PICK):

            constraint = JointConstraint()

            constraint.joint_name = joint_name
            constraint.position = joint_position

            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0

            goal.joint_constraints.append(constraint)

        request.goal_constraints = [goal]

        print("Enviando solicitud de planeación...")

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=10.0
        )

        # Verificar respuesta
        if not future.done():
            print("ERROR: MoveIt no respondió.")
            return

        if future.result() is None:
            print("ERROR: La respuesta del servicio está vacía.")
            return

        result = future.result().motion_plan_response

        # MoveIt SUCCESS = 1
        if result.error_code.val != 1:
            print(
                "Planeación fallida. Código:",
                result.error_code.val
            )
            return

        print("Trayectoria encontrada.")

        points = result.trajectory.joint_trajectory.points

        print("Número de puntos:", len(points))

        # Longitud total en espacio articular
        length = 0.0

        for i in range(1, len(points)):

            suma = 0.0

            for q_actual, q_anterior in zip(
                points[i].positions,
                points[i - 1].positions
            ):

                diferencia = q_actual - q_anterior

                suma += diferencia ** 2

            length += math.sqrt(suma)

        print()
        print("RESULTADO")
        print("Planner:", planner)
        print("Longitud de trayectoria:", length, "rad")
        print("Tiempo de planeación:", result.planning_time, "s")


def main():

    print("Iniciando comparación 4A...")

    rclpy.init()

    try:

        node = PlannerTest()

        # RRTConnect
        node.test(
            'RRTConnectkConfigDefault'
        )

        # RRT*
        node.test(
            'RRTstarkConfigDefault'
        )

        node.destroy_node()

    except Exception as error:

        print()
        print("ERROR:")
        print(error)

    finally:

        rclpy.shutdown()

    print()
    print("Fin de la comparación.")


if __name__ == '__main__':
    main()
