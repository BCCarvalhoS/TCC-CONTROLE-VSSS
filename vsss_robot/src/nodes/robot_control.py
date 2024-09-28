#!/usr/bin/env python3

import math

import rospy
import tf_conversions
import tf2_ros
from geometry_msgs.msg import Twist, Vector3


def get_control_vel(transform, kp_x, kp_th):
    """Lei de controle proporcional para atingir uma *posição*."""
    # Calcula o erro em x e y (diferença de posição entre o robô e o objetivo)
    error_x = transform.transform.translation.x
    error_y = transform.transform.translation.y
    # Calcula a distância do erro (distância até o objetivo)
    distance_error = math.sqrt(error_x**2 + error_y**2)
    # Calcula o erro de orientação (ângulo entre o robô e o objetivo)
    heading_error = math.atan2(error_y, error_x)
    # Calcula a velocidade linear proporcional ao erro de distância, com um limite máximo de 0.35
    lin_vel = min(kp_x * distance_error, 0.35)
    # Calcula a velocidade angular proporcional ao erro de orientação, com um limite máximo de 1.5
    ang_vel = min(kp_th * heading_error, 1.5)
    # Cria a mensagem de velocidade de controle
    control_vel = Twist(linear=Vector3(lin_vel, 0, 0),
                            angular=Vector3(0, 0, ang_vel))
    # Se o erro de distância for menor que 0.05, para o robô
    if (distance_error < 0.05):
      control_vel = Twist(linear=Vector3(0, 0, 0),
                            angular=Vector3(0, 0, 0))
   
    return control_vel


def main():
    """Nó de controle do robô."""
    # Inicia o nó e obtém os parâmetros
    rospy.init_node('robot_control')
    robot_tag = rospy.get_param('~robot_tag', 'robot')
    goal_tag = rospy.get_param('~goal_tag', 'goal')
    kp_x = rospy.get_param('~kp_x', 0.5 ) # Fator de proporcionalidade para a velocidade linear
    kp_th = rospy.get_param('~kp_th', 1)  # Fator de proporcionalidade para a velocidade angular
    controller_rate = rospy.get_param('~controller_rate', 10)  # Frequência de controle

    # Listener de TF (para acompanhar transformações entre o robô e o objetivo)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # Publicador da velocidade de controle
    control_vel_pub = rospy.Publisher(f'{robot_tag}/cmd_vel', Twist, queue_size=10)

    # Controla a taxa de execução do loop de controle
    rate = rospy.Rate(controller_rate)
    while not rospy.is_shutdown():
        try:
            # Tenta obter a transformação entre o robô e o objetivo
            transform = tf_buffer.lookup_transform(robot_tag, goal_tag, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Em caso de erro, espera o próximo ciclo
            rate.sleep()
            print("bla")  # Mensagem de erro
            continue
        # Calcula a velocidade de controle baseada na transformação
        control_vel = get_control_vel(transform=transform, kp_x=kp_x, kp_th=kp_th)
        # Publica a velocidade de controle
        control_vel_pub.publish(control_vel)
        rate.sleep()  # Aguarda o próximo ciclo


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass  # Exceção se o nó for interrompido

