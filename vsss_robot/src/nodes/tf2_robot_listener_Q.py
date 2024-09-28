#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import tf.transformations

if __name__ == '__main__':
    # Inicializa o nó ROS chamado 'tf2_robot_listener'
    rospy.init_node('tf2_robot_listener')

    # Cria um buffer para armazenar as transformações
    tfBuffer = tf2_ros.Buffer()
    # Cria um listener para ouvir as transformações em tempo real
    listener = tf2_ros.TransformListener(tfBuffer)

    # Cria um publicador para a posição do robô no tópico '/robot_pose'
    robot_pose_pub = rospy.Publisher('/robot_pose', geometry_msgs.msg.Pose, queue_size=1)

    # Define a taxa de repetição em 10 Hz
    rate = rospy.Rate(10.0)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            # Tenta buscar a transformação entre 'origem' e 'robot'
            trans = tfBuffer.lookup_transform('origem', 'robot', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Caso ocorra erro na busca da transformação, o código espera o próximo ciclo
            rate.sleep()
            continue

        # Extraindo os valores do quaternion (representação de rotação)
        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )
        
        # Convertendo o quaternion para ângulos de Euler (roll, pitch, yaw)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Criando a mensagem de Pose para publicar a posição do robô
        msg = geometry_msgs.msg.Pose()
        msg.position.x = trans.transform.translation.x  # Define a posição x do robô
        msg.position.y = trans.transform.translation.y  # Define a posição y do robô
        msg.position.z = 0  # Define z como 0 porque você só está interessado em x e y

        # Convertendo o valor de Yaw de volta para quaternion para manter a rotação
        new_quaternion = tf.transformations.quaternion_from_euler(0, 0, euler[2])
        msg.orientation.x = new_quaternion[0]
        msg.orientation.y = new_quaternion[1]
        msg.orientation.z = new_quaternion[2]
        msg.orientation.w = new_quaternion[3]

        # Publica a pose do robô no tópico '/robot_pose'
        robot_pose_pub.publish(msg)

        # Aguarda até o próximo ciclo de controle (10 Hz)
        rate.sleep()


