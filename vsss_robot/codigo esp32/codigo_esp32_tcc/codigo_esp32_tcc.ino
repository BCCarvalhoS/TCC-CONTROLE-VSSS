#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <WiFi.h>

// Informações da rede WiFi
const char* ssid = "nome_rede";  // Nome da rede WiFi
const char* password = "senha";  // Senha da rede WiFi

// Informações do servidor ROS
IPAddress server(xxx, xxx, x, xxx); // Endereço IP do servidor ROS (ip addr)
const uint16_t serverPort = 11411;  // Porta do servidor ROS

ros::NodeHandle nh;  // Objeto para comunicação com o ROS

// Definições de hardware para a ponte H L298N Mini
#define MotFwd1 32  // Pino para avanço do Motor A
#define MotRev1 33  // Pino para reverso do Motor A
#define MotFwd2 26  // Pino para avanço do Motor B
#define MotRev2 27  // Pino para reverso do Motor B

// Constantes de cinemática diferencial
const float baseWidth = 0.08;   // Distância entre as rodas em metros (8 cm)
const float wheelRadius = 0.017;  // Raio das rodas em metros (1.25 cm)

// Fator de escala para normalizar a velocidade do motor para PWM
const int fatorEscalaPWM = 1;  // Fator ajustável baseado em testes

// Função de callback para receber comandos de velocidade do ROS
void messageCb(const geometry_msgs::Twist& cmd_vel_msg) {
  float velLinearX = cmd_vel_msg.linear.x;  // Velocidade linear recebida
  float velAngularZ = cmd_vel_msg.angular.z;  // Velocidade angular recebida

  // Convertendo velocidades linear e angular para velocidades individuais dos motores
  float linearVel = velLinearX;  // Velocidade linear desejada
  float angularVel = velAngularZ;  // Velocidade angular desejada

  // Cálculo das velocidades dos motores usando cinemática diferencial
  float velocidadeMotorEsquerdo = (linearVel - (angularVel * baseWidth)) / wheelRadius;
  float velocidadeMotorDireito = (linearVel + (angularVel * baseWidth)) / wheelRadius;

  // Aplicando o fator de escala às velocidades dos motores
  velocidadeMotorEsquerdo = velocidadeMotorEsquerdo * fatorEscalaPWM;
  velocidadeMotorDireito = velocidadeMotorDireito * fatorEscalaPWM;

  // Limitando a velocidade máxima dos motores
  int offset = 30;  // Offset para ajustar o valor PWM
  int velocidadeMaxima = 60 - offset;  // Velocidade máxima permitida para o motor

  // Garantindo que a velocidade dos motores esteja dentro do limite
  velocidadeMotorEsquerdo = constrain(velocidadeMotorEsquerdo, -velocidadeMaxima, velocidadeMaxima);
  velocidadeMotorDireito = constrain(velocidadeMotorDireito, -velocidadeMaxima, velocidadeMaxima);

  // Controlando a direção e a velocidade do Motor A (esquerdo)
  if (velocidadeMotorEsquerdo > 0) {
    analogWrite(MotRev1, LOW);  // Desliga o reverso
    analogWrite(MotFwd1, velocidadeMotorEsquerdo + offset);  // Avança com a velocidade ajustada
  } else {
    analogWrite(MotFwd1, LOW);  // Desliga o avanço
    analogWrite(MotRev1, abs(velocidadeMotorEsquerdo) + offset);  // Reverte com a velocidade ajustada
  }

  // Controlando a direção e a velocidade do Motor B (direito)
  if (velocidadeMotorDireito > 0) {
    analogWrite(MotRev2, LOW);  // Desliga o reverso
    analogWrite(MotFwd2, velocidadeMotorDireito + offset);  // Avança com a velocidade ajustada
  } else {
    analogWrite(MotFwd2, LOW);  // Desliga o avanço
    analogWrite(MotRev2, abs(velocidadeMotorDireito) + offset);  // Reverte com a velocidade ajustada
  }
}

// Subscreve ao tópico do ROS que publica comandos de velocidade para o robô
ros::Subscriber<geometry_msgs::Twist> sub("robot/cmd_vel", &messageCb);

void setup() {
  Serial.begin(115200);  // Inicializa a comunicação serial
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  // Conecta à rede WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  // Espera até que a conexão seja estabelecida
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi conectado");  // Confirma que a conexão foi estabelecida
  Serial.println("Endereço IP: ");
  Serial.println(WiFi.localIP());  // Exibe o endereço IP obtido na rede

  // Inicializa o nó ROS e se inscreve no tópico de comandos de velocidade
  nh.initNode();
  nh.subscribe(sub);

  // Configura a conexão com o servidor ROS
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Configura os pinos dos motores como saída
  pinMode(MotFwd1, OUTPUT);
  pinMode(MotRev1, OUTPUT);
  pinMode(MotFwd2, OUTPUT);
  pinMode(MotRev2, OUTPUT);
}

void loop() {
  nh.spinOnce();  // Mantém a comunicação ativa com o ROS
  //delay(10);  // Opcional: pode ser usado para criar um pequeno atraso no loop
}

