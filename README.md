
# Robô Navegador com ROS2 e Stage

Este repositório contém o código para o trabalho de tópicos em Sistemas Robóticos - Graduação EAUTO 2025 - que implementa um robô diferencial autônomo usando ROS2 e o simulador Stage. O robô é capaz de navegar para múltiplos pontos de destino, evitando obstáculos de forma autônoma.

### Links

link para video do youtube: https://youtu.be/4gyGEZgXPDE

link para o repositorio Git: https://github.com/Gabriel-br2/trab_rosStage_TSR

---

## Descrição do Projeto

O objetivo deste projeto é desenvolver um nó em ROS2 para controlar um robô simulado no ambiente Stage. O robô deve navegar de sua posição inicial `(x=-7, y=-7)` para dois alvos: `(x=7, y=-3)` e `(x=7, y=7)` em qualquer ordem, uma vez que não foi especificado sequencialidade obrigatória. Para isso, ele utiliza dados de um sensor a laser (LIDAR) para detectar e desviar de obstáculos presentes no cenário. 

## Comportamento do Sistema

O sistema opera com base em uma máquina de estados simples que dita o comportamento do robô. O fluxo principal de interação e funcionamento é o seguinte:

1.  **Inicialização**: Ao iniciar, o nó define a pose inicial do robô no mapa, aplicando um *offset* de `(-7.0, -7.0)` em x, y e de `(- 45º)` em rz, aos dados de odometria brutos.  A lista de alvos é carregada, e o robô entra no estado inicial.

2.  **Máquina de Estados**:
    * **`GO_TO_GOAL`**: Este é o estado padrão. O robô calcula a rota para o alvo atual usando um controlador que ajusta a velocidade linear (`v`) e angular (`omega`) para minimizar a distância (`rho`) e o erro de ângulo (`alpha`) em relação ao alvo. Simultaneamente, ele monitora um cone de detecção em sua trajetória. Se um obstáculo entra em um raio de `0.8` metros, o estado muda para `AVOID_OBSTACLE`.
    * **`AVOID_OBSTACLE`**: Neste estado, a navegação para o alvo é pausada. O robô adota uma estratégia de "seguir a parede" (*wall follower*). Ele utiliza os dados do LIDAR para manter uma distância segura (`0.5` metros) da parede mais próxima à sua direita, usando um controlador PD para ajustar sua velocidade angular. Ele continua nesse estado até que o caminho em direção ao alvo esteja livre de obstáculos.
    * **`DONE`**: Uma vez que o robô alcança o primeiro alvo com uma margem de erro permitida, ele passa para o próximo alvo da lista. Após todos os alvos serem alcançados, o robô para e entra no estado `DONE`, finalizando a tarefa.

3.  **Troca de Mensagens ROS2**:
    * **Subscribers**: O nó assina os tópicos `/odom` para receber a posição e orientação e `/base_scan` para as leituras do sensor a laser. 
    * **Publisher**: O nó publica mensagens do tipo `Twist` no tópico `/cmd_vel` para enviar os comandos de velocidade linear e angular ao robô. 

---

## Estrutura do Código

O código-fonte está contido em um único arquivo Python e é estruturado em torno de uma classe principal e funções auxiliares.

* **`Robot` (Classe Principal - `rclpy.node.Node`)**:
    * `__init__()`: Inicializa o nó, os *publishers*, *subscribers* e o *timer* principal. Também define todos os parâmetros de configuração, como a lista de alvos, o *offset* inicial e os ganhos dos controladores.
    * `_callback_odom(msg)`: É a função de *callback* principal, executada a cada nova mensagem de odometria. Ela contém a lógica da máquina de estados (`GO_TO_GOAL`, `AVOID_OBSTACLE`), atualiza a pose do robô e toma decisões de navegação.
    * `_callback_scan(msg)`: Armazena a última mensagem recebida do LIDAR para que possa ser usada em outras partes do código.
    * `wall_follower_strategy()`: Implementa a lógica de desvio de obstáculo, calculando as velocidades necessárias para seguir uma parede a uma distância segura.
    * `check_obstacle_in_path(angle_to_target)`: Verifica se há obstáculos na direção do alvo atual, usando um subconjunto dos dados do LIDAR.
    * `_publish_cmd_vel()`: Publica as velocidades linear e angular no tópico `/cmd_vel` em uma frequência fixa.

* **Funções Auxiliares**:
    * `estimated_Pose(...)`: Converte a pose do robô do frame de odometria (`odom`) para o frame do mundo, aplicando o *offset* inicial.
    * `control(...)`: Implementa o controlador de feedback não-linear para guiar o robô até uma pose de destino.
    * `main()`: Função principal que inicializa o rclpy e o nó `Robot`, mantendo-o em execução.

## Notas

* **Dependências**: O projeto requer uma instalação funcional do ROS2 (humble) e do pacote `stage_ros2`. 
* **Execução**: Para executar o projeto, é necessário primeiro obter o ambiente do ROS2 e, em seguida, iniciar o simulador Stage com o mundo correto, conforme especificado no documento proposto:
  
    ```bash
    ros2 launch stage_ros2 stage.launch.py world:=new_cave enforce_prefixes:=false one_tf_tree:=true
    ```

    Após o simulador estar em execução, o nó do robô pode ser iniciado em um novo terminal.

    Setup do Ros2:
    ```bash 
      cd  ~/ros2_ws
      colcon build
      source ~/ros2_ws/install/setup.bash
      ros2 run trab_stage_pkg robot
    ```
      
* **Sistema de Localização**: O sistema de localização **não é robusto**. Ele depende inteiramente da correção da pose inicial por meio de um *offset* fixo.  Qualquer desvio ou erro acumulado na odometria do simulador afetará diretamente a precisão da navegação, como é possivel observar no video de demonstração em que o erro acumulado da odometria apresentou um desvio significativo ao chegar no ultimo ponto.
