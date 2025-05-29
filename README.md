# Robô Ambiental Chico Mendes - Visualização no Gazebo e RViz

Este repositório contém os arquivos necessários para a **visualização do robô ambiental Chico Mendes** nos ambientes de simulação **Gazebo** e **RViz**, utilizando o simulador Gazebo Classic e o RViz2 no ROS 2 Humble.

## Pré-requisitos

- **Sistema Operacional**: Ubuntu 22.04 LTS  
- **ROS 2**: Humble Hawksbill  
- **Gazebo**: Classic 11

## Instalação

#### 1. ROS 2 Humble

O guia oficial de [Instalação do ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) deve ser seguido. Para se familiarizar mais com o sistema, os [Tutoriais do ROS 2](https://docs.ros.org/en/humble/Tutorials.html) são recomendados.

#### 2. Gazebo Classic 11

Instale:

 ```
  curl -sSL http://get.gazebosim.org | sh
  ```
 Para verificar, execute:

 ```
  gazebo
  ```

Assim como no ROS 2, os [Tutoriais do Gazebo](https://classic.gazebosim.org/tutorials) são altamente recomendados para começar.  

## Como utilizar

### Clonando e compilando o pacote

1. Clone este repositório dentro da pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/alyssonm0/laser_rahcm_visualization.git
git clone https://github.com/LASER-Robotics/laser_rahcm_simulation.git
```

2. Compile o workspace:

```bash
cd ~/ros2_ws
colcon build 
```

3. Fonte o setup:

```bash
source install/setup.bash
```

---

## Visualização no **RViz2**

Para abrir o RViz com o modelo URDF do robô:

```bash
ros2 launch laser_rahcm_visualization display.launch.py
```

---

## Simulação no **Gazebo**

Para executar a simulação no Gazebo, em um terminal:

```bash
ros2 launch laser_rahcm_simulation rahcm_simulation.launch.py
```

E em outro terminal:
```bash
ros2 launch laser_rahcm_visualization gazebo.launch.py
```
