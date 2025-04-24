# Robô Ambiental Chico Mendes - Visualização no Gazebo e RViz

Este repositório contém os arquivos necessários para a **visualização do robô ambiental Chico Mendes** nos ambientes de simulação **Gazebo** e **RViz**, utilizando o simulador Gazebo Classic e o RViz2 no ROS 2 Humble.

## Como utilizar

### Clonando e compilando o pacote

1. Clone este repositório dentro da pasta `src` do seu workspace ROS 2:

```bash
cd ~/ros2_ws/src
git clone https://github.com/alyssonm0/laser_rahcm_visualization.git
```

2. Compile o workspace:

```bash
cd ~/ros2_ws
colcon build --packages-select laser_rahcm_visualization
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

Para executar a simulação no Gazebo:

```bash
ros2 launch laser_rahcm_visualization gazebo.launch.py
```

### 🔄 Comportamento do robô:

- O robô **irá aparecer automaticamente em qualquer mundo que já esteja aberto** no Gazebo.
- Se **nenhum mundo estiver aberto**, o launch irá carregar automaticamente o mundo padrão vazio do Gazebo (`empty.world`) e posicionar o robô lá.

---
