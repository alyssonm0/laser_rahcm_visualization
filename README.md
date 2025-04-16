# Robô Ambiental Chico Mendes - Visualização no Gazebo e RViz

Este repositório contém os arquivos necessários para a **visualização do robô ambiental Chico Mendes** nos ambientes de simulação **Gazebo** e **RViz**.

## Como utilizar

### RViz

Para visualizar corretamente o robô no **RViz**, siga os passos abaixo:

```bash
cd ~/<your_workspace>/src
git clone https://github.com/alyssonm0/laser_rahcm_visualization.git
colcon build
source install/setup.bash
ros2 launch laser_rahcm_visualisation display.launch.py
