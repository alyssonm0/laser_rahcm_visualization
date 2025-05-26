#include <memory>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <iostream> // Para std::stod e std::string

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

// Usando aliases para simplificar os tipos
using FollowJT = control_msgs::action::FollowJointTrajectory;
using GoalHandleFollowJT = rclcpp_action::ClientGoalHandle<FollowJT>;

class ArmMover : public rclcpp::Node
{
public:
  // Construtor: recebe a seleção do braço, a posição desejada e o tempo para atingir a posição
  ArmMover(const std::string &arm_selection_arg, double position_arg, double time_from_start_arg)
  : Node("arm_mover_action_client"), // Nome do nó
    position_(position_arg),           // Inicializa o membro position_
    time_from_start_(time_from_start_arg) // Inicializa o membro time_from_start_
  {
    // Mapeia nomes de braços para índices (para o array de posições)
    arm_names_map_ = {
      {"front_left", 0},
      {"front_right", 1},
      {"rear_left", 2},
      {"rear_right", 3},
    };

    // Determina se todos os braços devem ser movidos ou um específico
    if (arm_selection_arg == "all_arms") {
        all_arms_ = true;
        this->arm_selection_ = "all_arms"; // Armazena a seleção para fins de log, se necessário
    } else {
        all_arms_ = false;
        this->arm_selection_ = arm_selection_arg; // Armazena o nome do braço específico
    }

    // Cria o cliente de ação para se comunicar com o servidor /arm_controller/follow_joint_trajectory
    action_client_ = rclcpp_action::create_client<FollowJT>(
      this, "arm_controller/follow_joint_trajectory");

    // Espera o servidor de ação ficar disponível (com um timeout)
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) { // Aumentado timeout para 10s
      RCLCPP_ERROR(get_logger(), "Servidor de ação '/arm_controller/follow_joint_trajectory' não disponível após 10s!");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(get_logger(), "Servidor de ação encontrado. Enviando meta...");
    send_goal(); // Envia a meta de trajetória
  }

private:
  rclcpp_action::Client<FollowJT>::SharedPtr action_client_;
  std::map<std::string, size_t> arm_names_map_; // Mapa de nomes de braços para índices
  bool all_arms_;                               // Flag para mover todos os braços
  std::string arm_selection_;                   // Nome do braço selecionado (ou "all_arms")
  double position_;                             // Posição desejada para o(s) braço(s)
  double time_from_start_;                      // Tempo para atingir a posição

  // Função para construir e enviar a meta de trajetória
  void send_goal()
  {
    auto goal_msg = FollowJT::Goal();
    // Define os nomes das juntas que o controlador espera
    goal_msg.trajectory.joint_names = {
      "joint_base_arm_front_left",
      "joint_base_arm_front_right",
      "joint_base_arm_rear_left",
      "joint_base_arm_rear_right"
    };


    std::vector<double> target_positions(4, 0.0); // Assume 4 juntas

    if (all_arms_) {
      // Se "all_arms", define a posição desejada para todas as juntas
      for (size_t i = 0; i < target_positions.size(); ++i) {
        target_positions[i] = position_;
      }
      RCLCPP_INFO(get_logger(), "Movendo todos os braços para %.3f rad em %.1f segundos.", position_, time_from_start_);
    } else {
      // Se um braço específico, verifica se o nome é válido
      if (arm_names_map_.count(arm_selection_)) {
        size_t joint_index = arm_names_map_[arm_selection_];
        target_positions[joint_index] = position_; // Define a posição para o braço selecionado
        // As outras posições permanecem 0.0 (ou poderiam ser lidas do estado atual se necessário)
        RCLCPP_INFO(get_logger(), "Movendo braço '%s' (junta: %s) para %.3f rad em %.1f segundos. Outras juntas para 0.0.",
                    arm_selection_.c_str(), goal_msg.trajectory.joint_names[joint_index].c_str(), position_, time_from_start_);
      } else {
        RCLCPP_ERROR(get_logger(), "Seleção de braço inválida: '%s'. Braços válidos: front_left, front_right, rear_left, rear_right, all_arms.", arm_selection_.c_str());
        rclcpp::shutdown();
        return;
      }
    }

    // Cria um ponto na trajetória
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_);
    goal_msg.trajectory.points.push_back(point);

    // Configura callbacks para a resposta da meta e o resultado da ação
    auto send_goal_options = rclcpp_action::Client<FollowJT>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ArmMover::on_goal_response, this, std::placeholders::_1);
    send_goal_options.result_callback =
      std::bind(&ArmMover::on_result, this, std::placeholders::_1);
   

    RCLCPP_INFO(get_logger(), "Enviando meta de trajetória para o controlador...");
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  // Callback para quando o servidor responde à solicitação de meta
  void on_goal_response(GoalHandleFollowJT::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Meta rejeitada pelo servidor de ação.");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(get_logger(), "Meta aceita pelo servidor, aguardando resultado...");
    }
  }

  // Callback para quando a ação é concluída (com sucesso ou falha)
  void on_result(const GoalHandleFollowJT::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Trajetória concluída com sucesso!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Meta abortada pelo servidor.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(get_logger(), "Meta cancelada.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "Resultado desconhecido da meta.");
        break;
    }
    RCLCPP_INFO(get_logger(), "Erro da trajetória (código do controlador): %d (%s)", 
                result.result->error_code, get_error_string(result.result->error_code).c_str());
    rclcpp::shutdown(); // Desliga o nó após a conclusão
  }

  // Função auxiliar para converter códigos de erro do FollowJointTrajectory em strings
  std::string get_error_string(int error_code) {
    switch (error_code) {
      case control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL:
        return "SUCCESSFUL";
      case control_msgs::action::FollowJointTrajectory::Result::INVALID_GOAL:
        return "INVALID_GOAL";
      case control_msgs::action::FollowJointTrajectory::Result::INVALID_JOINTS:
        return "INVALID_JOINTS";
      case control_msgs::action::FollowJointTrajectory::Result::OLD_HEADER_TIMESTAMP:
        return "OLD_HEADER_TIMESTAMP";
      case control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED:
        return "PATH_TOLERANCE_VIOLATED";
      case control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED:
        return "GOAL_TOLERANCE_VIOLATED";
      default:
        return "UNKNOWN_ERROR_CODE";
    }
  }


};

// Função principal
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Valores padrão
  double position_goal = 0.2; // radianos
  std::string arm_to_move = "all_arms";
  double time_to_reach = 6.0; // segundos

  // Processa argumentos da linha de comando
  if (argc >= 2) {
    try {
      position_goal = std::stod(argv[1]);
    } catch (const std::invalid_argument& ia) {
      std::cerr << "Argumento de posição inválido: " << argv[1] << std::endl;
      return 1;
    } catch (const std::out_of_range& oor) {
      std::cerr << "Argumento de posição fora do intervalo: " << argv[1] << std::endl;
      return 1;
    }
  }
  if (argc >= 3) {
    arm_to_move = argv[2];
  }
  if (argc >= 4) {
    try {
      time_to_reach = std::stod(argv[3]);
    } catch (const std::invalid_argument& ia) {
      std::cerr << "Argumento de tempo inválido: " << argv[3] << std::endl;
      return 1;
    } catch (const std::out_of_range& oor) {
      std::cerr << "Argumento de tempo fora do intervalo: " << argv[3] << std::endl;
      return 1;
    }
  }

  std::cout << "Tentando mover: " << arm_to_move 
            << " para posição: " << position_goal 
            << " rad em " << time_to_reach << "s" << std::endl;

  auto arm_mover_node = std::make_shared<ArmMover>(arm_to_move, position_goal, time_to_reach);
  rclcpp::spin(arm_mover_node); 

  rclcpp::shutdown();
  return 0;
}
