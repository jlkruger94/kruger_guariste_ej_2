#include <functional>
#include <memory>
#include <thread>
#include <sstream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "kruger_guariste_ej_2/action/process_text.hpp"

class TextProcessorServer : public rclcpp::Node
{
public:
  using ProcessText = kruger_guariste_ej_2::action::ProcessText;
  using GoalHandleProcessText = rclcpp_action::ServerGoalHandle<ProcessText>;

  explicit TextProcessorServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("text_processor_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<ProcessText>(
      this,
      "process_text",
      std::bind(&TextProcessorServer::handle_goal, this, _1, _2),
      std::bind(&TextProcessorServer::handle_cancel, this, _1),
      std::bind(&TextProcessorServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("final_message", 10);

    RCLCPP_INFO(this->get_logger(), "Text Processor Server iniciado");
  }

private:
  rclcpp_action::Server<ProcessText>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ProcessText::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Recibido goal con texto: %s", goal->input_text.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleProcessText> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Recibida petición de cancelación");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleProcessText> goal_handle)
  {
    using namespace std::placeholders;

    std::thread{std::bind(&TextProcessorServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleProcessText> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Republicando...");
    rclcpp::Rate loop_rate(1); // 1 Hz
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<ProcessText::Feedback>();
    auto result = std::make_shared<ProcessText::Result>();

    // Separar texto en palabras
    std::istringstream iss(goal->input_text);
    std::vector<std::string> words;
    std::string word;
    
    while (iss >> word) {
      words.push_back(word);
    }

    // Procesar cada palabra
    for (size_t i = 0; i < words.size(); ++i) {
      // Verificar si fue cancelado
      if (goal_handle->is_canceling()) {
        result->final_message = "Procesamiento cancelado";
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal cancelado");
        return;
      }

      // Preparar feedback
      feedback->current_word = words[i];
      feedback->words_remaining = words.size() - i - 1;
      
      // Publicar feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Enviando palabra: %s (quedan %d)", 
                  feedback->current_word.c_str(), feedback->words_remaining);

      loop_rate.sleep();
    }

    // Verificar si fue cancelado antes de finalizar
    if (goal_handle->is_canceling()) {
      result->final_message = "Procesamiento cancelado";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal cancelado");
      return;
    }

    // Marcar como completado
    result->final_message = "Procesamiento completado exitosamente";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal completado exitosamente");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<TextProcessorServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}