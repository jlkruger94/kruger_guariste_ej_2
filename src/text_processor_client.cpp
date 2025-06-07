#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "kruger_guariste_ej_2/action/process_text.hpp"

class TextProcessorClient : public rclcpp::Node
{
public:
  using ProcessText = kruger_guariste_ej_2::action::ProcessText;
  using GoalHandleProcessText = rclcpp_action::ClientGoalHandle<ProcessText>;

  explicit TextProcessorClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("text_processor_client", options)
  {

    this->client_ptr_ = rclcpp_action::create_client<ProcessText>(
      this,
      "process_text");


    publisher_ = this->create_publisher<std_msgs::msg::String>("completion_message", 10);

    RCLCPP_INFO(this->get_logger(), "Text Processor Client iniciado");
  }

  void send_goal(const std::string & text)
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server no disponible después de esperar");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = ProcessText::Goal();
    goal_msg.input_text = text;

    RCLCPP_INFO(this->get_logger(), "Enviando goal con texto: %s", text.c_str());

    auto send_goal_options = rclcpp_action::Client<ProcessText>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TextProcessorClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TextProcessorClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TextProcessorClient::result_callback, this, _1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ProcessText>::SharedPtr client_ptr_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void goal_response_callback(const GoalHandleProcessText::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal fue rechazado por el servidor");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal aceptado por el servidor, esperando resultado");
    }
  }

  void feedback_callback(
    GoalHandleProcessText::SharedPtr,
    const std::shared_ptr<const ProcessText::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), 
                "Palabra actual: '%s', Palabras restantes: %d",
                feedback->current_word.c_str(), feedback->words_remaining);
  }

  void result_callback(const GoalHandleProcessText::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal completado exitosamente!");
        RCLCPP_INFO(this->get_logger(), "Resultado: %s", result.result->final_message.c_str());
        
        {
          auto msg = std_msgs::msg::String();
          msg.data = "Texto republicado!";
          publisher_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Publicado: %s", msg.data.c_str());
        }
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal fue abortado");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal fue cancelado");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Resultado desconocido");
        return;
    }
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("text_processor_client"), 
                 "Uso: ros2 run kruger_guariste_ej_2 text_processor_client \"texto a procesar\"");
    return 1;
  }

  std::ostringstream text_stream;
  for (int i = 1; i < argc; ++i) {
    text_stream << argv[i];
    if (i < argc - 1) {
      text_stream << " ";
    }
  }
  std::string input_text = text_stream.str();

  auto action_client = std::make_shared<TextProcessorClient>();
  action_client->send_goal(input_text);

  rclcpp::spin(action_client);

  return 0;
}