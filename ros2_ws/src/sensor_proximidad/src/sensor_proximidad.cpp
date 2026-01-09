#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SensorProximidad : public rclcpp::Node
{
public:
    SensorProximidad() : Node("sensor_proximidad")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        count_ = 0;
        timer_ = this->create_wall_timer(500ms, std::bind(&SensorProximidad::timer_callback, this));
        // Inicialización del sensor de proximidad
        RCLCPP_INFO(this->get_logger(), "Nodo de sensor de proximidad iniciado");

    }   
    // Método para leer datos del sensor
    void leerDatos()
    {
        // Simulación de lectura de datos
        RCLCPP_INFO(this->get_logger(), "Leyendo datos del sensor de proximidad...");
    }
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;

    void publishData()
    {
        auto message = std_msgs::msg::String();
        message.data = "Datos de proximidad #" + std::to_string(count_++);

        // Publicar mensaje
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Publicado: '%s'", message.data.c_str());
    }

    void timer_callback()
    {
        this->publishData();
    }
};  
int main(int argc, char * argv[])
{
    // Inicializar ROS 2
    rclcpp::init(argc, argv);

    // Crear y ejecutar nodo
    auto node = std::make_shared<SensorProximidad>();

    // Spin del nodo
    rclcpp::spin(node);

    // Limpieza
    rclcpp::shutdown();
    return 0;
}