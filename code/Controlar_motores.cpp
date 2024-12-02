// main.cpp
#include "Motor.h"
#include "pico_uart_transports.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

// Crear dos objetos Motor con los pines
Motor motor1(25, 2, 3, 4, 5, 6, 64, 50.0f);   // Motor 1
Motor motor2(26, 7, 8, 9, 10, 11, 64, 50.0f); // Motor 2

// Variables globales 
float cmd_motor1 = 0.0f;
float cmd_motor2 = 0.0f;

// ROS2 publishers y subscribers
rcl_subscription_t cmd_subs;
geometry_msgs__msg__Twist cmd_msg;

rcl_publisher_t encoder_pub;
geometry_msgs__msg__Twist encoder_msg;

// Funcion mapeo
float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Callback para el subscriber
void cmd_callback(const void* msgin) {
    const geometry_msgs__msg__Twist* twist_msg = (const geometry_msgs__msg__Twist*)msgin;
    printf("Received commands: linear.x=%f, linear.y=%f\n", twist_msg->linear.x, twist_msg->linear.y);

    // Mapear los comandos a valores de velocidad
    cmd_motor1 = map(twist_msg->linear.x, -3.0f, 3.0f, -100.0f, 100.0f);
    cmd_motor2 = map(twist_msg->linear.y, -3.0f, 3.0f, -100.0f, 100.0f);
}

// Callback para el timer
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    // Controlar los motores
    motor1.set_motor(cmd_motor1);
    motor2.set_motor(cmd_motor2);

    // Leer las revoluciones de los encoders
    float revs_motor1 = 0.0f;
    float revs_motor2 = 0.0f;
    motor1.calculate_revolutions(&revs_motor1);
    motor2.calculate_revolutions(&revs_motor2);

    // Publicar las posiciones de los encoders
    encoder_msg.linear.x = revs_motor1;
    encoder_msg.linear.y = revs_motor2;

    rcl_publish(&encoder_pub, &encoder_msg, NULL);

    // Alternar LED para indicar actividad
    motor1.toggleLED();
    motor2.toggleLED();
}

int main() {
    // Iniciar comunicacion serial con Micro-ROS
    rmw_uros_set_custom_transport(
        true, NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Iniciar ROS 2
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, NULL, &allocator);

    rcl_node_t node;
    rclc_node_init_default(&node, "pico_node", "", &support);

    // Iniciar publisher
    rclc_publisher_init_default(
        &encoder_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "encoder_topic"
    );

    // Inicializar el subscriber
    rclc_subscription_init_default(
        &cmd_subs,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_topic"
    );

    // Inicializar el timer
    rcl_timer_t timer;
    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback
    );

    // Inicializar el ejecutor
    rclc_executor_t executor;
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_subs, &cmd_msg, &cmd_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    // Bucle principal
    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Limpiar recursos
    rcl_publisher_fini(&encoder_pub, &node);
    rcl_subscription_fini(&cmd_subs, &node);
    rcl_node_fini(&node);

    return 0;
}
