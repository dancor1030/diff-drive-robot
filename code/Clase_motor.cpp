// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <hardware/gpio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include <map>

class Motor {
private:
    // Motor driver pins
    const uint LED_PIN;
    const uint ENA_PIN;  // PWM pin for speed control
    const uint IN1_PIN;  // Direction control pin 1
    const uint IN2_PIN;  // Direction control pin 2

    // Encoder pins
    const uint ENCODER_A_PIN;
    const uint ENCODER_B_PIN;

    // Encoder specifications
    const int TICKS_PER_REV;
    const float GR;

    // PWM slice
    uint pwmSlice;

    // Encoder tick counter
    volatile int32_t encoder_ticks;
    int32_t last_ticks;
    static constexpr float time_interval_sec = 0.1f;  // Timer interval in seconds

    // Low-pass filter variables
    static constexpr float alpha = 0.15f;  // Smoothing factor
    float filtered_rpm;

    // Static map to associate encoder pins with Motor instances
    static std::map<uint, Motor*> motor_instances;

    // Private encoder interrupt handler
    static void encoder_irq_handler(uint gpio, uint32_t events);

    // Non-static method to handle the encoder interrupt
    void handle_encoder_interrupt(uint gpio, uint32_t events);

public:
    // Constructor
    Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
          uint enc_a_pin, uint enc_b_pin,
          int ticks_per_rev = 64, float gear_ratio = 50.0f);

    // Motor control functions
    void set_motor(float speed);
    void calculate_revolutions(float* revs);
    void resetEncoderTicks();
    void toggleLED();
};

#endif  // MOTOR_H
// Motor.cpp

#include "Motor.h"

// Initialize the static map
std::map<uint, Motor*> Motor::motor_instances;

Motor::Motor(uint led_pin, uint ena_pin, uint in1_pin, uint in2_pin,
             uint enc_a_pin, uint enc_b_pin,
             int ticks_per_rev, float gear_ratio)
    : LED_PIN(led_pin), ENA_PIN(ena_pin), IN1_PIN(in1_pin), IN2_PIN(in2_pin),
      ENCODER_A_PIN(enc_a_pin), ENCODER_B_PIN(enc_b_pin),
      TICKS_PER_REV(ticks_per_rev), GR(gear_ratio),
      encoder_ticks(0), last_ticks(0), filtered_rpm(0.0f) {
    // Configure GPIO pins for the motor driver
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // Turn off LED initially

    gpio_init(IN1_PIN);
    gpio_init(IN2_PIN);
    gpio_set_dir(IN1_PIN, GPIO_OUT);
    gpio_set_dir(IN2_PIN, GPIO_OUT);

    // Set up PWM for speed control on ENA_PIN
    gpio_set_function(ENA_PIN, GPIO_FUNC_PWM);
    pwmSlice = pwm_gpio_to_slice_num(ENA_PIN);
    pwm_set_wrap(pwmSlice, 65535);  // Set PWM wrap for 16-bit resolution
    pwm_set_chan_level(pwmSlice, PWM_CHAN_A, 0);  // Set initial duty cycle (stopped)
    pwm_set_enabled(pwmSlice, true);  // Enable PWM

    // Configure encoder pins and interrupts
    gpio_init(ENCODER_A_PIN);
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);
    gpio_pull_up(ENCODER_B_PIN);

    // Associate encoder pins with this instance
    motor_instances[ENCODER_A_PIN] = this;
    motor_instances[ENCODER_B_PIN] = this;

    // Enable interrupts for encoder pins A and B
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &encoder_irq_handler);
    gpio_set_irq_enabled(ENCODER_B_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         true, &encoder_irq_handler);
}

void Motor::encoder_irq_handler(uint gpio, uint32_t events) {
    // Find the motor instance associated with this GPIO pin
    Motor* motor = motor_instances[gpio];
    if (motor != nullptr) {
        motor->handle_encoder_interrupt(gpio, events);
    }
}

void Motor::handle_encoder_interrupt(uint gpio, uint32_t events) {
    bool encoder_a = gpio_get(ENCODER_A_PIN);
    bool encoder_b = gpio_get(ENCODER_B_PIN);

    if (gpio == ENCODER_A_PIN) {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks++;  // Forward direction
        } else {
            encoder_ticks--;  // Reverse direction
        }
    } else if (gpio == ENCODER_B_PIN) {
        if ((encoder_a && !encoder_b) || (!encoder_a && encoder_b)) {
            encoder_ticks--;  // Forward direction
        } else {
            encoder_ticks++;  // Reverse direction
        }
    }
}

void Motor::set_motor(float speed) {
    uint16_t pwm_value = static_cast<uint16_t>(fabs(speed));

    if (speed > 0) {
        // Forward direction
        gpio_put(IN1_PIN, 1);
        gpio_put(IN2_PIN, 0);
    } else if (speed < 0) {
        // Reverse direction
        gpio_put(IN1_PIN, 0);
        gpio_put(IN2_PIN, 1);
    } else {
        // Stop the motor
        gpio_put(IN1_PIN, 0);
        gpio_put(IN2_PIN, 0);
    }

    // Set PWM duty cycle for speed control, scaled to 16-bit resolution
    pwm_set_gpio_level(ENA_PIN, (pwm_value * 65535) / 100);  // Map 0-100% to 16-bit PWM
}

void Motor::calculate_revolutions(float* revs) {
    *revs = static_cast<float>(encoder_ticks) / TICKS_PER_REV;
}

void Motor::resetEncoderTicks() {
    encoder_ticks = 0;
    last_ticks = 0;
    filtered_rpm = 0.0f;
}

void Motor::toggleLED() {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}
