# HVPS-20kV-version
 @file main.c
 @brief Dual MCPWM Channel Controller via Serial Monitor

 This application controls two independent PWM signals (CH0 and CH1) on an
 ESP32-WROOM-32 using the MCPWM (Motor Control PWM) peripheral driver from
 ESP-IDF v5/v6 new-style API.

 Each channel has:
   - Independent GPIO output pin
   - Independently controllable ON/OFF state
   - Independently configurable frequency (Hz)
   - Independently configurable duty cycle (%)

 Serial command protocol (send via Serial Monitor at 115200 baud):
   ON  <ch>              - Start PWM output on channel 0 or 1
   OFF <ch>              - Stop  PWM output on channel 0 or 1
   FREQ <ch> <hz>        - Set frequency in Hz  (1 – 1,000,000)
   DUTY <ch> <percent>   - Set duty cycle in %  (0.0 – 100.0)
   STATUS                - Print current settings for both channels

 Examples:
   ON 0           -> Start channel 0
   FREQ 1 1000    -> Set channel 1 to 1 kHz
   DUTY 0 25.5    -> Set channel 0 duty to 25.5 %
   OFF 1          -> Stop channel 1

 Hardware wiring (ESP32-WROOM-32):
   Channel 0 PWM output  -> GPIO 18
   Channel 1 PWM output  -> GPIO 19

 Key MCPWM Concepts (ESP-IDF v5+ new API):
   Timer      – Counts at resolution_hz ticks/sec, resets at period_ticks.
                Frequency = resolution_hz / period_ticks
   Operator   – Links a timer to one or more generators.
   Comparator – Fires when timer count == compare_value. Used to create edges.
   Generator  – Defines what happens at timer events (set/clear the output GPIO).

 Dependencies (in idf_component.yml or CMakeLists REQUIRES):
   esp_driver_mcpwm, driver, esp_timer
