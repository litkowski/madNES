#ifndef INPUT_H
#define INPUT_H

#include <cstdint>

enum user_command{NONE = 0, PAUSE = 1};

void signal_input_poll();
uint8_t read_controller_1();
uint8_t read_controller_2();
user_command read_command();

#endif
