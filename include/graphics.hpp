#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <cstdint>
#include <string>

// Externally available functions
void Init_Master_Palette (std::string filename);
void Init_Graphics_And_IO ();
void push_frame_to_screen ();

#endif
