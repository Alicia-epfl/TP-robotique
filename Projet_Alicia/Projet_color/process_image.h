#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
void find_color_2(uint8_t *buffer);
uint16_t get_line_position(void);
void process_image_start(void);
uint32_t find_color(uint8_t *buffer, uint8_t value);

#endif /* PROCESS_IMAGE_H */
