#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

//start the PI regulator thread
void turn(float alpha);
void pi_regulator_start(void);
uint8_t get_done_right(void);

#endif /* PI_REGULATOR_H */
