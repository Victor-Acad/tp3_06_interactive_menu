/*
 *
 * @file   : motor_attribute.h
 * @date   : May 23, 2025
 *
 */

#ifndef MOTOR_ATTRIBUTE_H
#define MOTOR_ATTRIBUTE_H

/********************** CPP guard ********************************************/
#ifdef __cplusplus
extern "C" {
#endif

/********************** defines **********************************************/
#define MAX_MOTORS 2

/********************** data types *******************************************/
typedef enum { OFF = 0, ON = 1 } power_state_t;
typedef enum { LEFT = 0, RIGHT = 1 } spin_dir_t;

typedef struct {
    power_state_t power;
    uint8_t speed;
    spin_dir_t spin;
    power_state_t power_tmp;
    uint8_t speed_tmp;
    spin_dir_t spin_tmp;
} motor_t;

/********************** End of CPP guard *************************************/
#ifdef __cplusplus
}
#endif

#endif // MOTOR_ATTRIBUTE_H

/********************** end of file ******************************************/
