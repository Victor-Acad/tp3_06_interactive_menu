/*
 * Copyright (c) 2023 Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * @file   : task_menu.c
 * @date   : Set 26, 2023
 * @author : Juan Manuel Cruz <jcruz@fi.uba.ar> <jcruz@frba.utn.edu.ar>
 * @version	v1.0.0
 */

/********************** inclusions *******************************************/
/* Project includes. */
#include "main.h"

/* Demo includes. */
#include "logger.h"
#include "dwt.h"

/* Application & Tasks includes. */
#include "board.h"
#include "app.h"
#include "task_menu_attribute.h"
#include "task_menu_interface.h"
#include "display.h"
#include "motor_attribute.h"

/********************** macros and definitions *******************************/
#define G_TASK_MEN_CNT_INI			0ul
#define G_TASK_MEN_TICK_CNT_INI		0ul

#define DEL_MEN_XX_MIN				0ul
#define DEL_MEN_XX_MED				50ul
#define DEL_MEN_XX_MAX				500ul

/********************** internal data declaration ****************************/
task_menu_dta_t task_menu_dta =
	{DEL_MEN_XX_MIN,
	 ST_MEN_XX_MAIN,
	 EV_MEN_ENT_IDLE,
	 false,
	 {
	 	{OFF, 0, LEFT, OFF, 0, LEFT},
	 	{OFF, 0, LEFT, OFF, 0, LEFT}
	 },
	 0};

#define MENU_DTA_QTY	(sizeof(task_menu_dta)/sizeof(task_menu_dta_t))

/********************** internal functions declaration ***********************/

/********************** internal data definition *****************************/
const char *p_task_menu 		= "Task Menu (Interactive Menu)";
const char *p_task_menu_ 		= "Non-Blocking & Update By Time Code";

/********************** external data declaration ****************************/
uint32_t g_task_menu_cnt;
volatile uint32_t g_task_menu_tick_cnt;

/********************** external functions definition ************************/
void task_menu_init(void *parameters)
{
	task_menu_dta_t *p_task_menu_dta;
	task_menu_st_t	state;
	task_menu_ev_t	event;
	bool b_event;

	/* Print out: Task Initialized */
	LOGGER_LOG("  %s is running - %s\r\n", GET_NAME(task_menu_init), p_task_menu);
	LOGGER_LOG("  %s is a %s\r\n", GET_NAME(task_menu), p_task_menu_);

	g_task_menu_cnt = G_TASK_MEN_CNT_INI;

	/* Print out: Task execution counter */
	LOGGER_LOG("   %s = %lu\r\n", GET_NAME(g_task_menu_cnt), g_task_menu_cnt);

	init_queue_event_task_menu();

	/* Update Task Actuator Configuration & Data Pointer */
	p_task_menu_dta = &task_menu_dta;

	/* Print out: Task execution FSM */
	state = p_task_menu_dta->state;
	LOGGER_LOG("   %s = %lu", GET_NAME(state), (uint32_t)state);

	event = p_task_menu_dta->event;
	LOGGER_LOG("   %s = %lu", GET_NAME(event), (uint32_t)event);

	b_event = p_task_menu_dta->flag;
	LOGGER_LOG("   %s = %s\r\n", GET_NAME(b_event), (b_event ? "true" : "false"));

	/* Print out: Initial motor states */
	for (uint8_t i = 0; i < MAX_MOTORS; i++) {
	    motor_t *m = &p_task_menu_dta->motors[i];
	    LOGGER_LOG("   motor[%u]: power = %s, speed = %u, spin = %s\r\n",
	               	   i,
	               	   (m->power == ON ? "ON" : "OFF"),
	               	   m->speed,
	               	   (m->spin == LEFT ? "LEFT" : "RIGHT")
				   );
	}

	cycle_counter_init();
	cycle_counter_reset();

	displayInit( DISPLAY_CONNECTION_GPIO_4BITS );

    displayCharPositionWrite(0, 0);
	displayStringWrite("TdSE Bienvenidos");

	displayCharPositionWrite(0, 1);
	displayStringWrite("Test Nro: ");

	g_task_menu_tick_cnt = G_TASK_MEN_TICK_CNT_INI;
}

void task_menu_update(void *parameters)
{
	task_menu_dta_t *p_task_menu_dta;
	bool b_time_update_required = false;
	char menu_str[8];

	/* Update Task Menu Counter */
	g_task_menu_cnt++;

	/* Protect shared resource (g_task_menu_tick) */
	__asm("CPSID i");	/* disable interrupts*/
    if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt)
    {
    	g_task_menu_tick_cnt--;
    	b_time_update_required = true;
    }
    __asm("CPSIE i");	/* enable interrupts*/

    while (b_time_update_required)
    {
		/* Protect shared resource (g_task_menu_tick) */
		__asm("CPSID i");	/* disable interrupts*/
		if (G_TASK_MEN_TICK_CNT_INI < g_task_menu_tick_cnt)
		{
			g_task_menu_tick_cnt--;
			b_time_update_required = true;
		}
		else
		{
			b_time_update_required = false;
		}
		__asm("CPSIE i");	/* enable interrupts*/

    	/* Update Task Menu Data Pointer */
		p_task_menu_dta = &task_menu_dta;

    	if (DEL_MEN_XX_MIN < p_task_menu_dta->tick)
		{
			p_task_menu_dta->tick--;
		}
		else
		{
			snprintf(menu_str, sizeof(menu_str), "%lu", (g_task_menu_cnt/1000ul));
			displayCharPositionWrite(10, 1);
			displayStringWrite(menu_str);

			p_task_menu_dta->tick = DEL_MEN_XX_MAX;

			if (true == any_event_task_menu())
			{
				p_task_menu_dta->flag = true;
				p_task_menu_dta->event = get_event_task_menu();
			}

			switch (p_task_menu_dta->state)
			{
				// TODO: Replace debug LOGGER_LOG instances with actual writes to the display.

				case ST_MEN_XX_MAIN:

					if ((true == p_task_menu_dta->flag) && (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event))
					{
						p_task_menu_dta->flag = false;
						p_task_menu_dta->state = ST_MEN_XX_MOTOR;
						LOGGER_LOG("Entering motor menu for motor %d.\n\n", p_task_menu_dta->current_motor);
					}

					break;

				case ST_MEN_XX_MOTOR:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							if ((p_task_menu_dta->current_motor + 1) == MAX_MOTORS)
							{
								p_task_menu_dta->current_motor = 0;
							}
							else
							{
								p_task_menu_dta->current_motor++;
							}
							LOGGER_LOG("Current motor changed to %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_POWER;
							LOGGER_LOG("Entering power menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_MAIN;
							LOGGER_LOG("Going back to main menu.\n\n");

							/* Print motor states */
							for (uint8_t i = 0; i < MAX_MOTORS; i++) {
							    motor_t *m = &p_task_menu_dta->motors[i];
							    LOGGER_LOG("   motor[%u]: power = %s, speed = %u, spin = %s\r\n",
							               	   i,
							               	   (m->power == ON ? "ON" : "OFF"),
							               	   m->speed,
							               	   (m->spin == LEFT ? "LEFT" : "RIGHT")
										   );
							}
						}
					}

					break;

				case ST_MEN_XX_POWER:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->state = ST_MEN_XX_SPEED;
							LOGGER_LOG("Switching to speed menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_POWER_OPT;
							LOGGER_LOG("Entering power setup menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_MOTOR;
							LOGGER_LOG("Going back to motor menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				case ST_MEN_XX_SPEED:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->state = ST_MEN_XX_SPIN;
							LOGGER_LOG("Switching to spin menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_SPEED_OPT;
							LOGGER_LOG("Entering speed setup menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_MOTOR;
							LOGGER_LOG("Going back to motor menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				case ST_MEN_XX_SPIN:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->state = ST_MEN_XX_POWER;
							LOGGER_LOG("Switching to power menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_SPIN_OPT;
							LOGGER_LOG("Entering spin setup menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_MOTOR;
							LOGGER_LOG("Going back to motor menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				case ST_MEN_XX_POWER_OPT:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].power_tmp = (p_task_menu_dta->motors[p_task_menu_dta->current_motor].power_tmp + 1) % 2;
							LOGGER_LOG("Power value changed to %s.\n\n", (p_task_menu_dta->motors[p_task_menu_dta->current_motor].power_tmp == ON ? "ON" : "OFF"));
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].power = p_task_menu_dta->motors[p_task_menu_dta->current_motor].power_tmp;
							LOGGER_LOG("Power value assigned. Motor %d is now %s.\n\n", p_task_menu_dta->current_motor, (p_task_menu_dta->motors[p_task_menu_dta->current_motor].power == ON ? "ON" : "OFF"));
							// Here we would raise an event for the motor actuator (output pins).
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_POWER;
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].power_tmp = p_task_menu_dta->motors[p_task_menu_dta->current_motor].power;
							LOGGER_LOG("Going back to power menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				case ST_MEN_XX_SPEED_OPT:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed_tmp = (p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed_tmp + 1) % 10;
							LOGGER_LOG("Speed value changed to %d.\n\n", p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed_tmp);
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed = p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed_tmp;
							LOGGER_LOG("Speed value assigned. Motor %d's speed is now %d.\n\n", p_task_menu_dta->current_motor, p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed);
							// Here we would raise an event for the motor actuator (output pins).
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_SPEED;
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed_tmp = p_task_menu_dta->motors[p_task_menu_dta->current_motor].speed;
							LOGGER_LOG("Going back to speed menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				case ST_MEN_XX_SPIN_OPT:

					if (p_task_menu_dta->flag)
					{
						p_task_menu_dta->flag = false;

						if (EV_MEN_NEX_ACTIVE == p_task_menu_dta->event) {
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin_tmp = (p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin_tmp + 1) % 2;
							LOGGER_LOG("Spin value changed to %s.\n\n", (p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin_tmp == RIGHT ? "RIGHT" : "LEFT"));
						}
						else if (EV_MEN_ENT_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin = p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin_tmp;
							LOGGER_LOG("Spin value assigned. Motor %d is now spinning %s.\n\n", p_task_menu_dta->current_motor, (p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin == RIGHT ? "RIGHT" : "LEFT"));
							// Here we would raise an event for the motor actuator (output pins).
						}
						else if (EV_MEN_ESC_ACTIVE == p_task_menu_dta->event)
						{
							p_task_menu_dta->state = ST_MEN_XX_SPIN;
							p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin_tmp = p_task_menu_dta->motors[p_task_menu_dta->current_motor].spin;
							LOGGER_LOG("Going back to spin menu for motor %d.\n\n", p_task_menu_dta->current_motor);
						}
					}

					break;

				default:

					p_task_menu_dta->tick  = DEL_MEN_XX_MIN;
					p_task_menu_dta->state = ST_MEN_XX_MAIN;
					p_task_menu_dta->event = EV_MEN_ENT_IDLE;
					p_task_menu_dta->flag  = false;

					break;
			}
		}
	}
}

/********************** end of file ******************************************/
