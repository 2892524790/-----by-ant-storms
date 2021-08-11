#ifndef __GET_GUDGE_MEASURE_H__
#define __GET_GUDGE_MEASURE_H__

#include "judgement_info.h"
#include "protocol.h"
#include "string.h"





extern const ext_game_status_t 					*get_game_state_t(void);
extern const ext_game_result_t 					*get_game_result_t(void);
extern const ext_game_robot_HP_t 				*get_game_robot_survivors_t(void);
extern const ext_event_data_t 					*get_event_data_t(void);
extern const ext_supply_projectile_action_t 	*get_supply_projectile_action_t(void);
extern const ext_supply_projectile_booking_t 	*get_supply_projectile_booking_t(void);
extern const ext_game_robot_status_t 			*get_game_robot_state_t(void);
extern const ext_power_heat_data_t 				*get_power_heat_data_t(void);
extern const ext_buff_t 						*get_buff_musk_t(void);
extern const aerial_robot_energy_t 				*get_robot_energy_t(void);
extern const ext_robot_hurt_t 					*get_robot_hurt_t(void);
extern const ext_shoot_data_t 					*get_shoot_data_t(void);
	

#endif







