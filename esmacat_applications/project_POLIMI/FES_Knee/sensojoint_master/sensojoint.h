
#ifndef ESMACAT_SENSO_JOINT_H
#define ESMACAT_SENSO_JOINT_H

#define	ESMACAT_SENSO_JOINT_VENDOR_ID	0x0000063a // FIXME: vendor and product check is commented out
#define	ESMACAT_SENSO_JOINT_PRODUCT_ID	0x005b9729

#include "slave.h"
#include "ethercat.h"
#include "sensojoint_structs.h"
#include <stdint.h>

class esmacat_sensojoint: public esmacat_slave{
private:
        uint16_t	statusword;
        CoE_state_t  state;
        CoE_mode_t modes_of_operation_display_CoE;
        int8_t	modes_of_operation_display;
        int32_t	position_actual_value;
        int32_t	position_actual_value_rad;

        int32_t	velocity_actual_value;
        double	motor_velocity_actual_value_filt;
        double  prev_motor_velocity_rad_s;
        double prova ;


        int16_t	torque_actual_value;
        double	torque_actual_value_filt;

        int32_t	output_torque_actual_value;
        int32_t	secondary_position_value;
        int32_t	secondary_velocity_value;
        uint32_t    timestamp;
        uint16_t    controlword;
        int8_t	modes_of_operation;
        int16_t	target_torque;
        int32_t	target_position;
        int32_t	target_velocity;
        int32_t	target_output_torque;
        int32_t	target_output_position;
        int32_t	target_output_velocity;
        int16_t	target_torque_offset;

public:
        esmacat_sensojoint();

        double motor_accel_actual_value_filt;

        out_sensojoint_PDO output_PDO;
        in_sensojoint_PDO input_PDO;

        uint32_t esmacat_slave_vendor_id = 0x0000063a;
        uint32_t esmacat_slave_product_id = 0x005b9729;


        void ecat_data_process(uint8_t* ec_slave_outputs,int oloop,uint8_t* ec_slave_inputs,int iloop);
        uint16_t get_statusword();
        CoE_state_t get_CoE_state();

        CoE_mode_t get_modes_of_operation_display_CoE();
        int8_t get_modes_of_operation_display();
        int32_t get_position_actual_value();
        double  get_position_actual_value_rad();

        int32_t get_velocity_actual_value();
        double get_velocity_actual_value_rad_s();

        double get_motor_acceleration_value_filt();

        int16_t get_torque_actual_value();
        double get_torque_actual_value_mNm();

        int32_t get_output_torque_actual_value_mNm();
        double get_output_torque_actual_value_mNm_filt();

        int32_t get_secondary_position_value();
        double get_secondary_position_value_rad();

        int32_t get_secondary_velocity_value_mrpm();
        double get_secondary_velocity_value_rad_s();

        uint32_t get_timestamp();

        void set_controlword(uint16_t value);
        void set_modes_of_operation(int8_t value);
        void set_modes_of_operation_display_CoE(CoE_mode_t value);
        void set_target_torque(int16_t value);
        void set_target_torque_mNm(double desired_torque_mNm);
        void set_target_position(int32_t value);
        void set_target_velocity(int32_t value);
        void set_target_output_torque(int32_t value);
        void set_target_output_position(int32_t value);
        void set_target_output_velocity(int32_t value);
        void set_torque_offset(int16_t value);


        // high level function


        esmacat_err set_impedance_control_rad(float setpoint_rad,float setpoint_mNm);
};

#endif
