#ifndef __CONFIG_H_
#define __CONFIG_H_

#define JOY_PORT_0					0
#define JOY_PORT_1					1
#define JOY_PORT_2					2
#define JOY_PORT_3					3
#define JOY_PORT_4					4

#define JOY_BTN_X					1
#define JOY_BTN_A					2
#define JOY_BTN_B					3
#define JOY_BTN_Y					4

#define JOY_BTN_LBM					5
#define JOY_BTN_RBM					6
#define JOY_BTN_LTG					7
#define JOY_BTN_RTG					8

#define JOY_SPC_BCK					9  // Back button
#define JOY_SPC_STR					10 // Start button
#define JOY_SPC_LST					11 // Push the left stick in
#define JOY_SPC_RST					12 // Push the right stick in

#define JOY_AXIS_LX					0
#define JOY_AXIS_LY					1
#define JOY_AXIS_RX					2
#define JOY_AXIS_RY					3
#define JOY_AXIS_DX					4
#define JOY_AXIS_DY					5

// Fire talons are CAN
#define PDP_CAN						1
#define TOP_FIRE_CAN_TALON			2
#define	BOT_FIRE_CAN_TALON			3
#define ARM_A_CAN_TALON				4
#define ARM_B_CAN_TALON				5
// Other talons are PWM
#define LEFT_TANK_TALON				0
#define RIGHT_TANK_TALON			1
#define INTAKE_TALON				2
#define WINCH_TALON					3
#define TAPE_TALON					4

// Relay
#define LED_RELAY					0

// Analog
#define GYRO_ANALOG					0

#define PHOTO_DIGITAL				9

#define ALIGN_ROTATE_POWER			0.25f

#define POWER_FUNC_A				2550.f
#define POWER_FUNC_B				349.7569f
#define POWER_FUNC_C				164.4825f
#define POWER_FUNC_D				2750.f

#define PORT_ARM_A					0
#define PORT_ARM_B					0
#define BRIDGE_ARM_A				0
#define BRIDGE_ARM_B				0
#define SALLY_ARM_A_A				-0.4f
#define SALLY_ARM_B_A				0.3f
#define SALLY_ARM_A_B				-0.2f
#define SALLY_ARM_B_B				0.3f
#define FRISE_ARM_A					0
#define FRISE_ARM_B					0

#endif
