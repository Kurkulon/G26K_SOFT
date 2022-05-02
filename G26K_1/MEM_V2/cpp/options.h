#ifndef _OPTIONS_H
#define _OPTIONS_H

#define OPTIONS_LEVEL_B_MAX	0x03FF
#define OPTIONS_LEVEL_B_MIN	0
#define OPTIONS_LEVEL_M_MAX	0x03FF
#define OPTIONS_LEVEL_M_MIN	0

typedef enum
{
	TELEMETRY_TX_BAUD_RATE_X1 = 1,
	TELEMETRY_TX_BAUD_RATE_X2,
	TELEMETRY_TX_BAUD_RATE_X3,
	TELEMETRY_TX_BAUD_RATE_X4,
} options_telemetry_baud_rate_type;

//typedef struct __attribute__ ((packed))
//{
//	u16 level_b;
//	u16 level_m;
//} options_nnk_type;


typedef struct __attribute__ ((packed))
{
	u16 serial;
	u16 baud_rate;
	
	u16 level_b;
	u16 level_m;
	u16 gen_freq;
	u16 win_count;
	u16 win_time;
	u16 crc;

} options_type;

extern options_type options;

extern void Options_Init();

inline void Options_Set_Serial(u16 serial) { options.serial = serial; }
inline u16 Options_Get_Serial() { return options.serial; }
inline u16 Options_Get_Telemetry_Baud_Rate() { return options.baud_rate; }
inline u16 Options_Get_Level_B() {	return options.level_b; }
inline u16 Options_Get_Level_M() { return options.level_m; }

extern bool Options_Set_Telemetry_Baud_Rate(u16 baud_rate);
extern void Options_Set_Level_B(u16 level);
extern void Options_Set_Level_M(u16 level);
extern void Options_Save();

#endif

