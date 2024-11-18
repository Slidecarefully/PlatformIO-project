//Program MD5
#define PROGRAM_MD5 "73cd0e44403384f819d16dc307a22d57"
//Comms configurations
#define MBSERIAL_IFACE Serial
#define MBSERIAL_BAUD 115200
#define MBSERIAL_SLAVE 1
#define MBTCP_MAC 0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD
#define MBTCP_IP 192,168,1,195
#define MBTCP_DNS 8,8,8,8
#define MBTCP_GATEWAY 192,168,1,1
#define MBTCP_SUBNET 255,255,255,0
#define MBTCP_SSID ""
#define MBTCP_PWD "password"
#define MBSERIAL
#define MODBUS_ENABLED


//IO Config
#define PINMASK_DIN 2, 3, 4, 5, 6
#define PINMASK_AIN A0, A1, A2, A3, A4, A5
#define PINMASK_DOUT 7, 8, 12, 13
#define PINMASK_AOUT 9, 10, 11
#define NUM_DISCRETE_INPUT 5
#define NUM_ANALOG_INPUT 6
#define NUM_DISCRETE_OUTPUT 4
#define NUM_ANALOG_OUTPUT 3


//Arduino Libraries


//Pin Array Sizes
