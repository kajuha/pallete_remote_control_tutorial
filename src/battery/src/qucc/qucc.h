#pragma once

#include <string>
#include <queue>

#define SWAP_BYTE 1
#define SHIFT_BIT (SWAP_BYTE*8)

#define SWAP_2BYTE(x) ((x>>SHIFT_BIT)|(x<<SHIFT_BIT))

// QUCC TX
#define QUCC_TX_SUCCESS 1
#define QUCC_TX_FAIL 0

#define QUCC_TX_START_IDX 0
#define QUCC_TX_START_LEN 1
#define QUCC_TX_START_VAL 0xDD

#define QUCC_TX_STATUS_IDX (QUCC_TX_START_IDX+QUCC_TX_START_LEN)
#define QUCC_TX_STATUS_LEN 1
#define QUCC_TX_STATUS_VAL 0xA5

#define QUCC_TX_COMMAND_IDX (QUCC_TX_STATUS_IDX+QUCC_TX_STATUS_LEN)
#define QUCC_TX_COMMAND_LEN 1
#define QUCC_TX_COMMAND_VAL 0x03

#define QUCC_TX_LENGTH_IDX (QUCC_TX_COMMAND_IDX+QUCC_TX_COMMAND_LEN)
#define QUCC_TX_LENGTH_LEN 1
#define QUCC_TX_LENGTH_VAL 0x00

#define QUCC_TX_DATA_IDX (QUCC_TX_LENGTH_IDX+QUCC_TX_LENGTH_LEN)
#define QUCC_TX_DATA_LEN 0
#define QUCC_TX_DATA_VAL 0x00

#define QUCC_TX_CHECKSUM_IDX (QUCC_TX_DATA_IDX+QUCC_TX_DATA_LEN)
#define QUCC_TX_CHECKSUM_LEN 2
#define QUCC_TX_CHECKSUM_VAL 0xFDFF

#define QUCC_TX_END_IDX (QUCC_TX_CHECKSUM_IDX+QUCC_TX_CHECKSUM_LEN)
#define QUCC_TX_END_LEN 1
#define QUCC_TX_END_VAL 0x77

#define QUCC_TX_LEN (QUCC_TX_END_IDX+QUCC_TX_END_LEN)

// QUCC_RX
#define QUCC_RX_SUCCESS 1
#define QUCC_RX_FAIL 0

#define QUCC_RX_VALID 1
#define QUCC_RX_INVALID 0

#define QUCC_RX_START_IDX 0
#define QUCC_RX_START_LEN 1
#define QUCC_RX_START_VAL 0xDD

#define QUCC_RX_COMMAND_IDX (QUCC_RX_START_IDX+QUCC_RX_START_LEN)
#define QUCC_RX_COMMAND_LEN 1
#define QUCC_RX_COMMAND_VAL 0x03

#define QUCC_RX_STATUS_IDX (QUCC_RX_COMMAND_IDX+QUCC_RX_COMMAND_LEN)
#define QUCC_RX_STATUS_LEN 1
#define QUCC_RX_STATUS_VAL 0

#define QUCC_RX_LENGTH_IDX (QUCC_RX_STATUS_IDX+QUCC_RX_STATUS_LEN)
#define QUCC_RX_LENGTH_LEN 1
#define QUCC_RX_LENGTH_VAL 0x1B

#define QUCC_RX_DATA_IDX (QUCC_RX_LENGTH_IDX+QUCC_RX_LENGTH_LEN)
#define QUCC_RX_DATA_LEN QUCC_RX_LENGTH_VAL

#define QUCC_RX_CHECKSUM_IDX (QUCC_RX_DATA_IDX+QUCC_RX_DATA_LEN)
#define QUCC_RX_CHECKSUM_LEN 2
#define QUCC_RX_CHECKSUM_VAL 0

#define QUCC_RX_END_IDX (QUCC_RX_CHECKSUM_IDX+QUCC_RX_CHECKSUM_LEN)
#define QUCC_RX_END_LEN 1
#define QUCC_RX_END_VAL 0x77

#define QUCC_RX_LEN (QUCC_RX_END_IDX+QUCC_RX_END_LEN)

#define QUCC_RX_CHECKSUM_START_IDX (QUCC_RX_STATUS_IDX)
#define QUCC_RX_CHECKSUM_BYTE_LEN (QUCC_RX_STATUS_LEN+QUCC_RX_LENGTH_LEN+QUCC_RX_DATA_LEN)

enum FSM_QUCC_RX {
    START, COMMAND, STATUS, LENGTH, DATA, CHECKSUM, END, OK
};

typedef struct _QuccData {
	uint16_t total_voltage_10mv;
	uint16_t current_10ma;
	uint16_t remaining_capacity_10mah;
	uint16_t norminal_capacity_10mah;
	uint16_t number_of_cycles;
	uint16_t production_date;
	uint16_t balanced_state;
	uint16_t balanced_state_high;
	uint16_t protection_status;
	uint8_t software_version;
	uint8_t rsoc;
	uint8_t fet_control_state;
	uint8_t number_of_battery_strings;
	uint8_t number_of_ntc;
	uint16_t ntc_1st;
	uint16_t ntc_2nd;
	uint16_t ntc_3rd;
	uint16_t ntc_4th;
} QuccData;

typedef struct _QuccInfo {
	double voltage_v;
	double current_a;
	double remaining_capacity_ah;
	double norminal_capacity_ah;
	uint16_t cycles;
	uint16_t production_date;
	uint16_t balanced_low;
	uint16_t balanced_high;
	uint16_t protection_status;
	uint8_t software_version;
	double remaining_capacity_percent;
	uint8_t charging;
	uint8_t discharging;
	uint8_t number_of_battery_strings;
	uint8_t number_of_ntc;
	double celsius_1st;
	double celsius_2nd;
	double celsius_3rd;
	double celsius_4th;
} QuccInfo;

class Qucc {
public:
	std::string serialPort;
	int baudrate;
	std::queue<uint8_t> queSerialRx;

	int fd;
	uint8_t serialBufferRx[BUFSIZ];
	uint8_t serialBufferTx[BUFSIZ];

	QuccData _quccData;
	QuccInfo _quccInfo;

	bool isParsed;
	
	Qucc();
	Qucc(std::string serialPort, int baudrate);

	bool initSerial();
	void closeSerial();
	bool sendQuccCmd();
	bool receiveQuccState(bool enableParsing=true);
	bool parseQuccState();
	int isValidChecksum(uint8_t *recv);
	QuccInfo parseRxData(QuccData quccData);
};