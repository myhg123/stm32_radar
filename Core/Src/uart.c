/*
 * uart.c
 *
 *  Created on: Mar 11, 2024
 *      Author: myhg1
 */

#include "uart.h"
#include <stdio.h>
UART_HandleTypeDef *myHuart;

#define rxBufferMax 255

int rxBufferGp; //get pointer(read)
int rxBufferPp; // put pointer (write)
uint8_t rxBuffer[rxBufferMax];
uint8_t rxChar;

// init device
void initUart(UART_HandleTypeDef *inHuart) {
	myHuart = inHuart;
	HAL_UART_Receive_IT(myHuart, &rxChar, 1);
}

// process received charactor
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	rxBuffer[rxBufferPp++] = rxChar;
	rxBufferPp %= rxBufferMax;
	HAL_UART_Receive_IT(myHuart, &rxChar, 1);
}

// get charator from buffer
int16_t getChar() {
	int16_t result;
	if (rxBufferGp == rxBufferPp)
		return -1;
	result = rxBuffer[rxBufferGp++];
	rxBufferGp %= rxBufferMax;
	return result;
}

//바이너리 데이터 전송
void binaryTransmit(protocol_t inData) {
	uint8_t txBuffer[] = { STX, 0, 0, 0, 0, 0, 0, 0, ETX };
	//데이터 복사
	//memcpy(&txBuffer[1],&inData, 6);
	txBuffer[1] = inData.id | 0x80;
	txBuffer[2] = inData.command | 0x80;
	txBuffer[3] = inData.data | 0x80;
	txBuffer[4] = inData.data >> 7 | 0x80;
	txBuffer[5] = inData.data >> 14 | 0x80;
	txBuffer[6] = inData.data >> 21 | 0x80;
	//CRC 계산
	for (int i = 0; i < 7; i++)
		txBuffer[7] += txBuffer[i];
	//전송
	HAL_UART_Transmit(myHuart, txBuffer, sizeof(txBuffer), 10);

}
//Packet 송신부
void transmitPacket(protocol_t data) {
	//사전준비
	uint8_t txBuffer[] = { STX, 0, 0, 0, 0, ETX };
	txBuffer[1] = data.command;
	txBuffer[2] = (data.data >> 7) | 0x80;
	txBuffer[3] = (data.data & 0x7f) | 0x80;
	//CRC 계산
	txBuffer[4] = txBuffer[0] + txBuffer[1] + txBuffer[2] + txBuffer[3];
	//데이터 전송
	HAL_UART_Transmit(myHuart, txBuffer, sizeof(txBuffer), 1);
	//데이터 전송 완료 대기
	while (HAL_UART_GetState(myHuart) == HAL_UART_STATE_BUSY_TX
			|| HAL_UART_GetState(myHuart) == HAL_UART_STATE_BUSY_TX_RX)
		;
}

//Packet 수신부
protocol_t receivePacket() {
	protocol_t result;
	uint8_t buffer[6];
	uint8_t count = 0;
	uint32_t timeout;

	int16_t ch = getChar();
	memset(&result, 0, sizeof(buffer));
	if (ch == STX) {
		buffer[count++] = ch;
		timeout = HAL_GetTick();	//타임 아웃 시작, gettick은 1ms 의 시간을 측정해준다.
		while (ch != ETX) {
			//stx를 보낸후 etx가 안돌아 올수 있으니 timeout을 통해 특정시간이 지나면 이곳을 빠져나가게 한다.
			ch = getChar();
			if (ch != -1) {
				buffer[count++] = ch;
			}
			//타임아웃 계산
			if (HAL_GetTick() - timeout >= 2)//2ms를 체크하기 위한 시간
				return result;
		}
		//CRC검사
		uint8_t crc = 0;
		for (int i = 0; i < 4; i++)
			crc += buffer[i];
		if (crc != buffer[4])
			return result;
		//수신완료후 파싱(pasing)
		result.command = buffer[1];
		result.data = buffer[3] &0x7f;
		result.data |= (buffer[2] & 0x7f)<<7;
	}
	return result;
}

int _write(int file, char *p, int len) {
	HAL_UART_Transmit(myHuart, p, len, 10);
	return len;
}
