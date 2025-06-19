// NAME: PN5180ISO14443.h
//
// DESC: ISO14443 protocol on NXP Semiconductors PN5180 module for Arduino.
//
// Copyright (c) 2019 by Dirk Carstensen. All rights reserved.
//
// This file is part of the PN5180 library for the Arduino environment.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// #define DEBUG 1

#include <Arduino.h>
#include "PN5180ISO14443.h"
#include <PN5180.h>
#include "Debug.h"

PN5180ISO14443::PN5180ISO14443(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin)
	: PN5180(SSpin, BUSYpin, RSTpin)
{
}

bool PN5180ISO14443::setupRF()
{
	PN5180DEBUG(F("Loading RF-Configuration...\n"));
	if (loadRFConfig(0x00, 0x80))
	{ // ISO14443 parameters
		PN5180DEBUG(F("done.\n"));
	}
	else
		return false;

	PN5180DEBUG(F("Turning ON RF field...\n"));
	if (setRF_on())
	{
		PN5180DEBUG(F("done.\n"));
	}
	else
		return false;

	return true;
}

uint16_t PN5180ISO14443::rxBytesReceived()
{
	uint32_t rxStatus;
	uint16_t len = 0;
	readRegister(RX_STATUS, &rxStatus);
	// Lower 9 bits has length
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}
/*
 * buffer : must be 10 byte array
 * buffer[0-1] is ATQA
 * buffer[2] is sak
 * buffer[3..6] is 4 byte UID
 * buffer[7..9] is remaining 3 bytes of UID for 7 Byte UID tags
 * kind : 0  we send REQA, 1 we send WUPA
 *
 * return value: the uid length:
 * -	zero if no tag was recognized
 * -	single Size UID (4 byte)
 * -	double Size UID (7 byte)
 * -	triple Size UID (10 byte) - not yet supported
 */
uint8_t PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind)
{
	uint8_t cmd[7];
	uint8_t uidLength = 0;
	// Load standard TypeA protocol
	if (!loadRFConfig(0x0, 0x80))
		return 0;

	// OFF Crypto
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF))
		return 0;
	// Clear RX CRC
	if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))
		return 0;
	// Clear TX CRC
	if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))
		return 0;
	// Send REQA/WUPA, 7 bits in last byte
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	if (!sendData(cmd, 1, 0x07))
		return 0;
	// READ 2 bytes ATQA into  buffer
	if (!readData(2, buffer))
		return 0;
	// Send Anti collision 1, 8 bits in last byte
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (!sendData(cmd, 2, 0x00))
		return 0;
	// Read 5 bytes, we will store at offset 2 for later usage
	if (!readData(5, cmd + 2))
		return 0;
	// Enable RX CRC calculation
	if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01))
		return 0;
	// Enable TX CRC calculation
	if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01))
		return 0;
	// Send Select anti collision 1, the remaining bytes are already in offset 2 onwards
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	if (!sendData(cmd, 7, 0x00))
		return 0;
	// Read 1 byte SAK into buffer[2]
	if (!readData(1, buffer + 2))
		return 0;
	// Check if the tag is 4 Byte UID or 7 byte UID and requires anti collision 2
	// If Bit 3 is 0 it is 4 Byte UID
	if ((buffer[2] & 0x04) == 0)
	{
		// Take first 4 bytes of anti collision as UID store at offset 3 onwards. job done
		for (int i = 0; i < 4; i++)
			buffer[3 + i] = cmd[2 + i];
		uidLength = 4;
	}
	else
	{
		// Take First 3 bytes of UID, Ignore first byte 88(CT)
		if (cmd[2] != 0x88)
			return 0;
		for (int i = 0; i < 3; i++)
			buffer[3 + i] = cmd[3 + i];
		// Clear RX CRC
		if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))
			return 0;
		// Clear TX CRC
		if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))
			return 0;
		// Do anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		if (!sendData(cmd, 2, 0x00))
			return 0;
		// Read 5 bytes. we will store at offset 2 for later use
		if (!readData(5, cmd + 2))
			return 0;
		// first 4 bytes belongs to last 4 UID bytes, we keep it.
		for (int i = 0; i < 4; i++)
		{
			buffer[6 + i] = cmd[2 + i];
		}
		// Enable RX CRC calculation
		if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01))
			return 0;
		// Enable TX CRC calculation
		if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01))
			return 0;
		// Send Select anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		if (!sendData(cmd, 7, 0x00))
			return 0;
		// Read 1 byte SAK into buffer[2]
		if (!readData(1, buffer + 2))
			return 0;
		uidLength = 7;
	}

	return uidLength;
}

// Работаем непосредственно с картой минуя PN5180
bool PN5180ISO14443::transceiveRF(uint8_t *txData, uint8_t txLen, uint8_t *rxData, uint8_t maxRxLen, uint8_t *actualRxLen)
{
	// 1. Переводим в IDLE
	writeRegister(PN5180_COMMAND, 0x00000000);

	// 2. Очищаем прерывания
	clearIRQStatus(0xFFFFFFFF);

	// 3. Передаём данные в буфер TX
	if (!sendData(txData, txLen))
	{
		PN5180DEBUG(F("Failed to send RF data\n"));
		return false;
	}

	// 4. Устанавливаем TRANSCEIVE
	writeRegister(PN5180_COMMAND, 0x09000000); // TRANSCEIVE

	// 5. Ждём RX IRQ
	if (!waitForIRQ(RX_IRQ_STAT, 100))
	{
		PN5180DEBUG(F("Timeout waiting for RX_IRQ\n"));
		return false;
	}

	// 6. Читаем ответ
	uint8_t len = readRFResponse(rxData, maxRxLen);
	if (actualRxLen)
		*actualRxLen = len;

	return (len > 0);
}

bool PN5180::waitForIRQ(uint32_t mask, uint16_t timeout)
{
	unsigned long start = millis();

	while (millis() - start < timeout)
	{
		uint32_t irq = getIRQStatus();
		if (irq & mask)
		{
			clearIRQStatus(mask); // не забудь сбросить, если сработало
			return true;
		}
	}
	return false; // таймаут
}

bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer)
{
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Send mifare command 30,blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	if (!sendData(cmd, 2, 0x00))
	{
		Serial.print(F("Ошибка чтения блока "));
		Serial.println(blockno, HEX);
		return false;
	}
	// Check if we have received any data from the tag
	delay(5);
	len = rxBytesReceived();
	if (len == 16)
	{
		// READ 16 bytes into buffer
		if (readData(16, buffer))
		{
			// Выводим только одну страницу (4 байта)
			Serial.print(F("--- Содержимое страницы 0x"));
			Serial.print(blockno, HEX);
			Serial.println(F(" ---"));
			char hexStr[4]; // "XX\0"
			for (int i = 0; i < 4; i++)
			{
				snprintf(hexStr, sizeof(hexStr), "%02X", buffer[i]);
				Serial.print(hexStr);
				if (i < 3)
					Serial.print(":");
			}
			Serial.println();
			success = true;
		}
		else
		{
			Serial.print(F("Ошибка чтения блока "));
			Serial.println(blockno, HEX);
		}
	}
	else
	{
		Serial.print(F("Ошибка чтения блока "));
		Serial.println(blockno, HEX);
	}
	return success;
}

uint8_t PN5180ISO14443::mifareBlockWrite16(uint8_t blockno, uint8_t *buffer)
{
	uint8_t cmd[1];
	// Clear RX CRC
	writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE);

	// Mifare write part 1
	cmd[0] = 0xA0;
	cmd[1] = blockno;
	sendData(cmd, 2, 0x00);
	readData(1, cmd);

	// Mifare write part 2
	sendData(buffer, 16, 0x00);
	delay(10);

	// Read ACK/NAK
	readData(1, cmd);

	// Enable RX CRC calculation
	writeRegisterWithOrMask(CRC_RX_CONFIG, 0x1);
	return cmd[0];
}

uint8_t PN5180ISO14443::mifareUltralightWrite(uint8_t block, uint8_t *data4)
{
	uint8_t cmd[6];
	cmd[0] = 0xA2;	// WRITE команда для Ultralight
	cmd[1] = block; // Адрес блока (page)
	memcpy(&cmd[2], data4, 4);

	// Отправляем команду
	if (!sendData(cmd, 6, 0x00))
		return 0xFF; // Ошибка отправки

	// Выводим информацию о блоке и данных
	Serial.print(F("Запись блока 0x"));
	Serial.print(block, HEX);
	Serial.print(F(": "));
	for (int i = 0; i < 4; i++)
	{
		if (i > 0)
			Serial.print(":");
		if (data4[i] < 0x10)
			Serial.print("0");
		Serial.print(data4[i], HEX);
	}
	Serial.println();

	uint8_t ack = 0;
	if (!readData(1, &ack))
		return 0xFE; // Ошибка чтения

	return ack; // Возвращаем код ответа
}

bool PN5180ISO14443::mifareHalt()
{
	uint8_t cmd[1];
	// mifare Halt
	cmd[0] = 0x50;
	cmd[1] = 0x00;
	sendData(cmd, 2, 0x00);
	return true;
}

uint8_t PN5180ISO14443::readCardSerial(uint8_t *buffer)
{

	uint8_t response[10];
	uint8_t uidLength;
	// Always return 10 bytes
	// Offset 0..1 is ATQA
	// Offset 2 is SAK.
	// UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
	// UID 7 bytes : offset 3 to 9 is UID
	for (int i = 0; i < 10; i++)
		response[i] = 0;
	uidLength = activateTypeA(response, 1);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
		return 0;
	// check for valid uid
	if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00))
		return 0;
	if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF))
		return 0;
	for (int i = 0; i < 10; i++)
		buffer[i] = response[i];

	mifareHalt();
	return uidLength;
}

uint8_t PN5180ISO14443::readCardSerial_ATQA_SAK(uint8_t *buffer)
{
	uint8_t response[10];
	uint8_t uidLength;
	// Always return 10 bytes
	// Offset 0..1 is ATQA
	// Offset 2 is SAK.
	// UID 4 bytes : offset 3 to 6 is UID, offset 7 to 9 to Zero
	// UID 7 bytes : offset 3 to 9 is UID
	for (int i = 0; i < 10; i++)
		response[i] = 0;
	uidLength = activateTypeA(response, 1);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
		return 0;
	// check for valid uid
	if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00))
		return 0;
	if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF))
		return 0;
	for (int i = 0; i < 10; i++)
	{
		buffer[i] = response[i];
	};

		// Читаем версию
	uint8_t versionData[8];
	if (mifareGetVersion(versionData))
	{
		// Дополнительно можешь разобрать структуру по байтам:
		Serial.print(F("Vendor ID: "));
		Serial.println(versionData[0], HEX);
		Serial.print(F("Product Type: "));
		Serial.println(versionData[1], HEX);
		// и так далее...
	}
	
	// Аутентификация PWD_AUTH
	// uint8_t password[4] = {0xD1, 0xF7, 0x34, 0x85}; //  твой пароль
	uint8_t password[4] = {0xFF, 0xFF, 0xFF, 0xFF}; //  пароль по умолчанию
	uint8_t pack_read[2];

	if (mifarePwdAuth(password, pack_read))
	{
		Serial.print(F("Аутентификация прошла успешно! PACK: "));
		Serial.print(pack_read[0], HEX);
		Serial.print(":");
		Serial.println(pack_read[1], HEX);
	}
	else
	{
		Serial.println(F("Аутентификация не удалась."));
		return 0;
	}



	// Читаем подпись
	uint8_t sig[32];
	if (mifareReadSignature(sig))
	{
		Serial.println(F("Подпись успешно считана!"));
	}

	// Прочитаем  блок
	uint8_t blockData[16];
	mifareBlockRead(0x0F, blockData);


	// mifareBlockRead(0x11, blockData);
	// mifareBlockRead(0x12, blockData);
	// mifareBlockRead(0x13, blockData);
	// mifareBlockRead(0x0F, blockData);
	// uint8_t data0F[4] = {0x01, 0x02, 0x03, 0x04};
	// mifareUltralightWrite(0x0F, data0F);
	// mifareBlockRead(0x0F, blockData);
	// if (mifareBlockRead(0x0F, blockData))
	// {
	// 	Serial.println(F("--- Содержимое блока 0x0F ---"));
	// 	char hexStr[4]; // "XX\0"
	// 	for (int i = 0; i < 4; i++)
	// 	{
	// 		snprintf(hexStr, sizeof(hexStr), "%02X", blockData[i]);
	// 		Serial.print(hexStr);
	// 		if (i < 3)
	// 			Serial.print(":");
	// 	}
	// 	Serial.println();
	// }
	// else
	// {
	// 	Serial.println(F("Ошибка чтения блока 0x0F"));
	// 	return 0;
	// }

	/* Запись защищённого блока */
	// uint8_t dataBuf[4] = {0x00, 0x00, 0x00, 0x00}; // 4 байта для одной страницы
	// if (writeProtectedBlock(0x0F, dataBuf))
	// {
	// 	Serial.println(F("Запись защищённого блока прошла успешно!"));
	// }
	// else
	// {
	// 	Serial.println(F("Не удалось записать защищённый блок!"));
	// }

	/* Настройка защиты Mifare Ultralight*/
	// uint8_t data[4] = {0x00, 0x00, 0x00, 0x00};
	// uint8_t data1[4] = {0x01, 0x02, 0x03, 0x04};

	// // Страница 0x10: CFG0 → AUTH0 с какой страницы начинается защита
	// uint8_t cfg0[4] = {0x00, 0x00, 0x00, 0x00}; // AUTH0 = 0x00 — защита с 1-й страницы
	// uint8_t cfg0[4] = {0x00, 0x00, 0x00, 0x0F}; // AUTH0 = 0x0F — защита с 15-й страницы
	// uint8_t cfg0[4] = {0x00, 0x00, 0x00, 0xFF}; // AUTH0 = 0xFF — нет защиты

	// // Страница 0x11: CFG1 защита
	// // AUTHLIM лучше не использовать карта блокируется навсегда!
	// uint8_t cfg1[4] = {0x00, 0x05, 0x00, 0x00}; // PROT = 0, AUTHLIM=0  → защита только от записи; неудачные попытки аутентификации неограничены
	// uint8_t cfg1[4] = {0x80, 0x05, 0x00, 0x00}; // PROT = 1, AUTHLIM=0 → защита чтения и записи; неудачные попытки аутентификации неограничены
	// !!!uint8_t cfg1[4] = {0x87, 0x05, 0x00, 0x00}; // PROT=1, AUTHLIM=3 → защита чтения и записи; !!!!!!максимум 7 неудачные попытки аутентификации (от 1 до 7 попыток 0x81…0x87) лучше не использовать карта блокируется навсегда!!!!

	// Страница 0x12 — устанавливаем пароль
	// uint8_t pwd[4] = {0xFF, 0xFF, 0xFF, 0xFF}; 

	// Страница 0x13 — устанавливаем PACK (последние 2 байта должны быть 0x00)
	// uint8_t pack[4] = {0x00, 0x00, 0x00, 0x00};

	// mifareUltralightWrite(0x09, data1);
	// mifareUltralightWrite(0x0F, data);
	//                                    обязательный порядок записи страниц
	// mifareUltralightWrite(0x12, pwd); //1/ Пароль для защиты
	// mifareUltralightWrite(0x13, pack); //2/ PACK для защиты
	// mifareUltralightWrite(0x11, cfg1); //3/ PROT=1 → защита чтения и записи
	// mifareUltralightWrite(0x10, cfg0); //4/ AUTH0=0x0F → защита со страницы ...

	// uint8_t password[5] = {0x1B, 0x12, 0x34, 0x56, 0x78}; // Команда + 4 байта PWD
	// uint8_t pack[2] = {0};
	/* Конец защиты Mifare Ultralight*/

	mifareHalt();
	return uidLength;
}

bool PN5180ISO14443::isCardPresent()
{
	uint8_t buffer[10];
	return (readCardSerial(buffer) >= 4);
}

// bool PN5180ISO14443::mifareUltralightPwdAuth(uint8_t *pwd, uint8_t *pack_out)
// {
// 	uint8_t cmd[5];
// 	cmd[0] = 0x1B;			 // Команда PWD_AUTH
// 	memcpy(&cmd[1], pwd, 4); // Копируем 4 байта пароля в команду

// 	if (!sendData(cmd, 5, 0x00))
// 		return false; // Ошибка отправки

// 	delay(5); // Небольшая задержка (чтобы карта успела подумать)

// 	uint8_t response[2];
// 	if (!readData(2, response))
// 		return false; // Не получили ответ

// 	if (pack_out)
// 	{
// 		pack_out[0] = response[0]; // PACK[0]
// 		pack_out[1] = response[1]; // PACK[1]
// 	}

// 	return true; // Успешная аутентификация
// }

bool PN5180ISO14443::readSignature(uint8_t *sigBuf, uint8_t &nak)
{
	uint8_t cmd[2] = {0x3C, 0x00}; // без CRC!
	uint8_t recv[33] = {0};		   // 32 байта сигнатуры + 1 байт NAK

	if (!transceiveCommand(cmd, sizeof(cmd), recv, sizeof(recv)))
		return false;

	memcpy(sigBuf, recv, 32);
	nak = recv[32];
	return true;
}

// bool PN5180ISO14443::writeProtectedBlock(uint8_t blockno, uint8_t *buffer)
// {
// 	uint8_t pwd[4] = {0x00, 0x00, 0x00, 0x00}; // Установленный пароль
// 	uint8_t expectedPACK[2] = {0x00, 0x00};	   // PACK, который ты записал ранее
// 	uint8_t packOut[2] = {0};

// 	// Шаг 1: Аутентификация
// 	if (!mifareUltralightPwdAuth(pwd, packOut))
// 	{
// 		Serial.println(F("PWD_AUTH провален!"));
// 		return false;
// 	}

// 	// Шаг 2: Проверим PACK
// 	if (packOut[0] != expectedPACK[0] || packOut[1] != expectedPACK[1])
// 	{
// 		Serial.print(F("PACK неверный! Получено: "));
// 		Serial.print(packOut[0], HEX);
// 		Serial.print(" ");
// 		Serial.println(packOut[1], HEX);
// 		return false;
// 	}

// 	Serial.println(F("Аутентификация успешна! Пишем блок..."));

// 	// Шаг 3: Запись блока, переданного через blockno
// 	if (!mifareUltralightWrite(blockno, buffer))
// 	{
// 		Serial.println(F("Ошибка записи защищённого блока!"));
// 		return false;
// 	}

// 	// Шаг 4: Отладочный вывод
// 	Serial.print(F("Блок 0x"));
// 	Serial.print(blockno, HEX);
// 	Serial.print(F(" записан: "));
// 	for (int i = 0; i < 4; i++)
// 	{
// 		Serial.print(buffer[i], HEX);
// 		if (i < 3)
// 			Serial.print(":");
// 	}
// 	Serial.println();

// 	return true;
// }

bool PN5180ISO14443::mifareGetVersion(uint8_t *versionBuffer)
{
	uint8_t cmd = 0x60; // GET_VERSION

	if (!sendData(&cmd, 1, 0x00))
	{
		Serial.println(F("Ошибка при отправке GET_VERSION"));
		return false;
	}

	delay(5); // Короткая задержка для получения ответа

	uint16_t len = rxBytesReceived();
	if (len != 8)
	{
		Serial.print(F("Ожидалось 8 байт, получено: "));
		Serial.println(len);
		return false;
	}

	if (!readData(8, versionBuffer))
	{
		Serial.println(F("Ошибка чтения данных GET_VERSION"));
		return false;
	}

	Serial.println(F("Версия чипа (GET_VERSION):"));
	for (int i = 0; i < 8; i++)
	{
		Serial.print("0x");
		if (versionBuffer[i] < 0x10)
			Serial.print("0");
		Serial.print(versionBuffer[i], HEX);
		if (i < 7)
			Serial.print(" ");
	}
	Serial.println();

	return true;
}

bool PN5180ISO14443::mifareReadSignature(uint8_t *sigBuffer)
{
	uint8_t cmd[2] = {0x3C, 0x00}; // READ_SIG и адрес

	// Отправка команды
	if (!sendData(cmd, 2, 0x00))
	{
		Serial.println(F("Ошибка отправки READ_SIG"));
		return false;
	}

	delay(5); // Короткая задержка

	uint16_t len = rxBytesReceived();
	if (len != 32)
	{
		Serial.print(F("READ_SIG: ожидалось 32 байта, получено "));
		Serial.println(len);
		return false;
	}

	// Читаем данные в буфер
	if (!readData(32, sigBuffer))
	{
		Serial.println(F("Ошибка чтения ECC подписи"));
		return false;
	}

	// Выводим подпись (по 16 байт на строку, как принято)
	Serial.println(F("ECC-подпись (READ_SIG):"));
	for (int i = 0; i < 32; i++)
	{
		if (sigBuffer[i] < 0x10)
			Serial.print("0");
		Serial.print(sigBuffer[i], HEX);
		Serial.print(" ");
		if ((i + 1) % 16 == 0)
			Serial.println();
	}

	return true;
}

bool PN5180ISO14443::mifarePwdAuth(uint8_t *pwd, uint8_t *pack)
{
	uint8_t cmd[5];
	uint8_t response[2]; // PACK должен быть 2 байта
	uint16_t len;

	// Формируем команду: 0x1B + 4 байта пароля
	cmd[0] = 0x1B;
	memcpy(&cmd[1], pwd, 4);

	Serial.print(F("Отправка PWD_AUTH: "));
	for (int i = 0; i < 5; i++)
	{
		Serial.print(cmd[i], HEX);
		Serial.print(" ");
	}
	Serial.println();

	// Отправляем команду на карту
	if (!sendData(cmd, 5, 0x00))
	{
		Serial.println(F("Ошибка отправки PWD_AUTH"));
		return false;
	}

	delay(5); // маленькая задержка, чтобы карта успела ответить

	len = rxBytesReceived();
	if (len != 2)
	{
		Serial.print(F("Ошибка: ожидалось 2 байта PACK, получено: "));
		Serial.println(len);
		return false;
	}

	// Читаем PACK
	if (!readData(2, response))
	{
		Serial.println(F("Ошибка чтения PACK после PWD_AUTH"));
		return false;
	}

	// Сохраняем ответ в pack
	pack[0] = response[0];
	pack[1] = response[1];

	Serial.print(F("PACK: "));
	Serial.print(pack[0], HEX);
	Serial.print(" ");
	Serial.println(pack[1], HEX);

	return true;
}
