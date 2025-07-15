// NAME: PN5180ISO14443.h
//
// DESC: Протокол ISO14443 на модуле NXP Semiconductors PN5180 для Arduino.
//
// Copyright (c) 2019 by Dirk Carstensen. Все права защищены.
//
// Этот файл является частью библиотеки PN5180 для среды Arduino.
//
// Эта библиотека является свободным программным обеспечением; вы можете распространять и/или
// изменять её на условиях Стандартной Общественной Лицензии GNU (LGPL) версии 2.1
// либо (по вашему выбору) любой более поздней версии.
//
// Эта библиотека распространяется в надежде, что она будет полезной,
// но БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ; даже без подразумеваемой гарантии
// КОММЕРЧЕСКОЙ ЦЕННОСТИ или ПРИГОДНОСТИ ДЛЯ ОПРЕДЕЛЕННОЙ ЦЕЛИ. Подробнее см. в
// Стандартной Общественной Лицензии GNU.
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
	PN5180DEBUG(F("Загрузка RF-конфигурации...\n"));
	if (loadRFConfig(0x00, 0x80))
	{ // параметры ISO14443
		PN5180DEBUG(F("готово.\n"));
	}
	else
		return false;

	PN5180DEBUG(F("Включение RF поля...\n"));
	if (setRF_on())
	{
		PN5180DEBUG(F("готово.\n"));
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
	// Младшие 9 бит содержат длину
	len = (uint16_t)(rxStatus & 0x000001ff);
	return len;
}
/*
 * buffer : должен быть массивом из 10 байт
 * buffer[0-1] — ATQA
 * buffer[2] — SAK
 * buffer[3..6] — 4 байта UID
 * buffer[7..9] — оставшиеся 3 байта UID для меток с 7-байтовым UID
 * kind : 0 — отправляем REQA, 1 — отправляем WUPA
 *
 * возвращаемое значение: длина uid:
 * -	ноль, если метка не распознана
 * -	одинарный UID (4 байта)
 * -	двойной UID (7 байт)
 * -	тройной UID (10 байт) — не поддерживается
 */
uint8_t PN5180ISO14443::activateTypeA(uint8_t *buffer, uint8_t kind)
{
	uint8_t cmd[7];
	uint8_t uidLength = 0;
	// Загружаем стандартный протокол TypeA
	if (!loadRFConfig(0x0, 0x80))
		return 0;

	// Отключаем Crypto
	if (!writeRegisterWithAndMask(SYSTEM_CONFIG, 0xFFFFFFBF))
		return 0;
	// Сбрасываем RX CRC
	if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))
		return 0;
	// Сбрасываем TX CRC
	if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))
		return 0;
	// Отправляем REQA/WUPA, 7 бит в последнем байте
	cmd[0] = (kind == 0) ? 0x26 : 0x52;
	if (!sendData(cmd, 1, 0x07))
		return 0;
	// Читаем 2 байта ATQA в buffer
	if (!readData(2, buffer))
		return 0;
	// Отправляем Anti collision 1, 8 бит в последнем байте
	cmd[0] = 0x93;
	cmd[1] = 0x20;
	if (!sendData(cmd, 2, 0x00))
		return 0;
	// Читаем 5 байт, сохраняем с offset 2 для дальнейшего использования
	if (!readData(5, cmd + 2))
		return 0;
	// Включаем вычисление RX CRC
	if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01))
		return 0;
	// Включаем вычисление TX CRC
	if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01))
		return 0;
	// Отправляем Select anti collision 1, остальные байты уже в offset 2 и далее
	cmd[0] = 0x93;
	cmd[1] = 0x70;
	if (!sendData(cmd, 7, 0x00))
		return 0;
	// Читаем 1 байт SAK в buffer[2]
	if (!readData(1, buffer + 2))
		return 0;
	// Проверяем, 4-байтовый UID или 7-байтовый UID и требуется ли anti collision 2
	// Если бит 3 равен 0 — это 4-байтовый UID
	if ((buffer[2] & 0x04) == 0)
	{
		// Берём первые 4 байта anti collision как UID, сохраняем с offset 3. Готово
		for (int i = 0; i < 4; i++)
			buffer[3 + i] = cmd[2 + i];
		uidLength = 4;
	}
	else
	{
		// Берём первые 3 байта UID, игнорируем первый байт 88(CT)
		if (cmd[2] != 0x88)
			return 0;
		for (int i = 0; i < 3; i++)
			buffer[3 + i] = cmd[3 + i];
		// Сбрасываем RX CRC
		if (!writeRegisterWithAndMask(CRC_RX_CONFIG, 0xFFFFFFFE))
			return 0;
		// Сбрасываем TX CRC
		if (!writeRegisterWithAndMask(CRC_TX_CONFIG, 0xFFFFFFFE))
			return 0;
		// Выполняем anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x20;
		if (!sendData(cmd, 2, 0x00))
			return 0;
		// Читаем 5 байт, сохраняем с offset 2 для дальнейшего использования
		if (!readData(5, cmd + 2))
			return 0;
		// первые 4 байта — это последние 4 байта UID, сохраняем их
		for (int i = 0; i < 4; i++)
		{
			buffer[6 + i] = cmd[2 + i];
		}
		// Включаем вычисление RX CRC
		if (!writeRegisterWithOrMask(CRC_RX_CONFIG, 0x01))
			return 0;
		// Включаем вычисление TX CRC
		if (!writeRegisterWithOrMask(CRC_TX_CONFIG, 0x01))
			return 0;
		// Отправляем Select anti collision 2
		cmd[0] = 0x95;
		cmd[1] = 0x70;
		if (!sendData(cmd, 7, 0x00))
			return 0;
		// Читаем 1 байт SAK в buffer[2]
		if (!readData(1, buffer + 2))
			return 0;
		uidLength = 7;
	}

	return uidLength;
}

bool PN5180ISO14443::mifareBlockRead(uint8_t blockno, uint8_t *buffer)
{
	bool success = false;
	uint16_t len;
	uint8_t cmd[2];
	// Отправляем команду mifare 30, blockno
	cmd[0] = 0x30;
	cmd[1] = blockno;
	if (!sendData(cmd, 2, 0x00))
	{
		Serial.print(F("Ошибка чтения блока "));
		Serial.println(blockno, HEX);
		return false;
	}
	// Проверяем, получили ли мы какие-либо данные от метки
	delay(5);
	len = rxBytesReceived();
	if (len == 16)
	{
		// Читаем 16 байт в buffer
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

uint8_t PN5180ISO14443::mifareUltralightWrite(uint8_t block, uint8_t *data4)
{
	uint8_t cmd[6];
	cmd[0] = 0xA2;	// WRITE-команда для Ultralight
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
	// Всегда возвращаем 10 байт
	// Смещение 0..1 — ATQA
	// Смещение 2 — SAK.
	// UID 4 байта: смещение 3–6 — UID, смещение 7–9 — нули
	// UID 7 байт: смещение 3–9 — UID
	for (int i = 0; i < 10; i++)
		response[i] = 0;
	uidLength = activateTypeA(response, 1);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
		return 0;
	// проверяем валидность uid
	if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00))
		return 0;
	if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF))
		return 0;
	for (int i = 0; i < 10; i++)
		buffer[i] = response[i];

	mifareHalt();
	return uidLength;
}

uint8_t PN5180ISO14443::readCard_UL_EV1(uint8_t *buffer)
{
	uint8_t response[10];
	uint8_t uidLength;
	// Всегда возвращаем 10 байт
	// Смещение 0..1 — ATQA
	// Смещение 2 — SAK.
	// UID 4 байта: смещение 3–6 — UID, смещение 7–9 — нули
	// UID 7 байт: смещение 3–9 — UID
	for (int i = 0; i < 10; i++)
		response[i] = 0;
	uidLength = activateTypeA(response, 1);
	if ((response[0] == 0xFF) && (response[1] == 0xFF))
		return 0;
	// проверяем валидность uid
	if ((response[3] == 0x00) && (response[4] == 0x00) && (response[5] == 0x00) && (response[6] == 0x00))
		return 0;
	if ((response[3] == 0xFF) && (response[4] == 0xFF) && (response[5] == 0xFF) && (response[6] == 0xFF))
		return 0;
	for (int i = 0; i < 10; i++)
	{
		buffer[i] = response[i];
	}

	// Проверка на SAK == 0x20, если так — вызываем sendRATS()
	if (response[2] == 0x20)
	{
		Serial.println(F("SAK == 0x20, отправка RATS..."));
		sendRATS();
	}

	// Проверяем: UID длина 7 байт, SAK = 0x00, ATQA = 0x0044
	if (!(uidLength == 7 && response[2] == 0x00 && response[0] == 0x44 && response[1] == 0x00))
	{
		Serial.println(F("Это не mifare_UL_EV1"));
		// mifareHalt();
		return uidLength;
	}

	// Читаем версию
	uint8_t versionData[8];
	if (mifare_UL_EV1_GetVersion(versionData))
	{
		// Проверяем, что это MIFARE Ultralight EV1 48 байт
		if (versionData[2] != 0x03 || versionData[4] != 0x01 || versionData[6] != 0x0B)
		{
			Serial.println(F("Это не mifare_UL_EV1 48 кБ"));
			return uidLength;
		}
	}

	Serial.println(F("Обнаружена mifare_UL_EV1 48 кБ!"));

	// Аутентификация PWD_AUTH
	// uint8_t password[4] = {0xD1, 0xF7, 0x34, 0x85}; //  твой пароль
	uint8_t password[4] = {0xFF, 0xFF, 0xFF, 0xFF}; //  пароль по умолчанию
	uint8_t pack_read[2];

	if (mifare_UL_EV1_PwdAuth(password, pack_read))
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
	if (mifare_UL_EV1_ReadSig(sig))
	{
		Serial.println(F("Подпись успешно считана!"));
	}

	// Читаем блок
	uint8_t blockData[16];
	mifareBlockRead(0x0F, blockData);

	mifareHalt();
	return uidLength;
}

bool PN5180ISO14443::isCardPresent()
{
	uint8_t buffer[10];
	return (readCardSerial(buffer) >= 4);
}

/*
 * Выполняет команду GET_VERSION (0x60) для карты MIFARE Ultralight EV1.
 *
 * Если команда успешна, устройство вернет 8 байт информации о чипе.
 * Формат ответа:
 *
 * Byte | Назначение                | Описание
 * -----|---------------------------|------------------------------------------
 * [0]  | Vendor ID                 | 0x04 = NXP Semiconductors
 * [1]  | Product Type              | 0x03 = MIFARE Ultralight
 * [2]  | Product Subtype           | 0x01 = EV1
 * [3]  | Major Product Version     | Например: 0x01
 * [4]  | Minor Product Version     | Например: 0x00
 * [5]  | Storage Size              | 0x0B = 192 байт (Ultralight EV1 192B)
 *                                  | 0x0E = 320 байт (Ultralight EV1 320B)
 * [6]  | Protocol Type             | 0x03 = ISO/IEC 14443-3 compliant
 * [7]  | RFU (Reserved for Future) | Может быть 0x00 или другое значение
 *
 * Пример возвращаемого буфера:
 *   0x04 0x03 0x01 0x01 0x00 0x0B 0x03 0x00
 *   └────┘ └────┘ └────┘ └────┘ └────┘ └────┘
 *    NXP   UL     EV1     v1.0    192B   ISO
 *
 * Возвращает: true — если команда успешно выполнена и данные получены,
 *             false — при ошибке отправки команды.
 */
bool PN5180ISO14443::mifare_UL_EV1_GetVersion(uint8_t *versionBuffer)

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

bool PN5180ISO14443::mifare_UL_EV1_ReadSig(uint8_t *sigBuffer)
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

bool PN5180ISO14443::mifare_UL_EV1_PwdAuth(uint8_t *pwd, uint8_t *pack)
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

void PN5180ISO14443::sendRATS()
{
	uint8_t rats[] = {0xE0, 0x80}; // RATS: FSDI=8, CID=0

	Serial.println(F("Отправляем RATS..."));
	if (!sendData(rats, sizeof(rats), 0))
	{
		Serial.println(F("Ошибка при отправке RATS"));
		return;
	}

	delay(3);
	uint8_t ats[32];
	uint8_t fwt_ats; // Таймаут ответа (Frame Waiting Time) из ATS
	int len = rxBytesReceived();
	if (len > 0 && static_cast<size_t>(len) <= sizeof(ats))
	{
		readData(len, ats);
		fwt_ats = ats[3]; // Получаем FWT из ATS, 4-й байт (индекс 3)
		Serial.print(F("ATS: "));
		for (int i = 0; i < len; i++)
		{
			Serial.print(ats[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
		delay(3);
		sendSelectAID(fwt_ats);
	}
	else
	{
		Serial.println(F("Не получили ATS или ошибка чтения"));
	}
}

// Отправляет команду SELECT AID для NFC Forum, учитывая FWI из ATS
void PN5180ISO14443::sendSelectAID(uint8_t fwt_ats)
{
	uint8_t selectNfcForum[] = {
		0x00, 0xA4, 0x04, 0x00,
		0x07, 0xD2, 0x76, 0x00,
		0x00, 0x85, 0x01, 0x01,
		0x00};

	// Формируем I-Block: PCB (0x02) + APDU
	uint8_t iblock[1 + sizeof(selectNfcForum)];
	iblock[0] = 0x02; // PCB для I-Block
	memcpy(&iblock[1], selectNfcForum, sizeof(selectNfcForum));

	// Вычисляем абсолютное значение FWT по формуле:
	// FWT = (256 * 16 / fc) * 2^FWI
	// fc = 13.56 МГц
	uint8_t FWI = fwt_ats & 0x0F;
	uint32_t base = (256UL * 16UL * 1000UL) / 13560UL; // ≈ 0.2234 мс
	uint32_t FWT_ms = base * (1UL << FWI);
	Serial.print(F("FWT (ms): "));
	Serial.println(FWT_ms);

	Serial.println(F("Отправляем SELECT AID (I-Block)"));
	if (!sendData(iblock, sizeof(iblock), 0))
	{
		Serial.println(F("Ошибка при отправке SELECT AID"));
		return;
	}

	uint8_t response[32];
	int len = 0;
	uint32_t start = millis();
	while ((len = rxBytesReceived()) == 0 && (millis() - start) < FWT_ms)
	{
		delay(20);
	}

	if (len > 0 && static_cast<size_t>(len) <= sizeof(response))
	{
		readData(len, response);
		if (response[0] != 0x02) // Проверяем PCB, должен быть I-Block
		{
			Serial.print(F("Ожидали I-Block (PCB 0x02), получили: "));
			Serial.println(response[0], HEX);
			return;
		}

		Serial.print(F("Ответ на SELECT AID: "));
		for (int i = 1; i < len; i++)
		{
			if (response[i] < 0x10)
				Serial.print("0");
			Serial.print(response[i], HEX);
			Serial.print(" ");
		}
		Serial.println();

		if (len >= 3 && response[len - 2] == 0x6A && response[len - 1] == 0x82)
		{
			Serial.println(F("разблокируйте телефон"));
		}
	}
	else
	{
		Serial.println(F("Не получили ответ на SELECT AID"));
	}
}
