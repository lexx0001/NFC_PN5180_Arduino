// ИМЯ: PN5180.cpp
//
// ОПИСАНИЕ: Реализация класса PN5180.
//
// Copyright (c) 2018 by Andreas Trappmann. All rights reserved.
//
// Этот файл является частью библиотеки PN5180 для среды Arduino.
//
// Эта библиотека является свободным программным обеспечением; вы можете распространять и/или
// изменять её на условиях Стандартной общественной лицензии GNU Lesser General Public
// License, опубликованной Free Software Foundation; либо версии 2.1 лицензии, либо (по вашему выбору) любой более поздней версии.
//
// Эта библиотека распространяется в надежде, что она будет полезной,
// но БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ; даже без подразумеваемой гарантии
// КОММЕРЧЕСКОЙ ПРИГОДНОСТИ или ПРИГОДНОСТИ ДЛЯ ОПРЕДЕЛЕННОЙ ЦЕЛИ. Подробнее см. в
// Стандартной общественной лицензии GNU Lesser General Public License.
//
//#define DEBUG 1

#include <Arduino.h>
#include "PN5180.h"
#include "Debug.h"



// 1-байтовые прямые команды PN5180
// см. 11.4.3.3 Host Interface Command List
#define PN5180_WRITE_REGISTER           (0x00)
#define PN5180_WRITE_REGISTER_OR_MASK   (0x01)
#define PN5180_WRITE_REGISTER_AND_MASK  (0x02)
#define PN5180_READ_REGISTER            (0x04)
#define PN5180_WRITE_EEPROM             (0x06)
#define PN5180_READ_EEPROM              (0x07)
#define PN5180_SEND_DATA                (0x09)
#define PN5180_READ_DATA                (0x0A)
#define PN5180_SWITCH_MODE              (0x0B)
#define PN5180_LOAD_RF_CONFIG           (0x11)
#define PN5180_RF_ON                    (0x16)
#define PN5180_RF_OFF                   (0x17)

#define DEBUG

uint8_t PN5180::readBuffer[508];

PN5180::PN5180(uint8_t SSpin, uint8_t BUSYpin, uint8_t RSTpin) {
  PN5180_NSS = SSpin;
  PN5180_BUSY = BUSYpin;
  PN5180_RST = RSTpin;

  /*
   * 11.4.1 Физический интерфейс хоста
   * Интерфейс PN5180 с микроконтроллером-хостом основан на интерфейсе SPI,
   * расширенном сигнальной линией BUSY. Максимальная скорость SPI — 7 Мбит/с и фиксирована на CPOL
   * = 0 и CPHA = 0.
   */
  // Настройки для PN5180: 7Мбит/с, старший бит первым, SPI_MODE0 (CPOL=0, CPHA=0)
  PN5180_SPI_SETTINGS = SPISettings(1000000, MSBFIRST, SPI_MODE0);
}

void PN5180::begin() {
  pinMode(PN5180_NSS, OUTPUT);
  pinMode(PN5180_BUSY, INPUT);
  pinMode(PN5180_RST, OUTPUT);

  digitalWrite(PN5180_NSS, HIGH); // отключить
  digitalWrite(PN5180_RST, HIGH); // нет сброса

  SPI.begin();
  PN5180DEBUG(F("SPI pinout: "));
  PN5180DEBUG(F("SS=")); PN5180DEBUG(SS);
  PN5180DEBUG(F(", MOSI=")); PN5180DEBUG(MOSI);
  PN5180DEBUG(F(", MISO=")); PN5180DEBUG(MISO);
  PN5180DEBUG(F(", SCK=")); PN5180DEBUG(SCK);
  PN5180DEBUG("\n");
}

void PN5180::end() {
  digitalWrite(PN5180_NSS, HIGH); // отключить
  SPI.end();
}

/*
 * WRITE_REGISTER - 0x00
 * Эта команда используется для записи 32-битного значения (младший байт первым) в конфигурационный регистр.
 * Адрес регистра должен существовать. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::writeRegister(uint8_t reg, uint32_t value) {
  uint8_t *p = (uint8_t*)&value;

#ifdef DEBUG
  PN5180DEBUG(F("Write Register 0x"));
  PN5180DEBUG(formatHex(reg));
  PN5180DEBUG(F(", value (LSB first)=0x"));
  for (int i=0; i<4; i++) {
    PN5180DEBUG(formatHex(p[i]));
  }
  PN5180DEBUG("\n");
#endif

  /*
  Для всех 4-байтовых передач параметров команд (например, значений регистров), параметры
  передаются в формате младший байт первым (Little Endian).
   */
  uint8_t buf[6] = { PN5180_WRITE_REGISTER, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * WRITE_REGISTER_OR_MASK - 0x01
 * Эта команда изменяет содержимое регистра с помощью логической операции ИЛИ. 
 * Содержимое регистра читается и выполняется логическая операция ИЛИ с предоставленной маской.
 * Изменённое содержимое записывается обратно в регистр.
 * Адрес регистра должен существовать. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::writeRegisterWithOrMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;

#ifdef DEBUG
  PN5180DEBUG(F("Write Register 0x"));
  PN5180DEBUG(formatHex(reg));
  PN5180DEBUG(F(" with OR mask (LSB first)=0x"));
  for (int i=0; i<4; i++) {
    PN5180DEBUG(formatHex(p[i]));
  }
  PN5180DEBUG("\n");
#endif

  uint8_t buf[6] = { PN5180_WRITE_REGISTER_OR_MASK, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * WRITE _REGISTER_AND_MASK - 0x02
 * Эта команда изменяет содержимое регистра с помощью логической операции И. 
 * Содержимое регистра читается и выполняется логическая операция И с предоставленной маской.
 * Изменённое содержимое записывается обратно в регистр.
 * Адрес регистра должен существовать. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::writeRegisterWithAndMask(uint8_t reg, uint32_t mask) {
  uint8_t *p = (uint8_t*)&mask;

#ifdef DEBUG
  PN5180DEBUG(F("Write Register 0x"));
  PN5180DEBUG(formatHex(reg));
  PN5180DEBUG(F(" with AND mask (LSB first)=0x"));
  for (int i=0; i<4; i++) {
    PN5180DEBUG(formatHex(p[i]));
  }
  PN5180DEBUG("\n");
#endif

  uint8_t buf[6] = { PN5180_WRITE_REGISTER_AND_MASK, reg, p[0], p[1], p[2], p[3] };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(buf, 6);
  SPI.endTransaction();

  return true;
}

/*
 * READ_REGISTER - 0x04
 * Эта команда используется для чтения содержимого конфигурационного регистра. Содержимое регистра
 * возвращается в 4-байтовом ответе.
 * Адрес регистра должен существовать. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::readRegister(uint8_t reg, uint32_t *value) {
  PN5180DEBUG(F("Reading register 0x"));
  PN5180DEBUG(formatHex(reg));
  PN5180DEBUG(F("...\n"));

  uint8_t cmd[2] = { PN5180_READ_REGISTER, reg };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2, (uint8_t*)value, 4);
  SPI.endTransaction();

  PN5180DEBUG(F("Register value=0x"));
  PN5180DEBUG(formatHex(*value));
  PN5180DEBUG("\n");

  return true;
}

/*
 * WRITE_EEPROM - 0x06
 */
bool PN5180::writeEEprom(uint8_t addr, uint8_t *buffer, uint8_t len) {
  uint8_t cmd[len + 2];
  cmd[0] = PN5180_WRITE_EEPROM;
  cmd[1] = addr;
  for (int i = 0; i < len; i++) cmd[2 + i] = buffer[i];
  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, len + 2);
  SPI.endTransaction();
  return true;
}

/*
 * READ_EEPROM - 0x07
 * Эта команда используется для чтения данных из области памяти EEPROM. Поле 'Address'
 * указывает начальный адрес операции чтения. Поле Length указывает количество
 * байтов для чтения. Ответ содержит данные, считанные из EEPROM (содержимое
 * EEPROM); Данные читаются последовательно, начиная с указанного адреса.
 * Адрес EEPROM должен быть в диапазоне от 0 до 254 включительно. Операция чтения не должна
 * выходить за пределы адреса EEPROM 254. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::readEEprom(uint8_t addr, uint8_t *buffer, int len) {
  if ((addr > 254) || ((addr+len) > 254)) {
    PN5180DEBUG(F("ERROR: Reading beyond addr 254!\n"));
    return false;
  }

  PN5180DEBUG(F("Reading EEPROM at 0x"));
  PN5180DEBUG(formatHex(addr));
  PN5180DEBUG(F(", size="));
  PN5180DEBUG(len);
  PN5180DEBUG(F("...\n"));

  uint8_t cmd[3] = { PN5180_READ_EEPROM, addr, len };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 3, buffer, len);
  SPI.endTransaction();

#ifdef DEBUG
  PN5180DEBUG(F("EEPROM values: "));
  for (int i=0; i<len; i++) {
    PN5180DEBUG(formatHex(buffer[i]));
    PN5180DEBUG(" ");
  }
  PN5180DEBUG("\n");
#endif

  return true;
}




/*
 * SEND_DATA - 0x09
 * Эта команда записывает данные в буфер передачи RF и запускает передачу RF.
 * Параметр ‘Number of valid bits in last Byte’ указывает точное количество бит для передачи в последнем байте
 * (для невыравненных по байтам кадров).
 * Предусловие: Хост должен сконфигурировать трансивер, установив регистр
 * SYSTEM_CONFIG.COMMAND в 0x3 перед использованием команды SEND_DATA, так как
 * команда SEND_DATA только записывает данные в буфер передачи и запускает
 * передачу, но не выполняет никакой конфигурации.
 * Размер поля ‘Tx Data’ должен быть в диапазоне от 0 до 260 включительно (длина 0 байт
 * позволяет передавать только символ, если TX_DATA_ENABLE сброшен). Поле ‘Number of
 * valid bits in last Byte’ должно быть в диапазоне от 0 до 7. Команда не должна вызываться во время
 * текущей передачи RF. Трансивер должен быть в состоянии ‘WaitTransmit’
 * с установленной командой ‘Transceive’. Если это условие не выполнено, возникает исключение.
 */
bool PN5180::sendData(uint8_t *data, int len, uint8_t validBits) {
  if (len > 260) {
    PN5180DEBUG(F("ERROR: sendData with more than 260 bytes is not supported!\n"));
    return false;
  }

#ifdef DEBUG
  PN5180DEBUG(F("Send data (len="));
  PN5180DEBUG(len);
  PN5180DEBUG(F("):"));
  for (int i=0; i<len; i++) {
    PN5180DEBUG(" ");
    PN5180DEBUG(formatHex(data[i]));
  }
  PN5180DEBUG("\n");
#endif

  uint8_t buffer[len+2];
  buffer[0] = PN5180_SEND_DATA;
  buffer[1] = validBits; // количество валидных бит в последнем байте для передачи (0 = все биты передаются)
  for (int i=0; i<len; i++) {
    buffer[2+i] = data[i];
  }

  writeRegisterWithAndMask(SYSTEM_CONFIG, 0xfffffff8);  // Команда Idle/StopCom
  writeRegisterWithOrMask(SYSTEM_CONFIG, 0x00000003);   // Команда Transceive
  /*
   * Команда Transceive; инициирует цикл передачи/приёма.
   * Примечание: В зависимости от значения бита Initiator,
   * начинается передача или включается приёмник
   * Примечание: Команда transceive не завершается
   * автоматически. Она остаётся в цикле передачи/приёма до
   * остановки через команду IDLE/StopCom
   */

  PN5180TransceiveStat transceiveState = getTransceiveState();
  if (PN5180_TS_WaitTransmit != transceiveState) {
    PN5180DEBUG(F("*** ERROR: Transceiver not in state WaitTransmit!?\n"));
    return false;
  }

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  bool success = transceiveCommand(buffer, len+2);
  SPI.endTransaction();

  return success;
}

/*
 * READ_DATA - 0x0A
 * Эта команда читает данные из буфера приёма RF после успешного приёма.
 * Регистр RX_STATUS содержит информацию для проверки успешности приёма. Данные доступны
 * в ответе на команду. Хост управляет количеством байтов для чтения через интерфейс SPI.
 * RF-данные были успешно приняты. В случае выполнения инструкции без
 * предшествующего приёма RF-данных, исключение не возникает, но данные, считанные из буфера приёма,
 * будут недействительными. Если это условие не выполнено, возникает исключение.
 */
uint8_t * PN5180::readData(int len) {
  if (len > 508) {
    Serial.println(F("*** FATAL: Reading more than 508 bytes is not supported!"));
    return 0L;
  }

  PN5180DEBUG(F("Reading Data (len="));
  PN5180DEBUG(len);
  PN5180DEBUG(F(")...\n"));

  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2, readBuffer, len);
  SPI.endTransaction();

#ifdef DEBUG
  PN5180DEBUG(F("Data read: "));
  for (int i=0; i<len; i++) {
    PN5180DEBUG(formatHex(readBuffer[i]));
    PN5180DEBUG(" ");
  }
  PN5180DEBUG("\n");
#endif

  return readBuffer;
}

bool PN5180::readData(uint8_t len, uint8_t *buffer) {
  if (len > 508) {
    return false;
  }
  uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };
  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  bool success = transceiveCommand(cmd, 2, buffer, len);
  SPI.endTransaction();
  return success;
}




/* переключить режим на LPCD (обнаружение карты при низком энергопотреблении)
 * Параметр 'wakeupCounterInMs' должен быть в диапазоне от 0x0 до 0xA82
 * максимальное время пробуждения — 2960 мс.
 */
bool PN5180::switchToLPCD(uint16_t wakeupCounterInMs) {
  // очистить все флаги IRQ
  clearIRQStatus(0xffffffff); 
  // включить только LPCD и общий IRQ ошибок
  writeRegister(IRQ_ENABLE, LPCD_IRQ_STAT | GENERAL_ERROR_IRQ_STAT);  
  // переключить режим на LPCD 
  uint8_t cmd[4] = { PN5180_SWITCH_MODE, 0x01, (uint8_t)(wakeupCounterInMs & 0xFF), (uint8_t)((wakeupCounterInMs >> 8U) & 0xFF) };
  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  bool success = transceiveCommand(cmd, sizeof(cmd));
  SPI.endTransaction();
  return success;
}

/*
 * LOAD_RF_CONFIG - 0x11
 * Параметр 'Transmitter Configuration' должен быть в диапазоне от 0x0 до 0x1C включительно. Если
 * параметр передатчика равен 0xFF, конфигурация передатчика не изменяется.
 * Поле 'Receiver Configuration' должно быть в диапазоне от 0x80 до 0x9C включительно. Если
 * параметр приёмника равен 0xFF, конфигурация приёмника не изменяется. Если это условие не выполнено, возникает исключение.
 * Конфигурация передатчика и приёмника всегда должны быть настроены на одну и ту же
 * скорость передачи/приёма. Ошибка не возвращается, если это условие не выполнено.
 *
 * Передатчик: RF   Протокол          Скорость     Приёмник: RF    Протокол    Скорость
 * конфигурация                        (кбит/с)    конфигурация                (кбит/с)
 * байт (hex)                                      байт (hex)
 * ----------------------------------------------------------------------------------------------
 * ->0D              ISO 15693 ASK100  26        8D              ISO 15693   26
 *   0E              ISO 15693 ASK10   26        8E              ISO 15693   53
 */
bool PN5180::loadRFConfig(uint8_t txConf, uint8_t rxConf) {
  PN5180DEBUG(F("Load RF-Config: txConf="));
  PN5180DEBUG(formatHex(txConf));
  PN5180DEBUG(F(", rxConf="));
  PN5180DEBUG(formatHex(rxConf));
  PN5180DEBUG("\n");

  uint8_t cmd[3] = { PN5180_LOAD_RF_CONFIG, txConf, rxConf };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 3);
  SPI.endTransaction();

  return true;
}

/*
 * RF_ON - 0x16
 * Эта команда используется для включения внутреннего RF-поля. Если включено, TX_RFON_IRQ
 * устанавливается после включения поля.
 */
bool PN5180::setRF_on() {
  PN5180DEBUG(F("Set RF ON\n"));

  uint8_t cmd[2] = { PN5180_RF_ON, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  SPI.endTransaction();

  while (0 == (TX_RFON_IRQ_STAT & getIRQStatus())); // ждать, пока RF-поле не будет установлено
  clearIRQStatus(TX_RFON_IRQ_STAT);
  return true;
}

/*
 * RF_OFF - 0x17
 * Эта команда используется для выключения внутреннего RF-поля. Если включено, TX_RFOFF_IRQ
 * устанавливается после выключения поля.
 */
bool PN5180::setRF_off() {
  PN5180DEBUG(F("Set RF OFF\n"));

  uint8_t cmd[2] { PN5180_RF_OFF, 0x00 };

  SPI.beginTransaction(PN5180_SPI_SETTINGS);
  transceiveCommand(cmd, 2);
  SPI.endTransaction();

  while (0 == (TX_RFOFF_IRQ_STAT & getIRQStatus())); // ждать, пока RF-поле не выключится
  clearIRQStatus(TX_RFOFF_IRQ_STAT);
  return true;
}

//---------------------------------------------------------------------------------------------

/*
11.4.3.1 Команда интерфейса хоста состоит из 1 или 2 SPI-фреймов в зависимости от того,
хочет ли хост записать или прочитать данные из PN5180. SPI-фрейм состоит из нескольких
байтов.

Все команды упакованы в один SPI-фрейм. SPI-фрейм состоит из нескольких байтов.
Во время отправки SPI-фрейма переключение NSS не допускается.

Для всех 4-байтовых передач параметров команд (например, значений регистров), параметры
передаются в формате младший байт первым (Little Endian).

Прямые инструкции состоят из кода команды (1 байт) и параметров инструкции
(максимум 260 байт). Фактический размер полезной нагрузки зависит от используемой инструкции.
Ответы на прямые инструкции содержат только поле полезной нагрузки (без заголовка).
Все инструкции подчиняются условиям. Если хотя бы одно из условий не выполнено, возникает исключение.
В случае исключения линия IRQ PN5180 активируется, а соответствующий регистр состояния прерывания
содержит информацию об исключении.
*/

/*
 * Команда интерфейса хоста состоит из 1 или 2 SPI-фреймов в зависимости от того,
 * хочет ли хост записать или прочитать данные из PN5180. SPI-фрейм состоит из нескольких
 * байтов.
 * Все команды упакованы в один SPI-фрейм. SPI-фрейм состоит из нескольких байтов.
 * Во время отправки SPI-фрейма переключение NSS не допускается.
 * Для всех 4-байтовых передач параметров команд (например, значений регистров), параметры
 * передаются в формате младший байт первым (Little Endian).
 * Линия BUSY используется для индикации того, что система занята и не может принимать данные
 * от хоста. Рекомендации по обработке линии BUSY хостом:
 * 1. Установить NSS в LOW
 * 2. Выполнить обмен данными
 * 3. Ждать, пока BUSY станет HIGH
 * 4. Установить NSS в HIGH
 * 5. Ждать, пока BUSY станет LOW
 * Если есть ошибка параметра, IRQ устанавливается в ACTIVE и устанавливается GENERAL_ERROR_IRQ.
 */
bool PN5180::transceiveCommand(uint8_t *sendBuffer, size_t sendBufferLen, uint8_t *recvBuffer, size_t recvBufferLen) {
#ifdef DEBUG
  PN5180DEBUG(F("Sending SPI frame: '"));
  for (uint8_t i=0; i<sendBufferLen; i++) {
    if (i>0) PN5180DEBUG(" ");
    PN5180DEBUG(formatHex(sendBuffer[i]));
  }
  PN5180DEBUG("'\n");
#endif

  // 0.
  unsigned long startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) return false;
  }; // ждать, пока busy не станет low
  // 1.
  digitalWrite(PN5180_NSS, LOW); delay(2);
  // 2.
  for (uint8_t i=0; i<sendBufferLen; i++) {
    SPI.transfer(sendBuffer[i]);
  }
  // 3.
  startedWaiting = millis();
  while (HIGH != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) return false;
  }; // ждать, пока busy не станет high
  // 4.
  digitalWrite(PN5180_NSS, HIGH); delay(1);
  // 5.
  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) return false;
  }; // ждать, пока busy не станет low

  // проверить, только ли запись
  //
  if ((0 == recvBuffer) || (0 == recvBufferLen)) return true;
  PN5180DEBUG(F("Receiving SPI frame...\n"));

  // 1.
  digitalWrite(PN5180_NSS, LOW); delay(2);
  // 2.
  for (uint8_t i=0; i<recvBufferLen; i++) {
    recvBuffer[i] = SPI.transfer(0xff);
  }
  // 3.
  startedWaiting = millis();
  while (HIGH != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) return false;
  }; // ждать, пока busy не станет high
  // 4.
  digitalWrite(PN5180_NSS, HIGH); delay(1);
  // 5.
  startedWaiting = millis();
  while (LOW != digitalRead(PN5180_BUSY)) {
    if (millis() - startedWaiting > commandTimeout) return false;
  }; // ждать, пока busy не станет low

#ifdef DEBUG
  PN5180DEBUG(F("Received: "));
  for (uint8_t i=0; i<recvBufferLen; i++) {
    if (i > 0) PN5180DEBUG(" ");
    PN5180DEBUG(formatHex(recvBuffer[i]));
  }
  PN5180DEBUG("'\n");
#endif

  return true;
}


/* Функция перемещена в PN5180ISO14443.cpp */

/*
 * Сбросить NFC-устройство
 */
void PN5180::reset() {
  digitalWrite(PN5180_RST, LOW);  // требуется не менее 10 мкс
  delay(200);
  digitalWrite(PN5180_RST, HIGH); // требуется 2 мс для запуска
  delay(200);

  while (0 == (IDLE_IRQ_STAT & getIRQStatus())); // ждать запуска системы

  clearIRQStatus(0xffffffff); // очистить все флаги
}

/**
 * @name  getInterrupt
 * @desc  прочитать регистр состояния прерывания и очистить его
 */
uint32_t PN5180::getIRQStatus() {

  PN5180DEBUG(F("Read IRQ-Status register...\n"));

  uint32_t irqStatus;
  readRegister(IRQ_STATUS, &irqStatus);

  PN5180DEBUG(F("IRQ-Status=0x"));
  PN5180DEBUG(formatHex(irqStatus));
  PN5180DEBUG("\n");

  return irqStatus;
}

bool PN5180::clearIRQStatus(uint32_t irqMask) {
  PN5180DEBUG(F("Clear IRQ-Status with mask=x"));
  PN5180DEBUG(formatHex(irqMask));
  PN5180DEBUG("\n");

  return writeRegister(IRQ_CLEAR, irqMask);
}

/*
 * Получить TRANSCEIVE_STATE из регистра RF_STATUS
 */
#ifdef DEBUG
extern void showIRQStatus(uint32_t);
#endif

PN5180TransceiveStat PN5180::getTransceiveState() {
  PN5180DEBUG(F("Get Transceive state...\n"));

  uint32_t rfStatus;
  if (!readRegister(RF_STATUS, &rfStatus)) {
#ifdef DEBUG
    showIRQStatus(getIRQStatus());
#endif
    PN5180DEBUG(F("ERROR reading RF_STATUS register.\n"));
    return PN5180TransceiveStat(0);
  }

  /*
   * TRANSCEIVE_STATE:
   *  0 - idle (ожидание)
   *  1 - wait transmit (ожидание передачи)
   *  2 - transmitting (передача)
   *  3 - wait receive (ожидание приёма)
   *  4 - wait for data (ожидание данных)
   *  5 - receiving (приём)
   *  6 - loopback (замкнутый цикл)
   *  7 - зарезервировано
   */
  uint8_t state = ((rfStatus >> 24) & 0x07);
  PN5180DEBUG(F("TRANSCEIVE_STATE=0x"));
  PN5180DEBUG(formatHex(state));
  PN5180DEBUG("\n");

  return PN5180TransceiveStat(state);
}

// Функция для отображения состояния IRQ в человекочитаемом виде
void PN5180::showIRQStatus(uint32_t irqStatus)
{
  Serial.print(F("IRQ-Status 0x"));
  Serial.print(irqStatus, HEX);
  Serial.print(": [ ");
  if (irqStatus & (1UL << 0))
    Serial.print(F("RQ ")); // RQ - Request - запрос на выполнение команды
  if (irqStatus & (1UL << 1))
    Serial.print(F("TX ")); // TX - передача данных
  if (irqStatus & (1UL << 2))
    Serial.print(F("IDLE ")); // Ожидание (Idle) - режим ожидания, когда нет активных команд
  if (irqStatus & (1UL << 3)) 
    Serial.print(F("MODE_DETECTED ")); // MODE_DETECTED - обнаружение режима работы (например, режим чтения карты)
  if (irqStatus & (1UL << 4)) 
    Serial.print(F("CARD_ACTIVATED ")); // CARD_ACTIVATED - карта активирована
  if (irqStatus & (1UL << 5))
    Serial.print(F("STATE_CHANGE ")); // STATE_CHANGE - изменение состояния
  if (irqStatus & (1UL << 6))
    Serial.print(F("RFOFF_DET ")); // RFOFF_DET - обнаружение выключения радиочастотного поля
  if (irqStatus & (1UL << 7))
    Serial.print(F("RFON_DET ")); // RFON_DET - обнаружение включения радиочастотного поля
  if (irqStatus & (1UL << 8))
    Serial.print(F("TX_RFOFF ")); // TX_RFOFF - радиочастотное поле выключено
  if (irqStatus & (1UL << 9))
    Serial.print(F("TX_RFON ")); // TX_RFON - радиочастотное поле включено
  if (irqStatus & (1UL << 10))
    Serial.print(F("RF_ACTIVE_ERROR ")); // RF Active Error - ошибка активного режима радиочастотной цепи
  if (irqStatus & (1UL << 11))
    Serial.print(F("TIMER0 ")); 
  if (irqStatus & (1UL << 12))
    Serial.print(F("TIMER1 ")); 
  if (irqStatus & (1UL << 13))
    Serial.print(F("TIMER2 ")); 
  if (irqStatus & (1UL << 14)) 
    Serial.print(F("RX_SOF_DET ")); // RX_SOF_DET Start of Frame Detection - обнаружение начала кадра 
  if (irqStatus & (1UL << 15))
    Serial.print(F("RX_SC_DET ")); // RX Short Circuit Detection - обнаружение короткого замыкания в цепи приёмника
  if (irqStatus & (1UL << 16))
    Serial.print(F("TEMPSENS_ERROR ")); // Temperature Sensor Error - ошибка датчика температуры
  if (irqStatus & (1UL << 17))
    Serial.print(F("GENERAL_ERROR ")); 
  if (irqStatus & (1UL << 18)) 
    Serial.print(F("HV_ERROR ")); // High Voltage Error - ошибка высокого напряжения в цепи питания PN5180
  if (irqStatus & (1UL << 19))
    Serial.print(F("LPCD ")); // Low Power Card Detection - обнаружение карты в режиме низкого энергопотребления
  Serial.println("]");
}

uint8_t PN5180::readRFResponse(uint8_t* buffer, uint8_t maxLen) {
    uint32_t rxLen = 0;
    if (!readRegister(SYSTEM_STATUS, &rxLen)) return 0;

    uint8_t len = rxLen & 0xFF;
    if (len == 0 || len > maxLen) return 0;

    uint8_t cmd[2] = { PN5180_READ_DATA, 0x00 };

    SPI.beginTransaction(PN5180_SPI_SETTINGS);
    bool ok = transceiveCommand(cmd, 2, buffer, len);
    SPI.endTransaction();

    return ok ? len : 0;
}
