// NAME: main.cpp
//
// DESC: Use the PN5180-NFC Module from NXP Semiconductors.
//
// BEWARE: SPI with an Arduino to a PN5180 module has to be at a level of 3.3V
// use of logic-level converters from 5V->3.3V is absolutly neccessary
// on most Arduinos for all input pins of PN5180!
// If used with an ESP-32, there is no need for a logic-level converter, since
// it operates on 3.3V already.
//
// Arduino <-> Level Converter <-> PN5180 pin mapping:
// 5V             <-->             5V
// 3.3V           <-->             3.3V
// GND            <-->             GND
// 5V      <-> HV
// GND     <-> GND (HV)
//             LV              <-> 3.3V
//             GND (LV)        <-> GND
// SCLK,13 <-> HV1 - LV1       --> SCLK
// MISO,12        <---         <-- MISO
// MOSI,11 <-> HV3 - LV3       --> MOSI
// SS,10   <-> HV4 - LV4       --> NSS (=Not SS -> active LOW)
// BUSY,9         <---             BUSY
// Reset,7 <-> HV2 - LV2       --> RST

#include <PN5180.h>
#include <PN5180ISO14443.h>

#define PN5180_NSS 10
#define PN5180_BUSY 9
#define PN5180_RST 7
// #define PIN_TRIGGER 4 // управление питанием PN5180

PN5180ISO14443 nfc(PN5180_NSS, PN5180_BUSY, PN5180_RST);
bool PN5180ISO14443_start();
void printCardSerial_ATQA_SAK();
uint32_t irqStatus = 0;
uint32_t loopCnt = 0;
bool errorFlag = false;
uint8_t productVersion[2];

void setup()
{
  // pinMode(PIN_TRIGGER, OUTPUT);    // Назначаем пин питания PN5180
  // digitalWrite(PIN_TRIGGER, HIGH); // Включаем питание PN5180
  // delay(30);                       // Ждем 30 мс для стабилизации питания

  Serial.begin(9600);

  while (PN5180ISO14443_start() == false)
  {
    Serial.println(F("PN5180 not detected!"));
    Serial.println(F("Please check wiring and power supply!"));
    Serial.println(F("Restarting PN5180..."));
    Serial.flush();
    delay(2000); // wait for a second before retrying
  }
}

// ISO 14443 loop
void loop()
{
  if (errorFlag)
  {
    irqStatus = nfc.getIRQStatus();
    nfc.showIRQStatus(irqStatus);

    if (0 == (RX_SOF_DET_IRQ_STAT & irqStatus))
    {
      Serial.println(F("*** ERROR! So many cards in RF field, maybe? ***"));
    }

    nfc.reset();
    nfc.setupRF();

    errorFlag = false;
    delay(10);
  }
  Serial.println(F("------------------------------------------------"));
  Serial.print(F("Loop #"));
  Serial.println(loopCnt++);
  if (!nfc.isCardPresent())
  {
    Serial.println(F("no card found"));

    irqStatus = nfc.getIRQStatus();
    nfc.showIRQStatus(irqStatus);

    if (irqStatus != 0x24007)
    {
      Serial.println(F("Error: Unexpected IRQ status (not 0x24007)"));
      errorFlag = true;
    }

    delay(1500);
    return;
  }

  printCardSerial_ATQA_SAK();
  delay(2500); // wait a second before next loop
}

// Function to start the PN5180 ISO14443
bool PN5180ISO14443_start()
{
  // digitalWrite(PIN_TRIGGER, LOW); // Выключаем питание PN5180
  Serial.println(F("================================================"));
  Serial.println(F("Uploaded: " __DATE__ " " __TIME__));
  Serial.println(F("PN5180 ISO14443 Sketch for Mifare Ultralight EV1"));

  Serial.println(F("------------------------------------------------"));
  nfc.begin();
  Serial.println(F("PN5180 Hard-Reset..."));
  nfc.reset();
  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Reading PN5180 version..."));
  nfc.readEEprom(PRODUCT_VERSION, productVersion, sizeof(productVersion));
  Serial.print(F("PN5180 version="));
  Serial.print(productVersion[1]);
  Serial.print(".");
  Serial.println(productVersion[0]);

  if (productVersion[1] != 4)
  { // if product version is not 4, the initialization failed

    return false; // return error
  }

  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Reading firmware PN5180 version..."));
  uint8_t firmwareVersion[2];
  nfc.readEEprom(FIRMWARE_VERSION, firmwareVersion, sizeof(firmwareVersion));
  Serial.print(F("Firmware PN5180 version="));
  Serial.print(firmwareVersion[1]);
  Serial.print(".");
  Serial.println(firmwareVersion[0]);

  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Reading EEPROM PN5180 version..."));
  uint8_t eepromVersion[2];
  nfc.readEEprom(EEPROM_VERSION, eepromVersion, sizeof(eepromVersion));
  Serial.print(F("EEPROM PN5180 version="));
  Serial.print(eepromVersion[1]);
  Serial.print(".");
  Serial.println(eepromVersion[0]);

  Serial.println(F("------------------------------------------------"));
  Serial.println(F("Enable RF field..."));
  nfc.setupRF();

  return true;
}

void printCardSerial_ATQA_SAK()
{
  uint8_t buffer[10] = {0}; // 0-1: ATQA, 2: SAK, 3-9: UID
                            // uint8_t uidLength = nfc.readCard_UL_EV1(buffer);
  uint8_t uidLength = nfc.readCard_UL_EV1(buffer);

  if (!uidLength)
  {
    Serial.println(F("Error in readCard: "));
    errorFlag = true;
    return;
  }

  // --- UID ---
  Serial.print(F("UID: "));
  for (int i = 3; i < 3 + uidLength; i++)
  {
    if (i > 3)
      Serial.print(":"); // Разделитель
    char byteStr[4];
    snprintf(byteStr, sizeof(byteStr), "%02X", buffer[i]);
    Serial.print(byteStr);
  }
  Serial.println();

  // --- SAK ---
  char sakStr[12];
  snprintf(sakStr, sizeof(sakStr), "SAK: 0x%02X", buffer[2]);
  Serial.println(sakStr);

  // --- ATQA ---
  char atqaStr[16];
  snprintf(atqaStr, sizeof(atqaStr), "ATQA: 0x%02X%02X", buffer[1], buffer[0]); // порядок [1][0] = High:Low
  Serial.println(atqaStr);

  Serial.println(F("------------------------------------------------"));
}
