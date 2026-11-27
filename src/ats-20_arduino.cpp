/*
  This sketch SHOULD work with the Chinese KIT sold on AliExpress, eBay and Amazon
  ATTENTION:
  IT IS NOT TESTED ENOUGH. SO, DO NOT TRY IT IF YOU DON'T KNOW WHAT ARE YOU DOING.
  YOU MUST BE ABLE TO GO BACK TO THE PREVIOUS VERSION IF THIS SKETCH DOES NOT WORK FOR YOU.

  New features:

   1) The final code (HEX file) is about 2.5K smaller than the previous one (now  you have more memory to add new features);
   2) All the previous status of the receiver can be rescued when you turn it on (including SSB mode, bandwidth, volume, frequency, BFO etc);
   3) The bandwidth now is a property of the band (you can use different bandwidth for different bands);
   4) Bandwidth control on FM mode (Auto, 110, 84, 60 and 40 kHz);
   5) FM/RDS presentation improved;
   6) The seek function was improved (it is more precise on FM mode). The seek direction is controlled by the encoder (clockwise or counter-clockwise . Press encoder push button for seeking;
   7) Steps: 1, 5, 9, 10 and 50 kHz;
   8) Now you can configure MW band space to 9 or 10 kHz;
   9) New FM band from 64 to 84 MHz;
  10) Added a MW band for Europe, Africa and Asia
  11) The frequency on Display is bigger than the previous version;
  12) Now the bandwidth sequence is ordered by bandwidth values.

  ATTENTION: Turn your receiver on with the encoder push button pressed at first time to RESET the eeprom content.

  ARDUINO LIBRARIES:
  1) This sketch uses the Rotary Encoder Class implementation from Ben Buxton (the source code is included together with this sketch). You do not need to install it;
  2) Tiny4kOLED Library and TinyOLED-Fonts (on your Arduino IDE, look for this library on Tools->Manage Libraries).
  3) PU2CLR SI4735 Arduino Library (on your Arduino IDE look for this library on Tools->Manage Libraries).


  ABOUT THE EEPROM:

  ATMEL says the lifetime of an EEPROM memory position is about 100,000 writes.
  For this reason, this sketch tries to avoid save unnecessary writes into the eeprom.
  So, the condition to store any status of the receiver is changing the frequency,  bandwidth, volume, band or step  and 10 seconds of inactivity.
  For example, if you switch the band and turn the receiver off immediately, no news information will be written into the eeprom.
  But you wait 10 seconds after changing anything, all new information will be written.


  ABOUT SSB PATCH:

  First of all, it is important to say that the SSB patch content is not part of this library. The paches used here were made available by Mr.
  Vadim Afonkin on his Dropbox repository. It is important to note that the author of this library does not encourage anyone to use the SSB patches
  content for commercial purposes. In other words, this library only supports SSB patches, the patches themselves are not part of this library.

  In this context, a patch is a piece of software used to change the behavior of the SI4735 device.
  There is little information available about patching the SI4735 or Si4732 devices. The following information is the understanding of the author of
  this project and it is not necessarily correct. A patch is executed internally (run by internal MCU) of the device.
  Usually, patches are used to fixes bugs or add improvements and new features of the firmware installed in the internal ROM of the device.
  Patches to the SI4735 are distributed in binary form and have to be transferred to the internal RAM of the device by
  the host MCU (in this case Arduino). Since the RAM is volatile memory, the patch stored into the device gets lost when you turn off the system.
  Consequently, the content of the patch has to be transferred again to the device each time after turn on the system or reset the device.

  ATTENTION: The author of this project does not guarantee that procedures shown here will work in your development environment.
  Given this, it is at your own risk to continue with the procedures suggested here.
  This library works with the I2C communication protocol and it is designed to apply a SSB extension PATCH to CI SI4735-D60.
  Once again, the author disclaims any liability for any damage this procedure may cause to your SI4735 or other devices that you are using.

  Features of this sketch:

  Prototype documentation: https://pu2clr.github.io/SI4735/
  PU2CLR Si47XX API documentation: https://pu2clr.github.io/SI4735/extras/apidoc/html/

  By Ricardo Lima Caratti, April  2021.
*/

#include <Arduino.h>

#include <SI4735.h>
#include <EEPROM.h>
#include <Tiny4kOLED.h>
#include <font8x16atari.h> // Please, install the TinyOLED-Fonts library
#include "Rotary.h"

#include "patch_ssb_compressed.h"    // Compressed SSB patch version (saving almost 1KB)

const size_t size_content = sizeof ssb_patch_content; // See ssb_patch_content.h
const size_t cmd_0x15_size = sizeof cmd_0x15;         // Array of lines where the 0x15 command occurs in the patch content.

#define FM_BAND_TYPE 0
#define MW_BAND_TYPE 1
#define SW_BAND_TYPE 2
#define LW_BAND_TYPE 3

// OLED Diaplay constants
#define RST_PIN -1        // Define proper RST_PIN if required.

#define RESET_PIN 12

// Enconder PINs
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3

// Buttons controllers
#define MODE_SWITCH 4      // Switch MODE (Am/LSB/USB)
#define BANDWIDTH_BUTTON 5 // Used to select the banddwith. Values: 1.2, 2.2, 3.0, 4.0, 0.5, 1.0 kHz
#define VOL_UP 6           // Volume Up
#define VOL_DOWN 7         // Volume Down
#define BAND_BUTTON_UP 8   // Next band
#define BAND_BUTTON_DOWN 9 // Previous band
#define AGC_SWITCH 11      // Switch AGC ON/OF
#define STEP_SWITCH 10     // Used to select the increment or decrement frequency step (1, 5 or 10 kHz)
#define BFO_SWITCH 14      // Used to select the enconder control (BFO or VFO)

#define BAT_VOLTAGE_PIN PIN_A2

#define MIN_ELAPSED_TIME 100
#define MIN_ELAPSED_RSSI_TIME 150

#define DEFAULT_VOLUME 20 // change it for your favorite sound volume

#define FM 0
#define LSB 1
#define USB 2
#define AM 3
#define LW 4

#define SSB 1

#define STORE_TIME 10000 // Time of inactivity to make the current receiver status writable (10s / 10000 milliseconds).

#define ARRAY_SIZE(x)   (sizeof(x)/sizeof(*(x)))

const uint8_t app_id =  39; // Useful to check the EEPROM content before processing useful data
const int eeprom_address = 0;
unsigned long storeTime = 0;

const char *bandModeDesc[] = {"FM ", "LSB", "USB", "AM "};
uint8_t currentMode = FM;
uint8_t seekDirection = 1;

bool bfoOn = false;

bool ssbLoaded = false;
bool fmStereo = true;

int currentBFO = 0;

unsigned long elapsedRSSI = 0;
unsigned long elapsedButton = 0;

// Encoder control variables
volatile int8_t encoderCount = 0;

// Some variables to check the SI4735 status
uint16_t currentFrequency;
uint16_t previousFrequency;
uint16_t minFreq;
uint16_t maxFreq;
uint16_t bandWidth;
uint8_t currentStep = 1;
uint8_t currentBFOStep = 25;

// Datatype to deal with bandwidth on AM, SSB and FM in numerical order.
// Ordering by bandwidth values.
typedef struct
{
    uint8_t idx;      // SI473X device bandwidth index value
    const char *desc; // bandwidth description
} Bandwidth;

Bandwidth bandwidthSSB[] = {
    {4, "0.5"},  // 0
    {5, "1.0"},  // 1
    {0, "1.2"},  // 2
    {1, "2.2"},  // 3
    {2, "3.0"},  // 4  - default
    {3, "4.0"}   // 5
}; // 3 = 4kHz
#define bwIdxSSB_default 4
#define bwIdxSSB_last (ARRAY_SIZE(bandwidthSSB) - 1)
uint8_t bwIdxSSB = bwIdxSSB_default;

const int maxFilterAM = 15;
Bandwidth bandwidthAM[] = {
    {4, "1.0"},   // 0
    {5, "1.8"},   // 1
    {3, "2.0"},   // 2
    {6, "2.5"},   // 3
    {2, "3.0"},   // 4 - default
    {1, "4.0"},   // 5
    {0, "6.0"}    // 6
};
#define bwIdxAM_default 4
#define bwIdxAM_last (ARRAY_SIZE(bandwidthAM) - 1)
uint8_t bwIdxAM = bwIdxAM_default;

Bandwidth bandwidthFM[] = {
    {0, "AUT"}, // Automatic - default
    {1, "110"}, // Force wide (110 kHz) channel filter.
    {2, " 84"},
    {3, " 60"},
    {4, " 40"}
};
#define bwIdxFM_default 0
#define bwIdxFM_last (ARRAY_SIZE(bandwidthFM) - 1)
uint8_t bwIdxFM = bwIdxFM_default;

// Atenuação and AGC
uint8_t agcIdx = 0;
uint8_t disableAgc = 0;
uint8_t agcNdx = 0;

/*
   Band data structure
*/
typedef struct
{
    uint8_t bandType;     // Band type (FM, MW or SW)
    uint16_t minimumFreq; // Minimum frequency of the band
    uint16_t maximumFreq; // maximum frequency of the band
    uint16_t currentFreq; // Default frequency or current frequency
    uint16_t currentStep; // Defeult step (increment and decrement)
    uint8_t  bandwidth;  // Bandwidth local table index.
} Band;

/*
   Band table
   To add a new band, all you have to do is insert a new line in the table below. No extra code will be needed.
   Remove or comment a line if you do not want a given band
   You have to RESET the eeprom after modiging this table.
   Turn your receiver on with the encoder push button pressed at first time to RESET the eeprom content.
*/
Band band[] = {
    {FM_BAND_TYPE, 6400,  8400,  7000, 10, 0},  // FM from 64 to 84 MHz
    {FM_BAND_TYPE, 8400, 10800, 10570, 10, 0},
    {LW_BAND_TYPE,   149,   530,   300, 1, 4},
    {MW_BAND_TYPE,   531,  1701,   783, 9, 4},  // MW for Europe, Africa and Asia
    {SW_BAND_TYPE,  1800,  3500,  1900, 5, 4},  // 160 meters
    {SW_BAND_TYPE,  3500,  3800,  3650, 1, 4},  // 80 meters amateur
    {SW_BAND_TYPE,  3800,  4500,  3700, 5, 4},  // 80 meters broadcast
    {SW_BAND_TYPE,  4500,  7000,  6000, 5, 4},
    {SW_BAND_TYPE,  7000,  7200,  7100, 1, 4},  // 40 meters amateur
    {SW_BAND_TYPE,  7200,  7800,  7200, 5, 4},  // 40 meters
    {SW_BAND_TYPE,  7800, 14000,  9600, 5, 4},
    {SW_BAND_TYPE, 14000, 14500, 14200, 5, 4},  // 20 meters
    {SW_BAND_TYPE, 14500, 26200, 15000, 5, 4},
    {SW_BAND_TYPE, 26200, 27900, 27500, 1, 4},  // CB band (11 meters)
    {SW_BAND_TYPE, 27900, 30000, 28400, 5, 4}   // 10 meters
};
#define band_default 1
#define band_last (ARRAY_SIZE(band) - 1)
uint8_t bandIdx = band_default;

uint8_t rssi = 0;
uint8_t stereo = 1;
uint8_t volume = DEFAULT_VOLUME;

// Devices class declarations
Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);
SI4735 si4735;

// Use Rotary.h and  Rotary.cpp implementation to process encoder via interrupt
void rotaryEncoder()
{
    uint8_t encoderStatus = encoder.process();
    int8_t ecnt = encoderCount;

    switch (encoderStatus) {
    case DIR_CW:
      if (ecnt < 127)
        ecnt++;
      break;

    case DIR_CCW:
      if (ecnt > -127)
        ecnt--;
      break;

    default:
      return;
    }

    encoderCount = ecnt;
}

/*
 * Clean the content of the third line (line 2 - remember the first line is 0)
 */
void cleanBfoRdsInfo()
{
    oled.setCursor(0, 2);
    oled.print("                    ");
}

/*
   This function loads the contents of the ssb_patch_content array into the CI (Si4735) and starts the radio on
   SSB mode.
*/
void loadSSB()
{
    oled.setCursor(0, 2);
    oled.print("  Switching to SSB  ");
    // si4735.setI2CFastModeCustom(850000); // It is working. Faster, but I'm not sure if it is safe.
    si4735.setI2CFastModeCustom(500000);
    si4735.queryLibraryId(); // Is it really necessary here? I will check it.
    si4735.patchPowerUp();
    delay(50); // why ?
    si4735.downloadCompressedPatch(ssb_patch_content, size_content, cmd_0x15, cmd_0x15_size);
    si4735.setSSBConfig(bandwidthSSB[bwIdxSSB].idx, 1, 0, 1, 0, 1);
    si4735.setI2CStandardMode();
    ssbLoaded = true;
    // oled.clear();
    cleanBfoRdsInfo();
}

/**
 * Converts a number to a char string and places leading zeros.
 * It is useful to mitigate memory space used by sprintf or generic similar function
 *
 * value  - value to be converted
 * strValue - the value will be receive the value converted
 * len -  final string size (in bytes)
 * dot - the decimal or tousand separator position
 */
void convertToChar(uint16_t value, char *strValue, uint8_t len, uint8_t dot)
{
    char d;
    for (int i = (len - 1); i >= 0; i--) {
        d = value % 10;
        value = value / 10;
        strValue[i] = d + '0';
    }
    strValue[len] = '\0';
    if (dot > 0) {
    for (int i = len; i >= dot ; i-- ) {
        strValue[i + 1] = strValue[i];
    }
    strValue[dot] = '.';
  }

  if (strValue[0] == '0') {
        strValue[0] = ' ';
        if (strValue[1] == '0') {
            strValue[1] = ' ';
        }
  }
}

// Show current frequency
void showFrequency()
{
    const char *unit;
    const char *bandMode;
    char freqDisplay[10];

    if (band[bandIdx].bandType == FM_BAND_TYPE) {
        convertToChar(currentFrequency, freqDisplay, 5, 3);
        unit = "MHz";
    } else {
        unit = "kHz";
        uint8_t dot = (band[bandIdx].bandType == MW_BAND_TYPE ||
            band[bandIdx].bandType == LW_BAND_TYPE) ? 0 : 2;

        convertToChar(currentFrequency, freqDisplay, 5, dot);
    }

    oled.invertOutput(bfoOn);

    oled.setFont(FONT8X16ATARI);
    oled.setCursor(34, 0);
    oled.print("      ");
    oled.setCursor(34, 0);
    oled.print(freqDisplay);
    oled.setFont(FONT6X8);
    oled.invertOutput(false);

    bandMode = currentFrequency < 520 ? "LW  " : bandModeDesc[currentMode];

    oled.setCursor(0, 0);
    oled.print(bandMode);

    oled.setCursor(95, 0);
    oled.print(unit);
}


/* *******************************
   Shows RSSI status
*/
void showRSSI()
{
    int bars = (rssi / 20.0) + 1;
    oled.setCursor(90, 3);
    oled.print("      ");
    oled.setCursor(90, 3);
    oled.print(".");
    for (int i = 0; i < bars; i++) {
        oled.print('_');
    }
    oled.print('|');

    if (currentMode == FM) {
        oled.setCursor(18, 0);
        oled.print("  ");
        oled.setCursor(18, 0);
        oled.invertOutput(true);
        if (si4735.getCurrentPilot()) {
            oled.invertOutput(true);
            oled.print("s");
        }
        oled.invertOutput(false);
    }
}

/*
   Shows the volume level on LCD
*/
void showVolume()
{
    oled.setCursor(60, 3);
    oled.print("  ");
    oled.setCursor(60, 3);
    oled.print(si4735.getCurrentVolume());
}

/*
   Shows the BFO current status.
   Must be called only on SSB mode (LSB or USB)
*/
void showBFO()
{
    oled.setCursor(0, 2);
    oled.print("BFO: ");
    oled.print(currentBFO);
    oled.print(" Hz");

    oled.fillLength(0, 93 - oled.getCursorX());

    oled.setCursor(93, 2);
    oled.print("S:");
    oled.print(currentBFOStep);
}

char *stationName;
char bufferStatioName[20];
unsigned long rdsElapsed = 0;

char oldBuffer[15];

/*
 * Show the Station Name.
 */
void showRDSStation()
{
    char *po, *pc;
    int col = 0;

    po = oldBuffer;
    pc = stationName;
    while (*pc) {
        if ( *po != *pc ) {
            oled.setCursor(col, 2);
            oled.print(*pc);
        }
        *po = *pc;
        po++;
        pc++;
        col+=10;
    }
    // strcpy(oldBuffer, stationName);
    delay(120); // why?
}

/**
   SHow bandwidth on AM,SSB and FM mode

*/
void showBandwidth()
{
    char bw[10];

    strcpy(bw, "BW: ");
    if (currentMode == LSB || currentMode == USB) {
        strcat(bw, bandwidthSSB[bwIdxSSB].desc);
        showBFO();
    } else if (currentMode == AM) {
        strcat(bw, bandwidthAM[bwIdxAM].desc);
    } else {
        strcat(bw, bandwidthFM[bwIdxFM].desc);
    }
    oled.setCursor(0, 3);
    oled.print("          ");
    oled.setCursor(0, 3);
    oled.print(bw);
}

/*
    Show some basic information on display
*/
void showStatus()
{
    showFrequency();

    oled.setCursor(93, 1);
    oled.print("      ");
    oled.setCursor(93, 1);
    oled.print("S:");
    oled.print(currentStep);

    showBandwidth();

    // Show AGC Information
    // si4735.getAutomaticGainControl();
    oled.setCursor(0, 1);
    oled.print("     ");
    oled.setCursor(0, 1);
    if (agcIdx == 0 ) {
        oled.print("AGC");
    } else {
        oled.print("- ");
        oled.print(agcNdx);
    }

    showRSSI();
    showVolume();
}

/*
 * To store any change into the EEPROM, it is needed at least STORE_TIME  milliseconds of inactivity.
 */
void resetEepromDelay()
{
    storeTime = millis();
    previousFrequency = 0;
}

/*
   Switch the radio to current band.
   The bandIdx variable points to the current band.
   This function change to the band referenced by bandIdx (see table band).
*/
void useBand()
{
    cleanBfoRdsInfo();

    minFreq = band[bandIdx].minimumFreq;
    maxFreq = band[bandIdx].maximumFreq;
    bandWidth = maxFreq - minFreq + 1;

    if (band[bandIdx].bandType == FM_BAND_TYPE) {
        currentMode = FM;
        si4735.setTuneFrequencyAntennaCapacitor(0);
        si4735.setFM(minFreq, maxFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
        si4735.setSeekFmLimits(minFreq, maxFreq);
        si4735.setSeekFmSpacing(1);
        bfoOn = ssbLoaded = false;
        si4735.setRdsConfig(1, 2, 2, 2, 2);
        bwIdxFM = band[bandIdx].bandwidth;
        si4735.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
    } else {
        if (band[bandIdx].bandType == MW_BAND_TYPE || band[bandIdx].bandType == LW_BAND_TYPE) {
            si4735.setTuneFrequencyAntennaCapacitor(0);
        } else {
            si4735.setTuneFrequencyAntennaCapacitor(1);
        }

        if (ssbLoaded) {
            si4735.setSSB(minFreq, maxFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep, currentMode);
            si4735.setSSBAutomaticVolumeControl(1);
            si4735.setSsbSoftMuteMaxAttenuation(0); // Disable Soft Mute for SSB
            bwIdxSSB = band[bandIdx].bandwidth;
            si4735.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
            si4735.setSSBBfo(currentBFO);
        } else {
            currentMode = AM;
            si4735.setAM(minFreq, maxFreq, band[bandIdx].currentFreq, band[bandIdx].currentStep);
            si4735.setAutomaticGainControl(disableAgc, agcNdx);
            si4735.setAmSoftMuteMaxAttenuation(6); // // Disable Soft Mute for AM
            bwIdxAM = band[bandIdx].bandwidth;
            si4735.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
            bfoOn = false;
        }
        si4735.setSeekAmLimits(minFreq, maxFreq);           // Consider the range all defined current band
        si4735.setSeekAmSpacing((band[bandIdx].currentStep > 10) ? 10 : band[bandIdx].currentStep); // Max 10kHz for spacing
    }

    delay(100); // why ?
    currentFrequency = band[bandIdx].currentFreq;
    currentStep = band[bandIdx].currentStep;
    showStatus();
    resetEepromDelay();
}

/*
   EEPROM receiver status
*/

// TODO: refactor config storage

void saveAllReceiverInformation()
{
    int addr = eeprom_address;
    EEPROM.update(addr++, app_id); // stores the app id;
    EEPROM.update(addr++, si4735.getVolume()); // stores the current Volume
    EEPROM.update(addr++, bandIdx); // Stores the current band
    EEPROM.update(addr++, currentMode); // Stores the current Mode (FM / AM / SSB)
    EEPROM.update(addr++, currentBFO >> 8);
    EEPROM.update(addr++, currentBFO &  0XFF);

    band[bandIdx].currentFreq = currentFrequency;

    for (size_t i = 0; i <= band_last; i++) {
        EEPROM.update(addr++, (band[i].currentFreq >> 8) );   // stores the current Frequency HIGH byte for the band
        EEPROM.update(addr++, (band[i].currentFreq & 0xFF));  // stores the current Frequency LOW byte for the band
        EEPROM.update(addr++, band[i].currentStep);          // Stores current step of the band
        EEPROM.update(addr++, band[i].bandwidth);           // table index (direct position) of bandwidth
    }
}


void readAllReceiverInformation()
{
    int addr = eeprom_address;
    // Checking the EEPROM content
    if (EEPROM.read(addr++) != app_id) {
        return;
    }

    volume = EEPROM.read(addr++); // Gets the stored volume;
    bandIdx = EEPROM.read(addr++);
    currentMode = EEPROM.read(addr++);
    currentBFO = EEPROM.read(addr++) << 8;
    currentBFO |= EEPROM.read(addr++);

    for (size_t i = 0; i <= band_last; i++) {
        band[i].currentFreq = EEPROM.read(addr++) << 8;
        band[i].currentFreq |= EEPROM.read(addr++);
        band[i].currentStep = EEPROM.read(addr++);
        band[i].bandwidth = EEPROM.read(addr++);
    }

    if (bandIdx > band_last) {
        bandIdx = band_default;
    }

    previousFrequency = currentFrequency = band[bandIdx].currentFreq;
    currentStep = band[bandIdx].currentStep;
    uint8_t bwIdx = band[bandIdx].bandwidth;

    if (currentMode == LSB || currentMode == USB) {
        loadSSB();
        bwIdxSSB = (bwIdx > bwIdxSSB_last) ? bwIdxSSB_default : bwIdx;
        si4735.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
        // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
        si4735.setSSBSidebandCutoffFilter(bwIdxSSB > 2);
   } else if (currentMode == AM) {
        bwIdxAM = (bwIdx > bwIdxAM_last) ? bwIdxAM_default : bwIdx;
        si4735.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
   } else {
        bwIdxFM = (bwIdx > bwIdxFM_last) ? bwIdxFM_default : bwIdx;
        si4735.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
   }
}

/**
    This function is called by the seek function process.
*/
void showFrequencySeek(uint16_t freq)
{
    currentFrequency = freq;
    showFrequency();
}

/**
   Checks the stop seeking criterias.
   Returns true if the user press the touch or rotates the encoder.
*/
bool checkStopSeeking()
{
    // Checks the touch and encoder
    return (bool) encoderCount || (digitalRead(BFO_SWITCH) == LOW); // returns true if the user rotates the encoder or press the push button
}

/*
 * Checks the station name is available
 */
void checkRDS()
{
    si4735.getRdsStatus();
    if (si4735.getRdsReceived() && si4735.getRdsSync() && si4735.getRdsSyncFound()) {
        stationName = si4735.getRdsText0A();
        // BUG: RDS update may be lost once due millis() wrap
        // TODO: use separate flag for RDS update
        if (stationName != NULL && (millis() - rdsElapsed) > 10 ) {
            showRDSStation();
            rdsElapsed = millis();
        }
    }
}

/*
   Goes to the next band (see Band table)
*/
void bandUp()
{
    // save the current frequency for the band
    band[bandIdx].currentFreq = currentFrequency;
    band[bandIdx].currentStep = currentStep;

    if (++bandIdx > band_last) {
        bandIdx = 0;
    }
    useBand();
}

/*
   Goes to the previous band (see Band table)
*/
void bandDown()
{
    // save the current frequency for the band
    band[bandIdx].currentFreq = currentFrequency;
    band[bandIdx].currentStep = currentStep;

    if (--bandIdx > band_last) {
        bandIdx = band_last;
    }
    useBand();
}

void show_battery_voltage()
{
    // TODO: check voltage divider presense
    // Assuming reference is AVCC

    analogRead(BAT_VOLTAGE_PIN); // discard first read

    uint8_t count = 3;
    do {
        auto vbat_raw = analogRead(BAT_VOLTAGE_PIN);
        // Inaccurate but simple
        auto vbat_cv = vbat_raw * 123ul / 256;     // Vref * 1.5 / 1023

        char vbat_str[5]; // TODO: check convertToChar() correctness
        convertToChar(vbat_cv, vbat_str, sizeof(vbat_str)-1, 2);

        oled.setCursor(0, 3);
        oled.print("Vbat: ");
        oled.print(vbat_str);
        oled.print(" V");
        delay(250); // TODO: don't wait 250 ms if BAND- was pressed but then released

        if (count) {
            --count;
        }
        // Hold BAND- to extend splash)
    } while (count || (digitalRead(BAND_BUTTON_DOWN) == LOW));
}

void setup()
{
    // Encoder pins
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);

    pinMode(BANDWIDTH_BUTTON, INPUT_PULLUP);
    pinMode(BAND_BUTTON_UP, INPUT_PULLUP);
    pinMode(BAND_BUTTON_DOWN, INPUT_PULLUP);
    pinMode(VOL_UP, INPUT_PULLUP);
    pinMode(VOL_DOWN, INPUT_PULLUP);
    pinMode(BFO_SWITCH, INPUT_PULLUP);
    pinMode(AGC_SWITCH, INPUT_PULLUP);
    pinMode(STEP_SWITCH, INPUT_PULLUP);
    pinMode(MODE_SWITCH, INPUT_PULLUP);

    oled.begin();
    oled.clear();
    oled.on();
    oled.setFont(FONT6X8);

    // Splash - Change it for your introduction text.
    oled.setCursor((128-6*6)/2, 0);
    oled.print("Si473X");
    oled.setCursor((128-16*6)/2, 1);
    oled.print("V3.0.4 by PU2CLR");

    oled.setCursor(0, 2);
    oled.print("FW " SOURCE_VERSION);

    // If you want to reset the eeprom, keep the ENCODER_PUSH_BUTTON (BFO_SWITCH) button pressed during statup
    if (digitalRead(BFO_SWITCH) == LOW)
    {
        EEPROM.write(eeprom_address, 0);
        oled.setCursor((128-14*6)/2, 3);
        oled.print("EEPROM RESET");
        delay(1000);
        oled.setCursor((128-14*6)/2, 3);
        oled.print("            ");
    }

    show_battery_voltage();

    // end Splash

    // Encoder interrupt
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

    si4735.getDeviceI2CAddress(RESET_PIN); // Looks for the I2C bus address and set it.  Returns 0 if error

    si4735.setup(RESET_PIN, MW_BAND_TYPE);

    readAllReceiverInformation();

    // Set up the radio for the current band (see index table variable bandIdx )
    useBand();

    currentFrequency = previousFrequency = si4735.getFrequency();

    si4735.setVolume(volume);
    oled.clear();
    showStatus();
}

struct intGuard {
    intGuard()  { noInterrupts(); }
    ~intGuard() { interrupts();   }
};

static inline void process_encoder()
{
    int8_t ecnt;

    {
        intGuard guard;
        ecnt = encoderCount;

        if (ecnt == 0) {
            return;
        }

        encoderCount = 0;
    }

    resetEepromDelay();

    if (bfoOn) {
        // TODO: clamp to ±16383
        currentBFO += currentBFOStep * ecnt;
        si4735.setSSBBfo(currentBFO);
        previousFrequency = 0;  // Forces eeprom update
        showBFO();
        return;
    }

    // TODO: come up with nice wrap-around
    // TODO: avoid dividions and overcomplicated arithmetics:
    //       separate positive / negative ecnt branches
    // TODO: consider snap to grid
    unsigned int position = currentFrequency - minFreq;
    unsigned int channel = position / currentStep;
    unsigned int offset = position % currentStep;
    unsigned int nchannels = (bandWidth + currentStep - offset - 1) / currentStep;

    channel = ((static_cast<int>(channel) + ecnt) % static_cast<int>(nchannels)
                + nchannels) % nchannels;

    currentFrequency = minFreq + (channel * currentStep + offset) % bandWidth;

    si4735.setFrequency(currentFrequency);

    seekDirection = (ecnt > 0) ? 1 : 0;

    // Show the current frequency only if it has changed
    currentFrequency = si4735.getFrequency();
    showFrequency();
}

static inline void process_keys()
{
    // Check button commands
    if (millis() - elapsedButton <= MIN_ELAPSED_TIME) {
        return;
    }

    // check if some button is pressed
    if (digitalRead(BANDWIDTH_BUTTON) == LOW) {
        if (currentMode == LSB || currentMode == USB)
        {
            bwIdxSSB++;
            if (bwIdxSSB > bwIdxSSB_last)
            bwIdxSSB = 0;
            band[bandIdx].bandwidth = bwIdxSSB;
            si4735.setSSBAudioBandwidth(bandwidthSSB[bwIdxSSB].idx);
            // If audio bandwidth selected is about 2 kHz or below, it is recommended to set Sideband Cutoff Filter to 0.
            si4735.setSSBSidebandCutoffFilter(bwIdxSSB > 2);
        } else if (currentMode == AM) {
            bwIdxAM++;
            if (bwIdxAM > bwIdxAM_last)
            bwIdxAM = 0;
            band[bandIdx].bandwidth = bwIdxAM;
            si4735.setBandwidth(bandwidthAM[bwIdxAM].idx, 1);
        } else {
            bwIdxFM++;
            if (bwIdxFM > bwIdxFM_last)
            bwIdxFM = 0;
            else if (bwIdxFM < 0)
            bwIdxFM = bwIdxFM_last;
            band[bandIdx].bandwidth = bwIdxFM;
            si4735.setFmBandwidth(bandwidthFM[bwIdxFM].idx);
        }
        showStatus();
        resetEepromDelay();
        delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
    } else if (digitalRead(BAND_BUTTON_UP) == LOW) {
        bandUp();
    } else if (digitalRead(BAND_BUTTON_DOWN) == LOW) {
        bandDown();
    } else if (digitalRead(VOL_UP) == LOW) {
        si4735.volumeUp();
        volume = si4735.getVolume();
        showVolume();
        delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
    } else if (digitalRead(VOL_DOWN) == LOW) {
        si4735.volumeDown();
        volume = si4735.getVolume();
        showVolume();
        delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
    } else if (digitalRead(BFO_SWITCH) == LOW) {
        if (currentMode == LSB || currentMode == USB) {
            bfoOn = !bfoOn;
            if (bfoOn) {
                showBFO();
            }
            showStatus();
        } else if (currentMode == FM || currentMode == AM) {
            // Jumps up or down one space
            if (seekDirection) {
                si4735.frequencyUp();
            } else {
                si4735.frequencyDown();
            }

            si4735.seekStationProgress(showFrequencySeek, checkStopSeeking, seekDirection);
            delay(30);
            if (currentMode == FM) {
                float f = round(si4735.getFrequency() / 10.0);
                currentFrequency = (uint16_t) f * 10; // adjusts band space from 1 (10kHz) to 10 (100 kHz)
                si4735.setFrequency(currentFrequency);
            } else {
                currentFrequency = si4735.getFrequency(); //
            }
            showFrequency();
        }
        delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
    } else if (digitalRead(AGC_SWITCH) == LOW) {
        if (agcIdx == 0) {
            disableAgc = 0; // Turns AGC ON
            agcNdx = 0;
            agcIdx = 1;
        } else if (agcIdx == 1) {
            disableAgc = 1; // Turns AGC OFF
            agcNdx = 0;     // Sets minimum attenuation
            agcIdx = 2;
        } else if (agcIdx == 2) {
            disableAgc = 1; // Turns AGC OFF
            agcNdx = 10;    // Increases the attenuation AM/SSB AGC Index  = 10
            agcIdx = 3;
        } else if (agcIdx == 3) {
            disableAgc = 1; // Turns AGC OFF
            agcNdx = 20;    // Increases the attenuation AM/SSB AGC Index  = 20
            agcIdx = 4;
        } else if (agcIdx == 4) {
            disableAgc = 1; // Turns AGC OFF
            agcNdx = 36;    // Sets maximum attenuation
            agcIdx = 0;
        }
        // Sets AGC on/off an gain
        si4735.setAutomaticGainControl(disableAgc, agcNdx);
        showStatus();
        resetEepromDelay();
    } else if (digitalRead(STEP_SWITCH) == LOW) {
        if (currentMode == FM) {
            fmStereo = !fmStereo;
            if (fmStereo) {
                si4735.setFmStereoOn();
            } else {
                si4735.setFmStereoOff(); // It is not working so far.
            }
        } else {
            // This command should work only for SSB mode
            if (bfoOn && (currentMode == LSB || currentMode == USB)) {
                currentBFOStep = (currentBFOStep == 25) ? 10 : 25;
                showBFO();
            } else {

                if (currentStep == 1)
                    currentStep = 5;
                else if (currentStep == 5)
                    currentStep = 9;
                else if (currentStep == 9)
                    currentStep = 10;
                else if (currentStep == 10)
                    currentStep = 50;
                else
                    currentStep = 1;

                si4735.setFrequencyStep(currentStep);
                band[bandIdx].currentStep = currentStep;
                si4735.setSeekAmSpacing((band[bandIdx].currentStep > 10) ? 10 : band[bandIdx].currentStep); // Max 10kHz for spacing
                showStatus();
            }
            resetEepromDelay();
            delay(MIN_ELAPSED_TIME); // waits a little more for releasing the button.
        }
    } else if (digitalRead(MODE_SWITCH) == LOW) {
        if (currentMode != FM) {
            if (currentMode == AM) {
                // If you were in AM mode, it is necessary to load SSB patch (avery time)
                loadSSB();
                currentMode = LSB;
            } else if (currentMode == LSB) {
                currentMode = USB;
            } else if (currentMode == USB) {
                currentMode = AM;
                ssbLoaded = false;
                bfoOn = false;
            }
            // Nothing to do if you are in FM mode
            band[bandIdx].currentFreq = currentFrequency;
            band[bandIdx].currentStep = currentStep;
            useBand();
        }
    }
    elapsedButton = millis();
}

void loop()
{
    process_encoder();

    process_keys();

    // Show RSSI status only if this condition has changed
    if ((millis() - elapsedRSSI) > MIN_ELAPSED_RSSI_TIME * 9) {
        si4735.getCurrentReceivedSignalQuality();
        int aux = si4735.getCurrentRSSI();
        if (rssi != aux) {
            rssi = aux;
            showRSSI();
        }
        elapsedRSSI = millis();
    }

    if (currentMode == FM) {
        if (currentFrequency != previousFrequency) {
            cleanBfoRdsInfo();
        }
        checkRDS();
    }

    // Show the current frequency only if it has changed
    if (currentFrequency != previousFrequency) {
        if (millis() - storeTime > STORE_TIME) {
            saveAllReceiverInformation();
            storeTime = millis();
            previousFrequency = currentFrequency;
        }
    }
    delay(10); // why ?
}
