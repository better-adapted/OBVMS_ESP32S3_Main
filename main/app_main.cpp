/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "app_main_defines.h"

#include "esp32-hal-gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <Arduino.h>
#include <WiFi.h>
#include <inttypes.h>
#include <stdio.h>

#include "ESP32S3_Common.h"
#include "esp_info.h"
#include <HTTPClient.h>

#ifdef APP_WIFI_MULTI_SUPPORT
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#endif

#include <hal/usb_serial_jtag_hal.h>
//#include <WiFiClientSecure.h>
//#include <NetworkClientSecure.h>

// todo
// reset wifi prov with "reset_provisioned" every time?
// green led would be better showing internet connected status not wifi connected! use a ping!


const char *hardware_version = {"OBVMS_Dongle"};
const char *firmware_version = {"0.0.0.1"};

#ifdef APP_SUPPORT_I2C
#include <Wire.h>
void Scan_i2c(Stream *pStream);
#endif

#ifdef GPIO_NEOPIXEL
#include <Adafruit_NeoPixel.h>
static uint8_t rgb_led_red_pwm;
static uint8_t rgb_led_green_pwm;
static uint8_t rgb_led_blue_pwm;
Adafruit_NeoPixel pixels(1, GPIO_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

uint32_t red_led_off_ms = 0;
int ReceivedPacket = 0;
uint32_t TxEntryTick;

void rgb_led_task();
void rgb_led_init();
void rgb_led_boot_pattern();
void RedledSet(uint8_t pRed); 
void GreenledSet(uint8_t pGreen); 
void BlueledSet(uint8_t pBlue);

#ifdef APP_SUPPORT_WIFI_PROV
#include <app_wifi_prov.h>
#endif

#ifdef APP_SUPPORT_CLI
#include <CLI.h>
void CLI_Service(Stream *pStream);
static uint32_t ESC_pressed = 0;
static App_debug_flags_t DebugFlags;
#endif

Stream *Stream_UART0 = nullptr;

void WDT_Feed(int pTrace, bool pPoll);
static bool WDT_feed_disable;

#ifdef APP_SUPPORT_SPI
#include <SPI.h>
#endif

#ifdef GPIO_NEOPIXEL
void rgb_led_set(uint8_t pRed, uint8_t pGreen, uint8_t pBlue)
{
	rgb_led_red_pwm=pRed;
	rgb_led_green_pwm=pGreen;
	rgb_led_blue_pwm=pBlue;
	pixels.setPixelColor(0, pixels.Color(rgb_led_red_pwm, rgb_led_green_pwm, rgb_led_blue_pwm));
	pixels.show();
}

void rgb_led_init() 
{ 
	rgb_led_boot_pattern();
}

void RedledSet(uint8_t pRed) 
{
	rgb_led_red_pwm=pRed;
	pixels.setPixelColor(0, pixels.Color(rgb_led_red_pwm, rgb_led_green_pwm, rgb_led_blue_pwm));
}

void GreenledSet(uint8_t pGreen) 
{  
	rgb_led_green_pwm=pGreen;
	pixels.setPixelColor(0, pixels.Color(rgb_led_red_pwm, rgb_led_green_pwm, rgb_led_blue_pwm));
}

void BlueledSet(uint8_t pBlue) 
{  
	rgb_led_blue_pwm=pBlue;
	pixels.setPixelColor(0, pixels.Color(rgb_led_red_pwm, rgb_led_green_pwm, rgb_led_blue_pwm));
}
#endif

static uint16_t crc_1021x(uint16_t old_crc, uint8_t data) {
	uint16_t crc;
	uint16_t x;

	// x = make8(old_crc,1) ^ data;
	x = ((old_crc >> 8) ^ data) & 0xff;
	x ^= x >> 4;

	crc = (old_crc << 8) ^ (x << 12) ^ (x << 5) ^ x;

	crc &= 0xffff; // enable this line for processors with more than 16 bits

	return crc;
}

static uint16_t Crc16(const uint8_t *ptr, int pSize) {
	volatile static uint16_t crc_citt;
	int x;
	crc_citt = 0xFFFF;
	for (x = 0; x < pSize; x++) {
		crc_citt = crc_1021x(crc_citt, ptr[x]);
	}
	return crc_citt;
}

void rgb_led_task() 
{
	static uint32_t green_flash_tick_start_ms = 0;
	static uint32_t green_flash_tick_active_ms = 0;
	
	if(wifi_prov_Active_ms()==0)
	{
		if ((millis() - green_flash_tick_start_ms) >= 950) {
			GreenledSet(0);
			green_flash_tick_start_ms = millis();
			green_flash_tick_active_ms = millis();
		}		
	}
	else
	{	
		static uint32_t prov_led_flash_tick_next;
		static bool prov_led_flash_toggle_state;
		if (millis() >= prov_led_flash_tick_next)
		{
			if(prov_led_flash_toggle_state)
			{
				RedledSet(0);
				GreenledSet(0);
				BlueledSet(0);
				prov_led_flash_tick_next=millis()+900;
				prov_led_flash_toggle_state=false;				
			}
			else
			{
				RedledSet(1);
				GreenledSet(1);
				BlueledSet(1);
				prov_led_flash_tick_next=millis()+100;				
				prov_led_flash_toggle_state=true;				
			}
			
		}
	}

	if (green_flash_tick_active_ms) {
		uint32_t diff = millis() - green_flash_tick_active_ms;

		if (diff >= 50) {
			GreenledSet(40);
			green_flash_tick_active_ms = 0;
		}
	}

	if (red_led_off_ms) 
	{
		if (millis() >= red_led_off_ms) 
		{
			red_led_off_ms = 0;
			RedledSet(0);
		}
	}	
}

void rgb_led_boot_pattern() {
	rgb_led_set(250, 0, 0); // setup()
	delay(500);
	rgb_led_set(0, 250, 0); // setup()
	delay(500);
	rgb_led_set(0, 0, 250); // setup()
	delay(500);
	rgb_led_set(0, 0, 0); // setup()
	delay(500);
}

String CLI_Prefix() {
	static String temp;
	char temp_str[100] = {};

	sprintf(temp_str, "@%010lu", millis());

	temp = temp_str;
	return temp;
}

void WDT_Feed(int pTrace, bool pPoll) {
	if (DebugFlags.bits.ShowWDTfeed) {
		Stream_UART0->printf("%s,WDT_Feed(%d,%d)\r\n", CLI_Prefix().c_str(),
							 pTrace, pPoll);
	}
}

void Set_wifi_credentials(String ssid, String pwd) {
	wifi_config_t current_conf;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
	memcpy(current_conf.sta.ssid, ssid.c_str(), sizeof(current_conf.sta.ssid));
	memcpy(current_conf.sta.password, pwd.c_str(),
		   sizeof(current_conf.sta.password));
	esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &current_conf);
}

#ifdef APP_SUPPORT_WIFI_PROV

void wifi_prov_WDT_Feed() {}

void wifi_prov_led_RGB(int pMode) {
	if (pMode == 0)
		rgb_led_set(0, 0, 0);

	if (pMode == 1)
		rgb_led_set(250, 255, 255);

	if (pMode == 2)
		rgb_led_set(0, 0, 0);
}

bool wifi_prov_switch_get()
{	
	return !digitalRead(GPIO_WIFI_PROV_SWT);
}
#endif // APP_SUPPORT_WIFI_PROV

bool ConfigSwitchGet(void) { return !digitalRead(GPIO_CONFIG_SWITCH); }

bool BootSwitchGet(void) { return !digitalRead(GPIO_BOOT_SWITCH); }

void Service_BootSwitch() {
	static bool boot_swt_prev = 0;
	bool boot_swt = BootSwitchGet();

	if (boot_swt_prev != boot_swt) {
		boot_swt_prev = boot_swt;
		Stream_UART0->printf("BootSwitchGet()=%d", (int)boot_swt);
		Stream_UART0->println();
	}
}

void Service_ConfigSwitch() {
	static bool config_swt_prev = 0;
	bool config_swt = ConfigSwitchGet();

	if (config_swt_prev != config_swt) {
		config_swt_prev = config_swt;
		Stream_UART0->printf("ConfigSwitchGet()=%d", (int)config_swt);
		Stream_UART0->println();		
	}
}

#ifdef APP_SUPPORT_CLI

void CLI_ESC_Req() { ESC_pressed = millis(); }

void CLI_ESC_Cancel() { ESC_pressed = 0; }

void ESC_service() {
	if ((millis() - ESC_pressed) > 900000) {
		ESC_pressed = 0;
	}
}

bool ESC_Active() {
	if (ESC_pressed) {
		return true;
	} else {
		return false;
	}
}

uint32_t ESC_ActiveTimestamp() { return ESC_pressed; }

int CLI_Process_String_App(Stream *pOpStream, String pInput,String pPrompt)
{	
	return -1;
}

int CLI_Process_String(Stream *pOpStream, String pInput) {
	String Cmd;
	int processed = 0;
	String Prompt = ">";
	
	int app_res = CLI_Process_String_App(pOpStream,pInput,Prompt);
	if (app_res==0)
	{
		processed = 1;
		pOpStream->println("OK");
	}	
	
	if (pInput.startsWith("RESET")) {
		processed = 1;
		pOpStream->println("OK");
		ESP.restart();
	}

#ifdef APP_SUPPORT_WIFI_PROV
	if (pInput.startsWith("NET_RESET")) {
		processed = 1;
		wifi_prov_network_reset();
		pOpStream->println("OK");
	}
#endif

	if (pInput.startsWith("ESC?")) {
		processed = 1;
		pOpStream->printf("ESC_pressed=%ld\r\n", ESC_ActiveTimestamp());
		pOpStream->println("OK");
	}

	if (pInput.startsWith("ESC=1")) {
		processed = 1;
		CLI_ESC_Req();
		pOpStream->println("OK");
	}

	if (pInput.startsWith("ESC=0")) {
		processed = 1;
		CLI_ESC_Cancel();
		pOpStream->println("OK");
	}

#ifdef APP_SUPPORT_I2C
	if (pInput.startsWith("I2C_SCAN")) {
		processed = 1;
		Scan_i2c(pOpStream);
		pOpStream->println("OK");
	}
#endif

	if (pInput.startsWith("WDT_TEST")) {
		processed = 1;
		WDT_Feed(4, false); // one last feed before a disable!
		WDT_feed_disable = true;
		pOpStream->println("OK");
	}

	if (pInput.startsWith("VERSION")) {
		processed = 1;
		Version_Info(pOpStream);
		pOpStream->println("OK");
	}

	if (pInput.startsWith("MACS?")) {
		processed = 1;
		print_chip_MACS();
		pOpStream->println("OK");
	}

	if (CLI_Scan_UINT32_Setting(pOpStream, pInput, "DBG", &DebugFlags.all, 0,
								0xFFFFFFFF, 16, Prompt)) {
		processed = 1;
		pOpStream->println("OK");
		return 0;
	}

#ifdef APP_SUPPORT_OTA_COMMON
	if (pInput.startsWith("OTA_CHECK")) {
		ota_check_request = 1;
		pOpStream->println("OK");
	}
#endif

#ifdef APP_SUPPORT_EPOCH
	if (pInput.startsWith("EPOCH")) {
		processed = 1;
		char temp[200];
		sprintf(temp, "EPOCH=%lu", TimeEpochGet(pOpStream));
		pOpStream->println(temp);
		pOpStream->println("OK");
	}
#endif

#ifdef APP_SUPPORT_FILESYSTEM
	if (pInput.startsWith("FS_FORMAT")) {
		processed = 1;
		FileSystem.format(pOpStream);
	}

	if (pInput.startsWith("FS_INFO")) {
		processed = 1;
		FileSystem.Info(pOpStream);
	}

	if (pInput.startsWith("FILE_")) {
		processed = FileCommmand(pOpStream, &pInput);
	}
#endif

	if ((processed == 0) && (pInput.length() > 0)) {
		pOpStream->printf(">%s ?\r\n", pInput.c_str());
	}

	return processed;
}

void CLI_Service(Stream *pInStream, Stream *pOutStream) {
	static bool stringComplete = false;
	static String inputString;

	while (pInStream->available()) {
		// get the new byte:
		char inChar = (char)pInStream->read();

		if (inChar == 27) {
			if (millis() < 90000) {
				CLI_ESC_Req();
			}
		}

		// if the incoming character is a newline, set a flag
		// so the main loop can do something about it:
		if (inChar == '\r' || inChar == 0x04) {
			stringComplete = true;
			pOutStream->println(inputString);
		} else {
			// add it to the inputString:
			inputString += inChar;
		}
	}

	if (stringComplete) {
		CLI_Process_String(pOutStream, inputString);

		stringComplete = false;
		inputString = "";
	}
}

void CLI_loop() {
	CLI_Service(Stream_UART0, Stream_UART0);
	rgb_led_task();
	Service_ConfigSwitch();
	Service_BootSwitch();
}

#endif // APP_SUPPORT_CLI

#ifdef APP_SUPPORT_I2C
void Scan_i2c(Stream *pStream) {
	Wire.end();
	pinMode(GPIO_I2C_SDA, INPUT);
	pinMode(GPIO_I2C_SCL, INPUT);

	if (digitalRead(GPIO_I2C_SDA) == 0) {
		pStream->println("I2C_SCAN,SDA detected low!");
		// SDA jammed low - try and clear by pulsing the SDA clock
		int trys = 0;
		bool sda_released = 0;
		digitalWrite(GPIO_I2C_SCL, 0);

		while (trys < 32) {
			trys++;

			pinMode(GPIO_I2C_SCL, OUTPUT);
			delay(1);
			pinMode(GPIO_I2C_SCL, INPUT);
			delay(1);

			if (digitalRead(GPIO_I2C_SDA) == 1) {
				sda_released = true;
			}
		}

		if (sda_released == 0)
			pStream->println("I2C_SCAN,SDA free failed");
		else
			pStream->printf("I2C_SCAN,SDA now released,trys=%d\r\n", trys);
	}

	Wire.begin(GPIO_I2C_SDA, GPIO_I2C_SCL, APP_I2C_FREQ);

	byte error;
	uint16_t address;
	uint16_t nDevices;

	pStream->println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 256; address += 2) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		Wire.beginTransmission(address | 0x01);
		error = Wire.endTransmission();

		if (error == 0) {
			pStream->print("I2C device found at address 0x");
			if (address < 16)
				pStream->print("0");
			pStream->print(address, HEX);
			pStream->println(" !");

			nDevices++;
		} else if (error == 4) {
			pStream->print("Unknown error at address 0x");
			if (address < 16)
				pStream->print("0");
			pStream->println(address, HEX);
		}
	}
	if (nDevices == 0)
		pStream->println("No I2C devices found\n");
	else
		pStream->println("done\n");

	Wire.end();
}
#endif

void Setup_IO() 
{
#ifdef GPIO_BOOT_SWITCH
	pinMode(GPIO_BOOT_SWITCH, INPUT_PULLUP);
#endif

#ifdef GPIO_CONFIG_SWITCH
	pinMode(GPIO_CONFIG_SWITCH, INPUT_PULLUP);
#endif

#ifdef GPIO_WIFI_PROV_SWT
	pinMode(GPIO_WIFI_PROV_SWT, INPUT_PULLUP);
#endif
}

void setup(void) 
{
	usb_serial_jtag_hal_phy_set_external(NULL, false);
	
	Setup_IO();

	Stream_UART0 = &Serial;
	Serial.begin(115200);

	Serial.println();
	Serial.println();

	Version_Info(Stream_UART0);

#ifdef APP_SUPPORT_WIFI_PROV
	wifi_prov_init();
#endif

#ifdef APP_SUPPORT_SPI
	SPI.begin();
#endif	

	rgb_led_init();

#ifdef APP_WIFI_MULTI_SUPPORT	
	//wifiMulti.addAP("edimax_2.4G_31C2D2", "Development");
#endif	

	WiFi.begin();
}

void loop() 
{
	delay(1);
	CLI_loop();
	
	if(wifi_prov_Active_ms()>0)
	{
		if(wifi_prov_Active_ms()>(10 * 60 * 1000))
		{
			esp_restart();
		}
	}
}
