# Filogic
Filogic Arduino SDK download: https://drive.google.com/drive/folders/1z7HscksgsalRfHW7T49nVbI60gY-weV7

Youtube: https://www.youtube.com/watch?v=jckQQazaxCk&list=PLKzif2izCgMgxzFNsIPzpWcC789slgqvX

MediaTek Filogic 130A development kit:

All sample codes are compiled on Arduino IDE

####################

BlinkLED : Toggle two GPIO pins, connecting to LED lights. 
點亮兩顆連接在 GPIO 接腳上的 LED 燈號。

--------------------

GPIO : show IO mux of all GPIO pins to console. 
顯示所有 IO 腳位的功能與方向。

####################

ConnectWithWPA : get network time from NTP server via WiFi and set time to RTC. 
利用無線網路連線，取得網路時間，並設定時間到 Filogic 晶片上的 RTC。

--------------------

HttpClient : by using http connection, sample code will upload data to "ThingSpeak". 
透過 HTTP POST 連線，上傳數據到 ThingSpeak 網站。

####################

I2C : control LCM module and sensors by I2C interface. 
透過 I2C 介面控制 LCM 顯示模組以及其他感測器。

####################

SimplePeripheral : As BLE peripheral side. 
當做藍芽 BLE peripheral 端，電腦或手機可以連接到 Filogic 開發板上。

--------------------

ConnectPeripher : As BLE central side. 
當做藍芽 BLE central 端，主動去連接某一 BLE peripheral 設備。

####################

Display_Adafruit, SSD1306_Adafruit, TFT7735_Adafruit (based on SDK v1.1.0 or above)
基於Adafruit GFX library, 在 TFT-LCD 和 OLED 模組上顯示各類圖形

####################

PlayWavFromSD : play wave files from SD card (based on SDK v1.2.0 or above)
                also includes ASR demo.
讀取SD卡的 wav 檔案，並且播放出來。範例中，也包含語音辨識的demo

--------------------

PlayAudio : play alarm from internal RAM after pressing SW2/SW3 down.
按下SW2或SW3鈕後，播放短語音

####################

Car_Speed_Loop : self-balancing car 自動平衡車

MPU6050 : 三軸重力加速計的模組

TB6612_motor_encoder : 馬達驅動模組 以及 pulse 計數器

####################
