URL: https://github.com/mozolin/LilyGo_T-Zigbee_ESP32C3  
  
# Example: DS18B20 & BME280 & BH1750 sensors with LilyGo T-Zigbee ESP32C3 (Platformio IDE)
  
Zigbe2MQTT Settings:  
[Example: Zigbee2MQTT CC2538 settings for ESP32-C3, ESP32-C6 and CC2530](https://github.com/mozolin/Zigbee2MQTT_CC2538)  
  
![](img/esp32c3-tlsr8258_zigbee.jpg)  
![](img/esp32c3-tlsr8258_zigbee.png)  

# Make firmware for ESP32-C3 LilyGo T-Zigbee 
Before firmware update, adjust the DIP switch:  
![](img/upload_mode_c3.png)
  
1) Install Platformio IDE as VSCode extension (https://platformio.org/install/ide?install=vscode)  
![](img/ESP32C3_Flash_01.png)

2) Disable ESP-IDF status bar icons  
![](img/ESP32C3_Flash_02.png)

3) Set the correct COM-port for upload and serial monitor  
![](img/ESP32C3_Flash_03.png)

4) Build and upload firmware  
![](img/ESP32C3_Flash_04.png)

5) The result might look like this  
![](img/ESP32C3_Flash_05.png)

6) Open serial monitor  
![](img/ESP32C3_Flash_06.png)

7) The result might look like this  
![](img/ESP32C3_Flash_07.png)
![](img/ssd1306_tlsr8258-esp32c3.jpg)  
![](img/ssd1306_zigbee.jpg)  
