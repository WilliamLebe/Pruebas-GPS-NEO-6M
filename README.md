PRUEBAS_


CARPETAS:

•	PRUEBA GPS:
  Tiene el código de prueba GPS NEO
  Se debe tener raspberry pi pico w, cargar libreria de micropyGPS, nrf24l01, ssd1306.
  Cargar en la raspberry programa main.gps.py y cargarlo como main.py
  
•	PRUEBA GPS y TCRT5000:
  Tiene el código de prueba GPS NEO Agregando el sensor TCRT5000
  Se debe tener raspberry pi pico w, cargar librería de micropyGPS, nrf24l01, ssd1306.
  Cargar en la raspberry programa main-gps-v2.py y cargarlo como main.py
  
•	PRUEBA Jostick, Servo motor, ESC:
  Tiene el código de prueba GPS NEO Agregando el sensor TCRT5000, servo motor SG90, modulo ESC, pantalla oled para motor de tracción para el carro
  En el control se debe tener los 2 jostick, modulo nrf24l01, pantalla oled
  Se debe tener 2 raspberry pi pico w, cargar librería de nrf24l01, ssd1306.
  Cargar en la raspberry 1 programa tx-jostick.py y cargarlo como main.py
  Cargar en la raspberry 2 programa rx-jostick.py y cargarlo como main.py

•	Librerias:
  * micropyGPS para el uso del GPS.
  * nrf24l01 para el uso de la antena nrf.
  * ssd1306 para uso de la pantalla oled 128x32.
