/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *  Created on: 05 мая. 2023 г.
 *      Author: Oleg Volkov
 *
 *  YouTube: https://www.youtube.com/channel/UCzZKTNVpcMSALU57G1THoVw
 *  GitHub: https://github.com/Solderingironspb/Lessons-Stm32/blob/master/README.md
 *  Группа ВК: https://vk.com/solderingiron.stm32
 * 
 ******************************************************************************
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>

#define name_mqtt_user "kitchen_sink_lighting" //Имя устройства на MQTT сервере

/*-------------------------------Настройки Wi-fi------------------------------------------*/

uint8_t newMACAddress[] = {0x32, 0xAE, 0x11, 0x07, 0x0D, 0x66}; //Задаем MAC адрес устройства

const char *ssid = "sdfgsfg";       // Имя Wi-fi точки доступа
const char *pass = "sfgsdfg";  // Пароль от Wi-fi точки доступа

const char *mqtt_server = "192.168.0.104";  // Имя сервера MQTT
const int mqtt_port = 1883;                       // Порт для подключения к серверу MQTT
const char *mqtt_user = "sdfgsdfg";                // Логин от сервера
const char *mqtt_pass = "gsdfgsdf";            // Пароль от сервера

/*-------------------------------Настройки Wi-fi------------------------------------------*/

Ticker TIM1; //Таймер 1
#define RELAY_PIN 12
#define GREEN_LED 13
bool relay; //Реле на свет

WiFiClient wclient;
PubSubClient client(wclient, mqtt_server, mqtt_port);

/* USER CODE BEGIN PFP */
void callback(const MQTT::Publish &pub);
void uart_parsing(void);
/* USER CODE END PFP */


void Send_state_relay(void){
  client.publish("Kitchen/kitchen_sink_lighting", String(relay));
  }

// Функция получения данных от сервера
void callback(const MQTT::Publish &pub){
  String payload = pub.payload_string();

  /*------------------Парсинг приходящих писем в топики-----------------------*/
  
  //  проверяем из нужного ли нам топика пришли данные
  if (String(pub.topic()) == "Kitchen/kitchen_sink_lighting") {
    relay = payload.toInt();  //  преобразуем полученные данные в тип int(все типы данных, кроме float преобразуем в integer)
    digitalWrite(RELAY_PIN, relay);
    digitalWrite(GREEN_LED, !relay);
    TIM1.detach();
    TIM1.attach(10, Send_state_relay);
  }

  /*------------------Парсинг приходящих писем в топики-----------------------*/
}

void setup(void) {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(GREEN_LED, !LOW);
  
  Serial.begin(9600);
  delay(100);
  Serial.setTimeout(5);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  wifi_set_macaddr(STATION_IF, &newMACAddress[0]);
  WiFi.hostname("kitchen_sink_lighting");
  WiFi.begin(ssid, pass);
  TIM1.attach(10, Send_state_relay);
  
}

void loop(void){
  
  if (WiFi.status() != WL_CONNECTED)  //Проверяем статус подключения к Wi-fi точке
  {
    Serial.println("Wifi != OK");
        
    WiFi.begin(ssid, pass); //Вводим имя точки доступа и пароль

    if (WiFi.waitForConnectResult() != WL_CONNECTED)
      return;
    Serial.println("Wifi = OK");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("Name:");
    Serial.println(WiFi.hostname());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
      }

  if (WiFi.status() == WL_CONNECTED)  // Если подключились к Wi-fi точке
  {
    if (!client.connected())  //Проверяем статус подключения к MQTT серверу
    {
      Serial.println("MQTT = !OK");
      if (client.connect(MQTT::Connect(name_mqtt_user).set_auth(mqtt_user, mqtt_pass)))  //Если подключились к MQTT серверу, то авторизуемся
      {
        Serial.println("MQTT = OK");
        client.set_callback(callback);

        /*--------------------Указываем топики, на которые хотим подписаться-------------------------*/
        client.subscribe("Kitchen/kitchen_sink_lighting");
        /*--------------------Указываем топики, на которые хотим подписаться-------------------------*/
      } else {
        Serial.println("MQTT = !OK");
        
      }
    }

    if (client.connected()) {
      client.loop();
    }
  }
}
/************************** (C) COPYRIGHT Soldering iron *******END OF FILE****/
