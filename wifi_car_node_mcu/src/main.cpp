
/**
 * Created by K. Suwatchai (Mobizt)
 *
 * Email: k_suwatchai@hotmail.com
 *
 * Github: https://github.com/mobizt/Firebase-ESP-Client
 *
 * Copyright (c) 2023 mobizt
 *
 */

/** This example will show how to authenticate using
 * the legacy token or database secret with the new APIs (using config and auth data).
 */
#include <Arduino.h>

#include <SoftwareSerial.h>
//Pinos de comunicacao serial com a ST Núcleo
#define Pin_ST_NUCLEO_RX    5  //Pino D1 da placa Node MCU
#define Pin_ST_NUCLEO_TX    4  //Pino D2 da placa Node MCU

SoftwareSerial SSerial(Pin_ST_NUCLEO_RX, Pin_ST_NUCLEO_TX);

#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

#include <Firebase_ESP_Client.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Lucas"
#define WIFI_PASSWORD "12345679"

/* 2. If work with RTDB, define the RTDB URL and database secret */
#define DATABASE_URL "..." //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app
#define DATABASE_SECRET "..."

/* 3. Define the Firebase Data object */
FirebaseData fbdo;

/* 4, Define the FirebaseAuth data for authentication data */
FirebaseAuth auth;

/* Define the FirebaseConfig data for config data */
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;
char value = 0;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  SSerial.begin(115200);

 // Serial.println("Serial by hardware!");

  // set the data rate for the SoftwareSerial port


    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        //Serial.print(".");
        delay(300);
    }
    //Serial.println();
   // Serial.print("Connected with IP: ");
    //Serial.println(WiFi.localIP());
    //Serial.println();

    //Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the certificate file (optional) */
    // config.cert.file = "/cert.cer";
    // config.cert.file_storage = StorageType::FLASH;

    /* Assign the database URL and database secret(required) */
    config.database_url = DATABASE_URL;
    config.signer.tokens.legacy_token = DATABASE_SECRET;

    // The WiFi credentials are required for Pico W
    // due to it does not have reconnect feature.

    Firebase.reconnectWiFi(true);

    /* Initialize the library with the Firebase authen and config */
    Firebase.begin(&config, &auth);

    // Or use legacy authenticate method
    // Firebase.begin(DATABASE_URL, DATABASE_SECRET);
}

void loop()
{


  //IMPORTANTE: O codigo abaixo é apenas para demonstração. 
  // Este codigo precisará ser removidou ou modificado para o projeto final!
  char move = 'X';

  if (Firebase.RTDB.getString(&fbdo, "/movement")) {
    if (fbdo.dataType() == "string" && fbdo.stringData().length() >= 1) {
      move = fbdo.stringData().charAt(0);
     // Serial.printf("Character value: %c\n", move);
    }
  }
  SSerial.write(move);

  

   //IMPORTANTE: O codigo abaixo é apenas para demonstração. 
  // Este codigo precisará ser removidou ou modificado para o projeto final!

  if (SSerial.available()){

    String received = SSerial.readString();
    //Serial.print("Received distance: ");
    //Serial.println(received);

    int distance = received.charAt(0);

    if (Firebase.RTDB.setInt(&fbdo, "/distance", distance)) {
      //Serial.println("Distance value sent to Firebase.");
    } else {
      //Serial.print("Error sending distance value to Firebase: ");
      //Serial.println(fbdo.errorReason());
    }
  }
  
  // if (Firebase.RTDB.getString(&fbdo, "/movement")) {
  //   if (fbdo.dataType() == "string" && fbdo.stringData().length() == 1) {
  //     value = fbdo.stringData().charAt(0);
  //     Serial.printf("Character value: %c\n", value);
  //     SSerial.write(value);
  //   }
  // }

  delay(200);
  // delay(50);

}