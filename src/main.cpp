#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_system.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define SAMPLES_CNT 32

#define SS      5   // Pin NSS (CS) del modulo LoRa
#define RST     14  // Pin RESET del modulo LoRa
#define DIO0    4   // Pin DIO0 del modulo LoRa
#define BAND    433E6 // Frecuencia LoRa

//float voltageReal[] = {4.15, 8, 12, 12.5};
float normalRaw[] = {0, 0.5, 1};
//int adcRaw[] = {10535, 20244, 30307, 31566};
int adcRaw[] = {10158, 30307, 50456};
const int calibrationPoints = 3;
float normalCalib = 0.0;

#define ADC_UMBRAL_MIN adcRaw[0]
#define ADC_UMBRAL_MAX adcRaw[calibrationPoints-1]

#define RETRY_LIMIT 3
#define WATCHDOG_TIMEOUT 10

Adafruit_ADS1115 ads; // Crear objeto ADS1115

void disableWiFiAndBluetooth() {
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect(true);
    btStop();
}

float calibrater(int adcValue) {
    if (adcValue < adcRaw[0]) return normalRaw[0];
    if (adcValue > adcRaw[calibrationPoints - 1]) return normalRaw[calibrationPoints - 1];
    
    for (int i = 0; i < calibrationPoints - 1; i++) {
        if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i + 1]) {
            float deltaNormal = normalRaw[i + 1] - normalRaw[i];
            float deltaADC = adcRaw[i + 1] - adcRaw[i];
            return normalRaw[i] + (deltaNormal * (adcValue - adcRaw[i]) / deltaADC);
        }
    }
    return normalCalib;
}

float normalizeCurrent(float current_mA) {
    return (current_mA - 4.0) / (20.0 - 4.0);
}

void initLoRa() {
    Serial.println("Inicializando módulo LoRa...");
    
    // Configura pines
    LoRa.setPins(SS, RST, DIO0);
    
    // Inicializa LoRa    
    if (!LoRa.begin(BAND)) {
        Serial.println("Error al inicializar LoRa. Reiniciando...");
        esp_restart();
    }    

    // Configuracion avanzada (opcional)
    LoRa.setSyncWord(0xF3);
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN); // Potencia de transmisión
    LoRa.setSpreadingFactor(7);  // Spreading Factor
    LoRa.setSignalBandwidth(125E3); // Ancho de banda (125 kHz)
    LoRa.setCodingRate4(5);  // Coding Rate 4/5
    Serial.println("LoRa Inicializado Correctamente!");
    //delay(100);
}

void setup() {
    Serial.begin(115200);
    // Inicializar el watchodg con tiempo limite de 10 segundos
    esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
    esp_task_wdt_add(NULL); // Se aplica al loop principal
    disableWiFiAndBluetooth();
    initLoRa();
    
    Wire.begin();  // Iniciar I2C en ESP32 (SDA=21, SCL=22 por defecto)

    if (!ads.begin()) {
        Serial.println("¡Error! No se encontró el ADS1115 en la dirección 0x48.");
        while (1);
    }
    
    Serial.println("ADS1115 detectado en la dirección 0x48.");
    ads.setGain(GAIN_TWO);  // Configurar ganancia ±4.096V (1 bit = 0.125mV)
}

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 1000;
unsigned long lastLoRaResetTime = 0; // Variable para rastrear el tiempo del último reinicio de LoRa

void loop() {
    int retryCount = 0;
    int16_t rawValue = ads.readADC_SingleEnded(0);
    
    if (rawValue == 0x8000) { // Detecta error en la lectura
        Serial.println("Error en la lectura del ADC.");
        return;
    }

    if (millis() - lastSampleTime >= sampleInterval) {
        lastSampleTime = millis();
        rawValue = ads.readADC_SingleEnded(0);
        if (rawValue < ADC_UMBRAL_MIN || rawValue > ADC_UMBRAL_MAX) {
            Serial.println("Medición fuera de rango. Descartando datos.");
            return;
        }
        rawValue = ads.readADC_SingleEnded(0);
        float normalizedValue = calibrater(rawValue);  // Calibracion del Voltaje del ADC
        float mtr = normalizedValue * 570.4304;//669.754;//360.54 - 220.7; // Conversion a Metro columna de agua
        float m3d = mtr * 0.1227 + 0.0298; // Conversion a Metros cubicos
        
        // Formateo en JSON
        StaticJsonDocument<128> doc;
        doc["m3d"] = m3d;
        doc["mtr"] = mtr;
        doc["nor"] = normalizedValue;
        doc["adc"] = rawValue;
        char jsonBuffer[128];
        serializeJson(doc, jsonBuffer);

        // Imprimir datos en serial
        Serial.printf("\nADC: %d, Normal: %.3f, Metros Columna de Agua: %.3f cm, Metros Cubico: %.3f m3\n", rawValue, normalizedValue, mtr, m3d);
        Serial.println("Enviando Paquete...");
        
        LoRa.idle();
        LoRa.beginPacket();
        LoRa.print(jsonBuffer);
        
        retryCount = 0;
        while (retryCount < RETRY_LIMIT) {
            if (LoRa.endPacket() == 0) {
                Serial.println("Error en el envío. Reintentando...");
                delay(500);
                retryCount++;
            } else {
                Serial.println("Paquete enviado correctamente");
                break;
            }
        }

        if (retryCount == RETRY_LIMIT) {
            Serial.println("Error persistente. Reiniciando módulo LoRa...");
            LoRa.end();
            delay(1000);
            initLoRa();
            lastLoRaResetTime = millis();  // Actualiza el tiempo del último reinicio
        }
        
        // **Reinicio automático cada 10 segundos**
        if (millis() - lastLoRaResetTime >= 600000) {
            Serial.println("Reiniciando módulo LoRa por mantenimiento preventivo...");
            LoRa.end();
            delay(1000);
            initLoRa();
            delay(500); // Espera para estabilizar
            ads.readADC_SingleEnded(0); // Lectura en vacío para estabilizar
            lastLoRaResetTime = millis();  // Actualiza el tiempo del último reinicio
        }
        
        // Alimentar el watchdog
        esp_task_wdt_reset();
    }
}