#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_system.h>

#define ADC_CHANNEL 32
#define SAMPLES_CNT 8192

#define SS      5   // Pin NSS (CS) del modulo LoRa
#define RST     14  // Pin RESET del modulo LoRa
#define DIO0    4   // Pin DIO0 del modulo LoRa
#define BAND    433E6 // Frecuencia LoRa

float voltageReal[] = {0.7245, 0.8036, 0.959, 1.2557, 1.2896};//{0.487, 0.546, 0.604, 0.663, 0.722, 0.78, 0.864, 0.896, 0.956, 1.013, 1.071, 1.128, 1.188, 1.246, 1.305};
int adcRaw[] = {295, 362, 470, 745, 774};//{260, 312, 350, 397, 445, 493, 526, 595, 647, 702, 752, 781, 863, 914, 966};
const int calibrationPoints = 5;// 15;
float voltageCalib = 0.0;

#define ADC_UMBRAL_MIN adcRaw[0]
#define ADC_UMBRAL_MAX adcRaw[calibrationPoints-1]

#define RETRY_LIMIT 3
#define WATCHDOG_TIMEOUT 30

float x_est = 0.0;
float P_est = 1.0;
float Q = 0.1;
float R = 0.3;
float K = 0.0;

void disableWiFiAndBluetooth() {
    WiFi.mode(WIFI_OFF);
    WiFi.disconnect(true);
    btStop();
}

struct KalmanOutput {
    float x_est;
    float P_est;
    float K;
};

KalmanOutput kalmanFilter(float z_measured) {
    // Predicción
    float x_pred = x_est;
    float P_pred = P_est + Q;

    // Ganancia de Kalman
    float K = P_pred / (P_pred + R);

    // Corrección
    x_est = x_pred + K * (z_measured - x_pred);
    P_est = (1 - K) * P_pred;

    // Retornar los valores
    return {x_est, P_est, K};
}

int read_adc(int channel) {
    int adc_value = 0;
    for (int i = 0; i < SAMPLES_CNT; i++) {
        adc_value += analogRead(channel);
    }
    return adc_value / SAMPLES_CNT;
}

float calibrateVoltageADC(int adcValue) {
    if (adcValue < adcRaw[0]) return voltageReal[0];
    if (adcValue > adcRaw[calibrationPoints - 1]) return voltageReal[calibrationPoints - 1];
    
    for (int i = 0; i < calibrationPoints - 1; i++) {
        if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i + 1]) {
            float deltaVoltage = voltageReal[i + 1] - voltageReal[i];
            float deltaADC = adcRaw[i + 1] - adcRaw[i];
            return voltageReal[i] + (deltaVoltage * (adcValue - adcRaw[i]) / deltaADC);
        }
    }
    return voltageCalib;
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
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    // Inicializar el watchodg con tiempo limite de 10 segundos
    esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
    esp_task_wdt_add(NULL); // Se aplica al loop principal
    disableWiFiAndBluetooth();
    initLoRa();
}

unsigned long lastSampleTime = 0;
const unsigned long sampleInterval = 1000;

void loop() {
    if (millis() - lastSampleTime >= sampleInterval) {
        lastSampleTime = millis();
        int adc_value = read_adc(ADC_CHANNEL); // Lectura del ADc
        if (adc_value < ADC_UMBRAL_MIN || adc_value > ADC_UMBRAL_MAX) {
            Serial.println("Medición fuera de rango. Descartando datos.");
            return;
        }
        float calibrated_voltage = calibrateVoltageADC(adc_value);  // Calibracion del Voltaje del ADc
        KalmanOutput result = kalmanFilter(calibrated_voltage); // Aplicacion del Filtro de Kalman
        float mtr = calibrated_voltage * 360.54 - 220.7; // Conversion a Metro columna de agua
        float m3d = mtr * 0.1227 + 0.0298; // Conversion a Metros cubicos
        
        // Formateo en JSON
        StaticJsonDocument<64> doc;
        doc["m3d"] = m3d;
        char jsonBuffer[64];
        serializeJson(doc, jsonBuffer);

        // Imprimir datos en serial
        Serial.printf("\nADC Value: %d, Voltage: %.3f V, kalman: %.3f V, Metros Columna de Agua: %.3f cm, Metros Cubico: %.3f m3\n", adc_value, calibrated_voltage, result.x_est, mtr, m3d);
        Serial.println("Enviando Paquete...");
        Serial.printf("Kalman Output -> x_est: %.3f V, P_est: %.5f, K: %.5f\n", 
            result.x_est, result.P_est, result.K);
        
        LoRa.idle();
        LoRa.beginPacket();
        LoRa.print(jsonBuffer);
        if (LoRa.endPacket() == 0) {
            Serial.println("Error: No se pudo enviar el paquete. Reiniciando módulo LoRa...");
            LoRa.end();
            delay(1000);
            esp_task_wdt_delete(NULL); // Pausar watchdog temporalmente
            initLoRa();
            esp_task_wdt_add(NULL); // Reactivar watchdog
        } else {
            Serial.println("Paquete enviado correctamente");
        }
        
        // Alimentar el watchdog
        //esp_task_wdt_reset();
    }
}