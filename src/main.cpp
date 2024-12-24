#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <esp_bt.h>
#include <esp_system.h>  // Incluir esta librería para utilizar esp_restart()

#define ADC_CHANNEL 32         // GPIO36 (canal ADC1_CH0 en el ESP32)
#define SLEEP_DURATION_US 5000000  // 2.5 segundos en microsegundos
#define SAMPLES_CNT 64         // Número de muestras para suavizado

// Tabla de calibración (valores reales vs valores crudos del ADC)
float voltageReal[] = {0.487, 0.546, 0.604, 0.663, 0.722, 0.78, 0.838, 0.896, 0.956, 1.013, 1.071, 1.13, 1.188, 1.246, 1.305, 1.363, 1.421, 1.48, 1.538, 1.596, 1.653};  // Voltajes reales
float currentReal[] = {4, 4.48, 4.96, 5.44, 5.92, 6.4, 6.88, 7.36, 7.84, 8.32, 8.8, 9.28, 9.76, 10.24, 10.72, 11.2, 11.68, 12.16, 12.64, 13.12, 13.6};  // Corrientes reales
int adcRaw[] = {240, 302, 340, 387, 435, 483, 530, 585, 637, 692, 742, 797, 853, 904, 956, 1016, 1080, 1138, 1196, 1257, 1350};        // Valores crudos del ADC
const int calibrationPoints = 21;                  // Numero de puntos en la tabla
float voltageCalib = 0.0;
float currentCalib = 0.0;

// Valores para el mapeo
#define ADC_MIN 480//410//549//480//448            // Valor ADC para 0.66 V
#define ADC_MAX 850//1152//810//713//800//808            //780            // Valor ADC para 2.45 V  .
#define VOLTAGE_MIN 0.787//0.49866//0.864//0.787//0.487      // Voltaje mínimo esperado
#define VOLTAGE_MAX 1.205//2.44890//1.1550//1.051//1.151//1.160      //1.101      // Voltaje máximo esperado

// Variable global para almacenar el último valor válido del ADC
int previous_valid_adc = adcRaw[0];  // Inicializado al valor mínimo esperado
#define ADC_UMBRAL_MIN adcRaw[0]//480//448 // Corresponde a 0% de 4-20 mA. 0 mA
#define ADC_UMBRAL_MAX adcRaw[21]//1226//1271 // Corresponde a 33% de 4-20 mA, 0.01456 mA


// Valores para mapeo de presión
#define CURRENT_MIN 4.0
#define CURRENT_MAX 20.0
#define PRESSURE_MIN 0.0       // Valor de presión mínima
#define PRESSURE_MAX 10.0      // Valor de presión máxima

// Valores para mapeo de metros columna de agua
#define CURRENT_MIN_MCA 7.211//6.4455
#define CURRENT_MAX_MCA 9.4267
#define MCA_MIN 92.65//65.6235
#define MCA_MAX 195.05

// Valores para mapeo de metros cubicos
#define CURRENT_MIN_M3 7.211//6.4455
#define CURRENT_MAX_M3 9.4267
#define M3_MIN 11.365// 8.05
#define M3_MAX 23.9267835

const int csPin = 5;           // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 4;          // Cambiar dependiendo de la placa

#define RETRY_LIMIT 3          // Número máximo de reintentos para enviar un paquete
#define WATCHDOG_TIMEOUT 10    // Tiempo límite en segundos para el Watchdog

// Variables del filtro de Kalman
float x_est = 0.0;   // Estimación del estado inicial
float P_est = 1.0;   // Covarianza inicial
// Incrementa Q para que el filtro reaccione más rápido.
float Q = 0.05;//0.05;//0.1;       // Varianza del proceso
// Incrementa R para atenuar el impacto del ruido en la salida
float R = 0.35;//0.25;//0.5;       // Varianza del ruido de medición 
float K = 0.0;       // Ganancia de Kalman

void disableWiFiAndBluetooth() {
    WiFi.mode(WIFI_OFF);   // Apaga Wi-Fi
    WiFi.disconnect(true); // Desconecta Wi-Fi
    btStop();              // Apaga Bluetooth
}

// Función del filtro de Kalman
float kalmanFilter(float z_measured) {
    // Predicción
    float x_pred = x_est;
    float P_pred = P_est + Q;

    // Corrección
    K = P_pred / (P_pred + R);
    x_est = x_pred + K * (z_measured - x_pred);
    P_est = (1 - K) * P_pred;

    return x_est;
}

// Lee el ADC y promedia las lecturas
int read_adc(int channel) {
    int adc_value = 0;
    for (int i = 0; i < SAMPLES_CNT; i++) {
        adc_value += analogRead(channel);
    }
    return adc_value / SAMPLES_CNT;
}

// Mapea el valor ADC a voltaje en voltios
float map_adc_to_voltage(int adc_value) {
    return VOLTAGE_MIN + (VOLTAGE_MAX - VOLTAGE_MIN) * ((float)(adc_value - ADC_MIN) / (ADC_MAX - ADC_MIN));
}

// Mapea corriente a presión
float map_current_to_pressure(float current) {
    return PRESSURE_MIN + (PRESSURE_MAX - PRESSURE_MIN) * ((current - CURRENT_MIN) / (CURRENT_MAX - CURRENT_MIN));
}

// Mapea corriente a metro columna de agua
float map_current_to_mca(float current) {
    return MCA_MIN + (MCA_MAX - MCA_MIN) * ((current - CURRENT_MIN_MCA) / (CURRENT_MAX_MCA - CURRENT_MIN_MCA));
}

// Mapea corriente metro cubico
float map_current_to_m3d(float current) {
    return M3_MIN + (M3_MAX - M3_MIN) * ((current - CURRENT_MIN_M3) / (CURRENT_MAX_M3 - CURRENT_MIN_M3));
}

float calibrateADC(int adcValue) {
  if (adcValue < ADC_UMBRAL_MIN || adcValue > ADC_UMBRAL_MAX) {
    Serial.printf("ADC fuera de rango: %d. Usando último valor válido: %d\n", adcValue, previous_valid_adc);
    adcValue = previous_valid_adc;
    return adcValue;
  }

  else {
    for (int i = 0; i < calibrationPoints - 1; i++) {
      if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i+1]) {
        voltageCalib = voltageReal[i] + (voltageReal[i+1] - voltageReal[i]) * (adcValue - adcRaw[i]) / (adcRaw[i+1] - adcRaw[i]);
        previous_valid_adc = adcValue;
        return voltageCalib;
      }
    }
  }
}


float calibrateCurrentADC(int adcValue) {
    for (int i = 0; i < calibrationPoints - 1; i++) {
      if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i+1]) {
        currentCalib = currentReal[i] + (currentReal[i+1] -currentReal[i]) * (adcValue - adcRaw[i]) / (adcRaw[i+1] - adcRaw[i]);
        previous_valid_adc = adcValue;
        return currentCalib;
      }
    }
}

// Inicializa el módulo LoRa
void initLoRa() {
    Serial.println("Inicializando módulo LoRa...");
    LoRa.setPins(csPin, resetPin, irqPin);
    if (!LoRa.begin(433E6)) {
        Serial.println("Error al Iniciar LoRa!");
        esp_restart(); // Reiniciar el ESP32 si falla LoRa
    }
    LoRa.setSyncWord(0xF3);
    Serial.println("LoRa Inicializado Correctamente!");
}

void enterLowPowerMode() {
    LoRa.sleep();  // Módulo LoRa en modo de bajo consumo
    Serial.println("Módulo LoRa en modo de bajo consumo...");
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);  // Temporizador para despertar
    esp_light_sleep_start();  // Entra en Light Sleep
    Serial.println("Despertando...");
}

void setup() {
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db); //ADC_6db

    esp_task_wdt_init(WATCHDOG_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    disableWiFiAndBluetooth();

    while (!Serial);
    initLoRa();
}

void loop() {
    esp_task_wdt_reset();

    // Leer y procesar el valor ADC
    int adc_value = read_adc(ADC_CHANNEL);

    float calibrated_voltage = calibrateADC(adc_value);
    // float voltage = map_adc_to_voltage(adc_value);
    //float filtered_voltage = kalmanFilter(calibrated_voltage);  // Aplicar filtro de Kalman
    float value_current = calibrateCurrentADC(adc_value);
    float current = kalmanFilter(value_current);
    float pressure = map_current_to_pressure(current);
    float mca = pressure * 55.8111; //0.704;   // map_current_to_mca(current) / 100;
    float m3 = mca * 12.267 / 100; // Area de tanque de agua m2 (aprox)
    float m3d = m3;//float m3d = map_current_to_m3d(pressure);

    // Crear objeto JSON
    StaticJsonDocument<128> doc;
    doc["pressure"] = pressure;
    doc["mca"] = mca;
    doc["m3d"] = m3d;

    // Serializar a cadena
    char jsonBuffer[128];
    serializeJson(doc, jsonBuffer);

    // Imprimir resultados
    Serial.printf("\nADC Value: %d, Voltage: %.3f V, Current: %.3f mA, Pressure: %.3f psi, Metros Columna de Agua: %.3f cm, Metros Cubico: %.3f m3\n",
                  adc_value, calibrated_voltage, current, pressure, mca, m3);
    Serial.println("Enviando Paquete...");

    // Intentar enviar paquete con reintentos
    int retries = 0;
    bool success = false;

    while (retries < RETRY_LIMIT && !success) {
        // Enviar datos
        LoRa.idle();  // Activar módulo LoRa
        LoRa.beginPacket();
        LoRa.print(jsonBuffer);
        if (LoRa.endPacket() == 0) {
            retries++;
            Serial.printf("Error enviando paquete, intento %d/%d\n", retries, RETRY_LIMIT);
            delay(1000);
        } else {
            success = true;
            Serial.println("Paquete enviado correctamente");
        }
    }

    if (!success) {
        Serial.println("Error: No se pudo enviar el paquete después de varios intentos.");
        Serial.println("Reiniciando módulo LoRa...");
        LoRa.end();
        delay(1000);
        initLoRa();
        // Reiniciar si después de varios intentos LoRa no funciona
        Serial.println("Reiniciando el sistema...");
        esp_restart();  // Reiniciar el ESP32
    }

    // Entrar en modo de bajo consumo
    enterLowPowerMode();
}