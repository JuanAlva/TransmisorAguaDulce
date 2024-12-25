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
#define SAMPLES_CNT 128         // Número de muestras para suavizado

// Tabla de calibración (valores reales vs valores crudos del ADC)
float voltageReal[] = {0.487, 0.546, 0.604, 0.663, 0.722, 0.78, 0.838, 0.896, 0.956, 1.013, 1.071, 1.13, 1.188, 1.246, 1.305, 1.363, 1.421};//, 1.48, 1.538, 1.596, 1.653};  // Voltajes reales
float currentReal[] = {4, 4.48, 4.96, 5.44, 5.92, 6.4, 6.88, 7.36, 7.84, 8.32, 8.8, 9.28, 9.76, 10.24, 10.72, 11.2, 11.68};//, 12.16, 12.64, 13.12, 13.6};  // Corrientes reales
int adcRaw[] = {240, 302, 340, 387, 435, 483, 530, 585, 637, 692, 742, 797, 853, 904, 956, 1016, 1080};//, 1138, 1196, 1257, 1350};        // Valores crudos del ADC
const int calibrationPoints = 17;                  // Numero de puntos en la tabla
float voltageCalib = 0.0;
float currentCalib = 0.0;

// Variable global para almacenar el último valor válido del ADC
int previous_valid_adc = adcRaw[0];  // Inicializado al valor mínimo esperado
#define ADC_UMBRAL_MIN adcRaw[0]//480//448 // Corresponde a 0% de 4-20 mA. 0 mA
#define ADC_UMBRAL_MAX adcRaw[calibrationPoints-1]//1226//1271 // Corresponde a 33% de 4-20 mA, 0.01456 mA

// Valores para mapeo de presión
#define CURRENT_MIN 4.0
#define CURRENT_MAX 20.0
#define PRESSURE_MIN 0.0       // Valor de presión mínima
#define PRESSURE_MAX 10.0      // Valor de presión máxima

const int csPin = 5;           // LoRa radio chip select
const int resetPin = 14;       // LoRa radio reset
const int irqPin = 4;          // Cambiar dependiendo de la placa

#define RETRY_LIMIT 3          // Número máximo de reintentos para enviar un paquete
#define WATCHDOG_TIMEOUT 10    // Tiempo límite en segundos para el Watchdog

// Variables del filtro de Kalman
float x_est = 0.0;   // Estimación del estado inicial
float P_est = 1.0;   // Covarianza inicial
// Incrementa Q para que el filtro reaccione más rápido.
float Q = 0.1;//0.05;//0.05;//0.1;       // Varianza del proceso
// Incrementa R para atenuar el impacto del ruido en la salida
float R = 0.3;//0.35;//0.25;//0.5;       // Varianza del ruido de medición 
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

// Mapea corriente a presión
float map_current_to_pressure(float current) {
    return PRESSURE_MIN + (PRESSURE_MAX - PRESSURE_MIN) * ((current - CURRENT_MIN) / (CURRENT_MAX - CURRENT_MIN));
}

float calibrateVoltageADC(int adcValue) {
  // Buscar el intervalo correspondiente en la tabla de calibración
  for (int i = 0; i < calibrationPoints - 1; i++) {
    if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i + 1]) {
      // Interpolación lineal para calcular el valor calibrado
      float deltaVoltage = voltageReal[i + 1] - voltageReal[i];
      float deltaADC = adcRaw[i + 1] - adcRaw[i];
      voltageCalib = voltageReal[i] + (deltaVoltage * (adcValue - adcRaw[i]) / deltaADC);

      // Actualizar el último valor válido
      previous_valid_adc = adcValue;

      // Mensaje opcional de depuración
      Serial.printf("ADC dentro de rango: %d. Voltaje calibrado: %.2f\n", adcValue, voltageCalib);

      return voltageCalib;
    }
  }

  // En caso de que no se encuentre un intervalo válido
  Serial.println("Error: No se encontró un intervalo de calibración válido.");
  return previous_valid_adc;
}

float calibrateCurrentADC(int adcValue) {
    // Buscar el intervalo correspondiente en la tabla de calibración
    for (int i = 0; i < calibrationPoints - 1; i++) {
        if (adcValue >= adcRaw[i] && adcValue <= adcRaw[i + 1]) {
            // Interpolación lineal para calcular el valor calibrado
            float deltaCurrent = currentReal[i + 1] - currentReal[i];
            float deltaADC = adcRaw[i + 1] - adcRaw[i];
            currentCalib = currentReal[i] + (deltaCurrent * (adcValue - adcRaw[i]) / deltaADC);

            // Actualizar el último valor válido
            previous_valid_adc = adcValue;
            return currentCalib;
        }
    }

    // Este punto no debería alcanzarse si la entrada está dentro del rango
    Serial.printf("ADC fuera de rango: %d. Usando último valor válido: %d\n", adcValue, previous_valid_adc);
    return previous_valid_adc;
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

    float calibrated_voltage = calibrateVoltageADC(adc_value);
    // float voltage = map_adc_to_voltage(adc_value);
    //float filtered_voltage = kalmanFilter(calibrated_voltage);  // Aplicar filtro de Kalman
    float value_current = calibrateCurrentADC(adc_value);
    float current = kalmanFilter(value_current);
    float pressure = map_current_to_pressure(current);
    float mtr = pressure * 55.6701; //0.704;   // map_current_to_mca(current) / 100;
    float m3d = mtr * 12.267 / 100; // Area de tanque de agua m2 (aprox)

    // Crear objeto JSON
    StaticJsonDocument<64> doc;
    doc["m3d"] = m3d;

    // Serializar a cadena
    char jsonBuffer[64];
    serializeJson(doc, jsonBuffer);

    // Imprimir resultados
    Serial.printf("\nADC Value: %d, Voltage: %.3f V, Current: %.3f mA, Pressure: %.3f psi, Metros Columna de Agua: %.3f cm, Metros Cubico: %.3f m3\n",
                  adc_value, calibrated_voltage, current, pressure, mtr, m3d);
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