#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <DHT.h>
#include <PID_v1.h>

#define TDS_PIN A0
#define pH_PIN A1
#define CO2_PIN A2
#define DHT_PIN 4

Adafruit_ADS1015 ads;

DHT dht(DHT_PIN, DHT22);

double setpoint_TDS = 1000.0; // Valor de TDS desejado em ppm
double setpoint_pH = 7.0;    // Valor de pH desejado
double setpoint_CO2 = 1000.0; // Nível de CO2 desejado em ppm
double setpoint_Temperature = 25.0; // Temperatura desejada em Celsius

double input_TDS, output_TDS;
double input_pH, output_pH;
double input_CO2, output_CO2;
double input_Temperature, output_Temperature;

double Kp = 5; // Ganho proporcional
double Ki = 0.1; // Ganho integral
double Kd = 1; // Ganho derivativo

double elapsedTime, previousTime;
double error_TDS, lastError_TDS;
double cumError_TDS, rateError_TDS;

double error_pH, lastError_pH;
double cumError_pH, rateError_pH;

double error_CO2, lastError_CO2;
double cumError_CO2, rateError_CO2;

double error_Temperature, lastError_Temperature;
double cumError_Temperature, rateError_Temperature;

double outMin = 0;
double outMax = 255;

const int CONTROL_PIN_TDS = 5;  // Pino de controle para TDS (por exemplo, bomba peristáltica)
const int CONTROL_PIN_pH = 6;   // Pino de controle para pH (por exemplo, sistema de dosagem de solução tampão)
const int CONTROL_PIN_CO2 = 9;  // Pino de controle para CO2 (por exemplo, válvula de controle de CO2)
const int CONTROL_PIN_Heater = 10; // Pino de controle para aquecedor

void setup() {
  Serial.begin(9600);
  Wire.begin();
  ads.begin();
  dht.begin();

  pinMode(CONTROL_PIN_TDS, OUTPUT);
  pinMode(CONTROL_PIN_pH, OUTPUT);
  pinMode(CONTROL_PIN_CO2, OUTPUT);
  pinMode(CONTROL_PIN_Heater, OUTPUT);
  
  digitalWrite(CONTROL_PIN_TDS, LOW);
  digitalWrite(CONTROL_PIN_pH, LOW);
  digitalWrite(CONTROL_PIN_CO2, LOW);
  digitalWrite(CONTROL_PIN_Heater, LOW);
  
  previousTime = millis();
}

void loop() {
  double current_TDS = readTDS();
  double current_pH = readpH();
  double current_CO2 = readCO2();
  double current_Temperature = readTemperature();

  PID_Controller_TDS(current_TDS);
  PID_Controller_pH(current_pH);
  PID_Controller_CO2(current_CO2);
  PID_Controller_Temperature(current_Temperature);

  Serial.print("TDS: ");
  Serial.print(current_TDS);
  Serial.print(" ppm | pH: ");
  Serial.print(current_pH);
  Serial.print(" | CO2: ");
  Serial.print(current_CO2);
  Serial.print(" ppm | Temperature: ");
  Serial.print(current_Temperature);
  Serial.println(" °C");

  delay(1000); // Aguarda um segundo antes de realizar a próxima leitura
}

double readTDS() {
  int rawValue = ads.readADC_SingleEnded(TDS_PIN);
  double TDSValue = map(rawValue, 0, 1023, 0, 5000); // Mapeia a leitura para o intervalo de TDS (0 a 5000 ppm)
  return TDSValue;
}

double readpH() {
  // Lógica para ler o valor de pH do módulo de pH
  // Implemente esta parte de acordo com a documentação do seu módulo específico
  // e o método de calibração que você está utilizando
  return 6.8 + random(-10, 10) / 10.0;
}

double readCO2() {
  co2Serial.write(0xFF);
  co2Serial.write(0x01);
  co2Serial.write(0x86);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);
  co2Serial.write(0x00);

  delay(10);

  while (co2Serial.available() > 0) {
    if (co2Serial.read() == 0xFF) {
      if (co2Serial.read() == 0x86) {
        int co2High = co2Serial.read();
        int co2Low = co2Serial.read();
        return (co2High << 8) + co2Low;
      }
    }
  }

  return -1; // Retorno de erro se não conseguir ler corretamente
}

double readTemperature() {
  return dht.readTemperature();
}

void PID_Controller_TDS(double current_TDS) {
  error_TDS = setpoint_TDS - current_TDS;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_TDS;
  cumError_TDS += error_TDS * elapsedTime;
  double I = Ki * cumError_TDS;
  rateError_TDS = (error_TDS - lastError_TDS) / elapsedTime;
  double D = Kd * rateError_TDS;

  output_TDS = P + I + D;
  output_TDS = constrain(output_TDS, outMin, outMax);

  digitalWrite(CONTROL_PIN_TDS, output_TDS > 50);
  lastError_TDS = error_TDS;
  previousTime = currentTime;
}

void PID_Controller_pH(double current_pH) {
  error_pH = setpoint_pH - current_pH;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_pH;
  cumError_pH += error_pH * elapsedTime;
  double I = Ki * cumError_pH;
  rateError_pH = (error_pH - lastError_pH) / elapsedTime;
  double D = Kd * rateError_pH;

  output_pH = P + I + D;
  output_pH = constrain(output_pH, outMin, outMax);

  digitalWrite(CONTROL_PIN_pH, output_pH > 50);
  lastError_pH = error_pH;
  previousTime = currentTime;
}

void PID_Controller_CO2(double current_CO2) {
  error_CO2 = setpoint_CO2 - current_CO2;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_CO2;
  cumError_CO2 += error_CO2 * elapsedTime;
  double I = Ki * cumError_CO2;
  rateError_CO2 = (error_CO2 - lastError_CO2) / elapsedTime;
  double D = Kd * rateError_CO2;

  output_CO2 = P + I + D;
  output_CO2 = constrain(output_CO2, outMin, outMax);

  digitalWrite(CONTROL_PIN_CO2, output_CO2 > 50);
  lastError_CO2 = error_CO2;
  previousTime = currentTime;
}

void PID_Controller_Temperature(double current_Temperature) {
  error_Temperature = setpoint_Temperature - current_Temperature;
  double currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;

  double P = Kp * error_Temperature;
  cumError_Temperature += error_Temperature * elapsedTime;
  double I = Ki * cumError_Temperature;
  rateError_Temperature = (error_Temperature - lastError_Temperature) / elapsedTime;
  double D = Kd * rateError_Temperature;

  output_Temperature = P + I + D;
  output_Temperature = constrain(output_Temperature, outMin, outMax);

  digitalWrite(CONTROL_PIN_Heater, output_Temperature > 50);
  lastError_Temperature = error_Temperature;
  previousTime = currentTime;
}
