//***************************
// Universidad del Valle de Guatemala
// BE3015 - Electrónica Digital 2

// PROYECTO 2 
//***************************

//***************************
// LIBRERÍAS
//***************************
//sensor
#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
MAX30105 sensor;
#include <Adafruit_NeoPixel.h>

//UART
#define rx 16
#define tx 17

//Neopixel
#define PIN 6
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); 


//***************************
//DEFINICIÓN DE PINES
//***************************
//TFT
#define screenWidth 320//ancho de la pantalla en pixeles 240x320
#define screenHeight 240//alto de la pantalla en pixeles

//***************************
// VARIABLES GLOBALES
//***************************

const byte RATE_SIZE = 10; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
int beatsPerMinute;
int beatAvg;

//botones de la tiva c
int medicion = 0;

//***************************
// PROTOTIPOS DE FUNCIONES
//***************************
void valorsensor(void);//función sensor

//***************************
// CONFIGURACIÓN
//***************************
void setup() {
  //COMUNICACIÓN SERIAL
  Serial.begin(115200);
  Serial2.begin(115200);
  //NEOPIXEL 
  pixels.begin();

  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  }

  

//***************************
// LOOP PRINCIPAL
//***************************
void loop() {

  //Botones de la tiva c
  if (Serial2.available() > 0) {

    medicion = Serial2.read();
    Serial.print("medir:  ");
    Serial.println(medicion);
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show(); 

  }

  if (medicion == 1) { //si medir es 1
    valorsensor(); //función del sensor
    Serial2.write(beatAvg); //enviar el valor del promedio a la tiva C
   
  }
 if (medicion ==2){
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(210, 150, 0));
    pixels.show(); 
 }
}

//***************************
//función para el sensor
//***************************
void valorsensor() {

  long irValue = sensor.getIR(); //Reading the IR value it will permit us to know if there's a finger on the sensor or not
  //Also detecting a heartbeat

  if (checkForBeat(irValue) == true) //si se detecta un heart beat
  {
    //We sensed a beat!
    long delta = millis() - lastBeat; //measure duration between two beats
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0); //calcular bpm

    if (beatsPerMinute < 255 && beatsPerMinute > 20) //para calcular el beatAvg se almacenan 4 datos
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);

  if (irValue < 50000) { //si no se detecta ningún dedo
    Serial.println(" No finger?    "); //indicar que no hay dedo

    //imprimir datos en la pantalla
    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.println(beatAvg);
    Serial.println();
  }

}
