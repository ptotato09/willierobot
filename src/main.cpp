#include <Arduino.h>
#include <Bluepad32.h>
#include <ESP32Servo.h>

// ==================== PIN INDICADOR LED ====================
const int PIN_LED = 2; // LED integrado para indicar conexión

// ==================== PINES DEL CHASIS MECANUM ====================
// Delantera Izquierda (FL)
const int FL_IN1 = 32;
const int FL_IN2 = 33;
const int FL_PWM = 25;

// Delantera Derecha (FR)
const int FR_IN1 = 26;
const int FR_IN2 = 27;
const int FR_PWM = 14;

// Trasera Izquierda (BL)
const int BL_IN1 = 16;
const int BL_IN2 = 17;
const int BL_PWM = 18;

// Trasera Derecha (BR)
const int BR_IN1 = 19;
const int BR_IN2 = 23;
const int BR_PWM = 13;

// ==================== PINES Y VARIABLES DE SERVOS ====================
const int PIN_PINZA = 4;
const int PIN_ELEVADOR = 5;

Servo servoPinza;
Servo servoElevador;

// Configuración Pinza (Toggle con botón A)
const int PINZA_ABIERTA = 5;
const int PINZA_CERRADA = 70;
bool pinzaCerrada = false;
bool botonA_Anterior = false;

// Configuración Brazo Elevador (Eje Y Joystick Derecho)
float anguloElevador = 0.0;
const int ELEVADOR_MIN = 0;     // Hasta abajo
const int ELEVADOR_MAX = 180;   // Tope superior
const float VEL_ELEVADOR = 2.5; // Grados por ciclo

// ==================== CONFIGURACIÓN BLUEPAD32 ====================
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
const int DEADZONE = 80; // Zona muerta actualizada a 80

// Callbacks de conexión del control
void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("Control conectado en índice=%d\n", i);
            myControllers[i] = ctl;
            digitalWrite(PIN_LED, HIGH); // Enciende el pin 2 al conectar
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("Control desconectado de índice=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
    
    // Verificar si queda algún control conectado; si no, apagar el LED y frenar
    bool algunoConectado = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr) {
            algunoConectado = true;
            break;
        }
    }
    if (!algunoConectado) {
        digitalWrite(PIN_LED, LOW);
    }
}

// ==================== FUNCIONES DE CONTROL DE MOTORES ====================
void moverMotor(int in1, int in2, int pwmPin, int velocidad) {
    velocidad = constrain(velocidad, -255, 255);
    if (velocidad > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, velocidad);
    } else if (velocidad < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        analogWrite(pwmPin, -velocidad);
    } else {
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        analogWrite(pwmPin, 0);
    }
}

void detenerChasis() {
    moverMotor(FL_IN1, FL_IN2, FL_PWM, 0);
    moverMotor(FR_IN1, FR_IN2, FR_PWM, 0);
    moverMotor(BL_IN1, BL_IN2, BL_PWM, 0);
    moverMotor(BR_IN1, BR_IN2, BR_PWM, 0);
}

int aplicarDeadzone(int valor) {
    if (abs(valor) < DEADZONE) return 0;
    return valor;
}

// ==================== PROCESAMIENTO DEL CONTROL ====================
void procesarGamepad(ControllerPtr ctl) {
    // 1. LECTURA DE EJES PARA EL CHASIS
    int rawY  = -aplicarDeadzone(ctl->axisY());  // Y+ Enfrente, Y- Atrás
    int rawX  =  aplicarDeadzone(ctl->axisX());  // X+ Derecha, X- Izquierda (Strafe)
    int rawRx =  aplicarDeadzone(ctl->axisRX()); // X+ Giro Horario, X- Antihorario

    // Mapeo a rango PWM (-255 a 255)
    float y  = map(rawY, -512, 512, -255, 255);
    float x  = map(rawX, -512, 512, -255, 255);
    float rx = map(rawRx, -512, 512, -255, 255);

    // Cinemática corregida según tu cableado físico actual:
    // Ahora con Strafe X+ (Derecha): FL y BR van adelante (+), FR y BL van atrás (-)
    // Ahora con Giro RX+ (Horario): Izquierdos (FL y BL) van adelante (+), Derechos (FR y BR) van atrás (-)
    float fl = y + x + rx;
    float fr = y - x + rx; // Compensado por intercambio físico con BL
    float bl = y - x - rx; // Compensado por intercambio físico con FR
    float br = y + x - rx;

    // Normalización para mantener proporción sin pasar de 255
    float maxVal = max(max(abs(fl), abs(fr)), max(abs(bl), abs(br)));
    if (maxVal > 255.0) {
        fl = (fl / maxVal) * 255.0;
        fr = (fr / maxVal) * 255.0;
        bl = (bl / maxVal) * 255.0;
        br = (br / maxVal) * 255.0;
    }

    moverMotor(FL_IN1, FL_IN2, FL_PWM, (int)fl);
    moverMotor(FR_IN1, FR_IN2, FR_PWM, (int)fr);
    moverMotor(BL_IN1, BL_IN2, BL_PWM, (int)bl);
    moverMotor(BR_IN1, BR_IN2, BR_PWM, (int)br);

    // 2. CONTROL DEL BRAZO ELEVADOR (Joystick Derecho Eje Y)
    int rawRy = -aplicarDeadzone(ctl->axisRY());
    if (rawRy != 0) {
        float delta = (rawRy / 512.0) * VEL_ELEVADOR;
        anguloElevador += delta;
        anguloElevador = constrain(anguloElevador, ELEVADOR_MIN, ELEVADOR_MAX);
        servoElevador.write((int)anguloElevador);
    }

    // 3. CONTROL DE LA PINZA (Toggle con Botón A)
    bool botonA_Actual = ctl->a();
    if (botonA_Actual && !botonA_Anterior) {
        pinzaCerrada = !pinzaCerrada;
        if (pinzaCerrada) {
            servoPinza.write(PINZA_CERRADA); // 70 grados
        } else {
            servoPinza.write(PINZA_ABIERTA); // 5 grados
        }
    }
    botonA_Anterior = botonA_Actual;
}

// ==================== SETUP Y LOOP ====================
void setup() {
    Serial.begin(115200);

    // Configurar pin 2 como salida e iniciar apagado
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    // Configurar pines de motores como salida
    int pinesMotores[] = {FL_IN1, FL_IN2, FL_PWM, FR_IN1, FR_IN2, FR_PWM, 
                          BL_IN1, BL_IN2, BL_PWM, BR_IN1, BR_IN2, BR_PWM};
    for (int pin : pinesMotores) {
        pinMode(pin, OUTPUT);
    }
    detenerChasis();

    // Asignación de timers para ESP32Servo
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servoPinza.setPeriodHertz(50);
    servoElevador.setPeriodHertz(50);

    // Conectar servos
    servoPinza.attach(PIN_PINZA, 500, 2400);
    servoElevador.attach(PIN_ELEVADOR, 500, 2400);

    // Posiciones iniciales
    servoPinza.write(PINZA_ABIERTA);
    servoElevador.write(ELEVADOR_MIN);

    // Inicializar Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        for (auto myController : myControllers) {
            if (myController && myController->isConnected() && myController->hasData()) {
                if (myController->isGamepad()) {
                    procesarGamepad(myController);
                }
            }
        }
    }
    delay(15);
}