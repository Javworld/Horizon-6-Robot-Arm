#include "SPI.h"
#include "AccelStepper.h"

// Definir un enum para nombrar los motores en lugar de usar números
enum StepperID {
    J1=0,J2=1,J3=2,J4=3,J5=4,J6=5
};

const int PIN_CLK = 13;
const int PIN_MOSI = 11;
const int PIN_MISO = 12;

uint8_t TOff = 0x05;
float SPISpeed = 8000000;

struct StepperPins {
    int stepPin;
    int enablePin;
    int dirPin;
    int csPin;
};

// Crear un array con la configuración de los motores
StepperPins steppers[] = {
    {6, 0, 35, 40}, // Motor 1
    {5, 0, 34, 39}, // Motor 2
    {4, 0, 33, 38},  // Motor 3
    {3, 0, 32, 37},  // Motor 4
    {2, 0, 31, 36},  // Motor 5
    {1, 0, 30, 10}   // Motor 6
};

// Número de steppers
const int numSteppers = sizeof(steppers) / sizeof(steppers[0]);

AccelStepper Motor[numSteppers] = {
    AccelStepper(1, steppers[0].stepPin, steppers[0].dirPin),
    AccelStepper(1, steppers[1].stepPin, steppers[1].dirPin),
    AccelStepper(1, steppers[2].stepPin, steppers[2].dirPin),
    AccelStepper(1, steppers[3].stepPin, steppers[3].dirPin),
    AccelStepper(1, steppers[4].stepPin, steppers[4].dirPin),
    AccelStepper(1, steppers[5].stepPin, steppers[5].dirPin)
};

uint8_t microstepsDict[numSteppers] = {32, 32, 32, 32, 32, 32}; // Valor por defecto: 16 microsteps

struct MotorDefaultConfig {
    uint8_t currentRange; // Corresponde a tu columna "Ibase"
    uint8_t runCurrent;   // Corresponde a tu columna "Irun"
    uint8_t holdCurrent;  // Corresponde a tu columna "Ihold"
    uint8_t microsteps;   // Corresponde a tu columna "Microsteps"
};

// Array con los valores de configuración por defecto para cada motor, según tu tabla.
const MotorDefaultConfig defaultConfigs[numSteppers] = {
    // Ibase, Irun, Ihold, Microsteps
    { 3,     30,    21,     32 }, // J1
    { 3,     30,    21,     32 }, // J2
    { 3,     30,    21,     32 }, // J3
    { 2,     30,    21,     32 }, // J4
    { 2,     30,    21,     32 }, // J5
    { 1,     23,    16,     32 }  // J6
};

float gearRatios[numSteppers] = {
    6.25,    // J1: Ejemplo, 8:1
    13.3333,   // J2: Ejemplo, 10:1
    16.67,   // J3: Ejemplo, 10:1
    6.25,    // J4: Ejemplo, 5:1
    5.0,    // J5: Ejemplo, 5:1
    10.0     // J6: Ejemplo, sin engranaje (directo)
};

const int limitSwitchPins[numSteppers] = {
    7,  // J1
    8,  // J2
    9,  // J3
    28, // J4
    29, // J5
    41  // J6 (que corresponde a A17 en un Arduino Mega)
};

const int limitSwitchActiveState[numSteppers] = {
    LOW,  // J1
    HIGH,  // J2
    HIGH,  // J3
    HIGH,  // J4
    HIGH,  // J5
    LOW   // J6
};

// DIRECCIÓN DE BÚSQUEDA para el Homing de cada eje.
// Usa -1 para mover en dirección NEGATIVA.
// Usa  1 para mover en dirección POSITIVA.
const int homingDirections[numSteppers] = {
    -1, -1, -1, -1, 1, -1
};


// Define el ángulo real de la articulación cuando el final de carrera es presionado.
const float homingOffsetsDegrees[numSteppers] = {
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

// --- PINES PARA ENCODERS (Lectura Analógica) ---
const int encoderPins[numSteppers] = {
    A7, // J1 en pin A7 (Digital 21)
    A6, // J2 en pin A6 (Digital 20)
    A5, // J3 en pin A5 (Digital 19)
    A4, // J4 en pin A4 (Digital 18)
    A3, // J5 en pin A3 (Digital 17)
    A2  // J6 en pin A2 (Digital 16)
};

// ¡IMPORTANTE! Debes calibrar estos valores para que tus encoders sean precisos.
const int ENCODER_MIN_RAW = 0;      // Valor analógico mínimo (0-1023) que lees del encoder.
const int ENCODER_MAX_RAW = 1023;   // Valor analógico máximo.
const float ENCODER_MIN_DEG = 0.0;  // A cuántos grados corresponde el valor mínimo.
const float ENCODER_MAX_DEG = 360.0;// A cuántos grados corresponde el valor máximo.


// --- PINES PARA E/S EXTERNAS ---
const int externalOutputPins[] = { A0, A1, A8, A9 }; // O1, O2, O3, O4
const int externalInputPins[] = { A10, A11, A12, A13 }; // I1, I2, I3, I4
const int numExternalOutputs = sizeof(externalOutputPins) / sizeof(externalOutputPins[0]);
const int numExternalInputs = sizeof(externalInputPins) / sizeof(externalInputPins[0]);

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando sistema del brazo robótico...");

  // --- Configuración de Drivers ---
  Serial.println("Configurando drivers de motores...");
  pinMode(steppers[0].enablePin, OUTPUT);
  digitalWrite(steppers[0].enablePin, LOW);
  for (int i = 0; i < numSteppers; i++) {
    pinMode(steppers[i].stepPin, OUTPUT);
    pinMode(steppers[i].dirPin, OUTPUT);
    pinMode(steppers[i].csPin, OUTPUT);
    digitalWrite(steppers[i].csPin, HIGH);
  }

  // --- Configuración de Finales de Carrera ---
  Serial.println("Configurando finales de carrera con PULLUP...");
  for (int i = 0; i < numSteppers; i++) {
    pinMode(limitSwitchPins[i], INPUT_PULLUP);
  }

  // --- Configuración de Encoders ---
  Serial.println("Configurando pines de encoders...");
  for (int i = 0; i < numSteppers; i++) {
    pinMode(encoderPins[i], INPUT);
  }

  // --- Configuración de E/S Externas ---
  Serial.println("Configurando E/S externas...");
  for (int i = 0; i < numExternalOutputs; i++) {
    pinMode(externalOutputPins[i], OUTPUT);
  }
  for (int i = 0; i < numExternalInputs; i++) {
    pinMode(externalInputPins[i], INPUT_PULLUP); // PULLUP es un buen default para botones
  }
  
  // --- Configuración de la comunicación SPI ---
  SPI.setMISO(PIN_MISO);
  SPI.setMOSI(PIN_MOSI);
  SPI.setSCK(PIN_CLK);
  SPI.begin();
  
  Serial.println("-----------------------------------------");
  Serial.println("Sistema listo. Esperando comandos.");
  Serial.println("-----------------------------------------");
  
}


void loop() {
    
    readSerialCommand();

    
    // Ejecutar cada motor en cada iteración del loop (sin bloqueos)
    for (int i = 0; i < numSteppers; i++) {
        Motor[i].run();
    }
}

// Función genérica de intercambio SPI
void exchangeSPI(int motorIndex, uint8_t* data, int size) { 
    int CS = steppers[motorIndex].csPin;
    if (motorIndex >= numSteppers) return; // Evita accesos fuera del array

    digitalWrite(CS, LOW);  
    SPI.beginTransaction(SPISettings(SPISpeed, MSBFIRST, SPI_MODE3)); 

    SPI.transfer(data, size); 

    SPI.endTransaction();
    digitalWrite(CS, HIGH); 
}

// Escribir en un registro del driver usando `J0`, `J1`, `J2`
void writeRegister(StepperID motor, uint8_t address, uint32_t data) {
    if (motor >= numSteppers) return;

    // Construir mensaje para escritura
    uint8_t buffer[5] = {
        (uint8_t)(address | 0x80), // Bit 7 en 1 para escritura
        (uint8_t)(data >> 24),
        (uint8_t)(data >> 16),
        (uint8_t)(data >> 8),
        (uint8_t)data
    };

    // Mostrar el mensaje enviado
    Serial.print("Escribiendo en motor ");
    Serial.print(motor);
    Serial.print(", registro 0x");
    Serial.print(address, HEX);
    Serial.print(": ");
    for (int i = 0; i < 5; i++) {
        Serial.print("0x");
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Enviar escritura
    exchangeSPI(motor, buffer, 5);

    // Leer para verificar
    uint32_t readBackData = 0;
    uint8_t status = 0;
    readRegister(motor, address, &readBackData, &status);

    // Mostrar lo leído
    Serial.print("Leído de regreso: status 0x");
    Serial.print(status, HEX);
    Serial.print(", data: 0x");
    Serial.println(readBackData, HEX);

    // Comparar
    if (readBackData == data) {
        Serial.println("✅ Verificación exitosa: los datos coinciden.");
    } else {
        Serial.println("❌ Error: los datos NO coinciden.");
    }
}
// Leer un registro del driver usando `J0`, `J1`, `J2`
void readRegister(StepperID motor,uint8_t address, uint32_t* data, uint8_t* status) {
    if (motor >= numSteppers) return;

    uint8_t buffer[5] = { address, 0, 0, 0, 0 };

    exchangeSPI(motor,buffer, 5); // Primera lectura
    exchangeSPI(motor,buffer, 5); // Segunda lectura

    *status = buffer[0];  
    *data = (buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4];
}

bool initMotor(StepperID motor, uint8_t runCurrent, uint8_t holdCurrent, uint8_t microsteps, uint8_t currentRange) {
    if (motor >= numSteppers) return false;

    bool setup = false;  
    int attempt = 0;
    const int maxAttempts = 10;  // Límite de intentos

    while (!setup && attempt < maxAttempts) {
        uint32_t ms;

        // Selección de microsteps (usando la codificación del driver)
        if (microsteps == 128) ms = 0x1;
        else if (microsteps == 64) ms = 0x2;
        else if (microsteps == 32) ms = 0x3;
        else if (microsteps == 16) ms = 0x4;
        else if (microsteps == 8) ms = 0x5;
        else if (microsteps == 4) ms = 0x6;
        else if (microsteps == 2) ms = 0x7;
        else if (microsteps == 1) ms = 0x8;
        else ms = 0x0;

        // 1️⃣ Configurar el registro DRV_CONF (0x0A) para establecer CURRENT_RANGE
        uint32_t drvConf = ((currentRange-1) & 0x03);
        writeRegister(motor, 0x0A, drvConf);

        // 2️⃣ Configurar IHOLD_IRUN (0x10) para definir las corrientes de operación
        uint32_t iholdIrun = ((uint32_t)0x0401 << 16) | ((uint32_t)(runCurrent & 0x1F) << 8) | ((uint32_t)(holdCurrent & 0x1F));
        writeRegister(motor, 0x10, iholdIrun);

        // 3️⃣ Configurar el registro de microsteps en CHOPCONF (0x6C)
        writeRegister(motor, 0x6C, 0x10410150 | (ms << 24) | TOff);

        // 4️⃣ Verificar configuración
        uint8_t status;
        uint32_t data;
        readRegister(motor, 0x6C, &data, &status);

        if ((data & 0x0000000F) == TOff) {
            Serial.print("✅ Motor ");
            Serial.print(motor + 1);
            Serial.println(" inicializado correctamente.");
            setup = true;
            microstepsDict[motor] = microsteps;
        } else {
            Serial.print("❌ Fallo en la inicialización del motor ");
            Serial.print(motor + 1);
            Serial.print(". Intento ");
            Serial.print(attempt + 1);
            Serial.print(" de ");
            Serial.println(maxAttempts);
            attempt++;
            delay(1000);
        }
    }

    if (!setup) {
        Serial.print("🚨 Error crítico: Motor ");
        Serial.print(motor + 1);
        Serial.println(" no pudo inicializarse después de 10 intentos.");
    }

    return setup;
}

void initAllMotors() {
    Serial.println("--- INICIANDO TODOS LOS MOTORES CON CONFIGURACIÓN POR DEFECTO ---");

    for (int i = 0; i < numSteppers; i++) {
        Serial.print("Inicializando J");
        Serial.print(i + 1);
        Serial.println("...");
        
        // Llama a la función initMotor con los datos del array de configuración
        initMotor(
            (StepperID)i,
            defaultConfigs[i].runCurrent,
            defaultConfigs[i].holdCurrent,
            defaultConfigs[i].microsteps,
            defaultConfigs[i].currentRange
        );
    }

    Serial.println("✅ Inicialización de todos los motores completada.");
}

void moveMotor(StepperID motor, int steps) {
    if (motor >= numSteppers) return; // Salir si el motor no es válido

    // Mensaje para indicar que el comando fue recibido y la acción iniciada
    Serial.print("Iniciando movimiento relativo del motor J");
    Serial.print(motor + 1);
    Serial.print(" por ");
    Serial.print(steps);
    Serial.println(" pasos.");

    // move() establece un objetivo de movimiento RELATIVO a la posición actual.
    // No bloquea el programa. El loop principal se encarga del resto.
    Motor[motor].move(steps); 
}

void configureMotor(StepperID motor, float maxSpeed, float acceleration) {
    if (motor >= numSteppers) return; // Verifica que el motor exista

    // Configurar la velocidad y aceleración en AccelStepper
    Motor[motor].setMaxSpeed(maxSpeed);
    Motor[motor].setAcceleration(acceleration);

    Serial.print("Configuración de J");
    Serial.print(motor + 1);
    Serial.println(" actualizada:");
    Serial.print("  Velocidad Máx: ");
    Serial.println(maxSpeed);
    Serial.print("  Aceleración: ");
    Serial.println(acceleration);
}

void moveMotorByDegrees(StepperID motor, float degrees, float rpm) {
    if (motor >= numSteppers) return; // Salir si el motor no es válido

    // Obtener los datos específicos del motor
    uint8_t microsteps = microstepsDict[motor];
    float gearRatio = gearRatios[motor]; // NUEVO: Obtenemos la relación de engranajes

    int motorStepsPerRev;
    if (motor == 0 || motor == 2) { // Asumo J1 y J3 son de 400 pasos/rev (0.9°)
        motorStepsPerRev = 400;
    } else {
        motorStepsPerRev = 200; // El resto son 200 pasos/rev (1.8°)
    }

    // --- CÁLCULOS MEJORADOS ---
    
    // 1. Calcular los grados que el MOTOR debe girar
    float motorDegrees = degrees * gearRatio;

    // 2. Convertir grados del MOTOR a pasos absolutos
    long totalSteps = (motorDegrees / 360.0) * motorStepsPerRev * microsteps;

    // 3. Calcular la velocidad del MOTOR en pasos/segundo
    float motorRPM = rpm * gearRatio;
    float stepsPerSecond = (motorRPM * motorStepsPerRev * microsteps) / 60.0;

    // --- FIN DE CÁLCULOS ---

    Serial.print("🔄 MOVIENDO J");
    Serial.print(motor + 1);
    Serial.print(" -> Articulación a ");
    Serial.print(degrees);
    Serial.print("° a ");
    Serial.print(rpm);
    Serial.println(" RPM (articulación).");

    // Configurar los parámetros de movimiento en AccelStepper
    Motor[motor].setMaxSpeed(stepsPerSecond);
    Motor[motor].setAcceleration(stepsPerSecond / 2); // Una aceleración razonable
    Motor[motor].moveTo(totalSteps); // moveTo() va a una posición ABSOLUTA
}

long degreesToSteps(StepperID motor, float degrees) {
    uint8_t microsteps = microstepsDict[motor];
    float gearRatio = gearRatios[motor];
    int motorStepsPerRev = (motor == 0 || motor == 2) ? 400 : 200;
    float motorDegrees = degrees * gearRatio;
    return (long)((motorDegrees / 360.0) * motorStepsPerRev * microsteps);
}

void homeSingleAxis(StepperID motor) {
    // --- Parámetros de Homing ---
    const float fast_homing_rpm = 150.0; // RPM del MOTOR para la búsqueda rápida
    const float slow_homing_rpm = 75.0;  // RPM del MOTOR para la aproximación lenta
    long backoff_steps = 400; // Pasos para retroceder (aumentado para asegurar que el switch se libere)

    Serial.print("Iniciando Homing de precisión para J");
    Serial.println(motor + 1);

    // --- Obtener configuraciones del motor ---
    int direction = homingDirections[motor];
    uint8_t microsteps = microstepsDict[motor];
    int motorStepsPerRev = (motor == 0 || motor == 2) ? 400 : 200;

    // --- Convertir RPM a Pasos por Segundo ---
    float fast_speed_steps_sec = (fast_homing_rpm * motorStepsPerRev * microsteps) / 60.0;
    float slow_speed_steps_sec = (slow_homing_rpm * motorStepsPerRev * microsteps) / 60.0;
    
    // =======================================================
    // ETAPA 1: BÚSQUEDA RÁPIDA
    // =======================================================
    Serial.println(" -> Etapa 1: Búsqueda rápida...");
    Motor[motor].setMaxSpeed(fast_speed_steps_sec);
    Motor[motor].setAcceleration(20000); // Aceleración alta para alcanzar la velocidad rápido

    // Moverse hacia el switch en la dirección configurada
    while (digitalRead(limitSwitchPins[motor]) != limitSwitchActiveState[motor]) {
        Motor[motor].move(1000 * direction);
        Motor[motor].run();
    }
    Motor[motor].stop(); // Parada inmediata al encontrar el switch
    
    // =======================================================
    // ETAPA 2: RETROCESO Y APROXIMACIÓN LENTA
    // =======================================================
    Serial.println(" -> Etapa 2: Retrocediendo para aproximación lenta...");

    // Moverse hacia atrás para liberar el switch
    Motor[motor].moveTo(Motor[motor].currentPosition() + (backoff_steps * -direction));
    while (Motor[motor].distanceToGo() != 0) {
        Motor[motor].run();
    }
    Motor[motor].stop();
    delay(100); // Pequeña pausa para asegurar que el estado mecánico y eléctrico se asiente

    Serial.println(" -> Reap roximando lentamente...");
    Motor[motor].setMaxSpeed(slow_speed_steps_sec); // Cambiamos a la velocidad lenta

    // Moverse de nuevo hacia el switch, esta vez lentamente
    while (digitalRead(limitSwitchPins[motor]) != limitSwitchActiveState[motor]) {
        Motor[motor].move(1000 * direction);
        Motor[motor].run();
    }
    Motor[motor].stop(); // Parada final y precisa

    // =======================================================
    // ETAPA FINAL: APLICAR OFFSET
    // =======================================================
    Serial.println(" -> Homing preciso finalizado. Aplicando offset.");
    float offset_in_degrees = homingOffsetsDegrees[motor];
    long offset_in_steps = degreesToSteps(motor, offset_in_degrees);
    Motor[motor].setCurrentPosition(offset_in_steps);

    Motor[motor].setMaxSpeed(1000); // Restablecer a una velocidad por defecto

    Serial.print("✅ Homing de J");
    Serial.print(motor + 1);
    Serial.print(" completado. Posición establecida en ");
    Serial.print(offset_in_degrees);
    Serial.println(" grados.");
}

void homeAllAxes() {
    Serial.println("--- INICIANDO SECUENCIA DE HOMING COMPLETA ---");
    // Ajusta este orden según la cinemática de tu brazo para un homing seguro
    homeSingleAxis(J6);
    homeSingleAxis(J5);
    homeSingleAxis(J4);
    homeSingleAxis(J3);
    homeSingleAxis(J2);
    homeSingleAxis(J1);
    
    Serial.println("🎉🎉🎉 HOMING DE TODOS LOS EJES COMPLETADO 🎉🎉🎉");
}


float readEncoderDegrees(StepperID motor) {
    if (motor >= numSteppers) return -1.0;
    
    int rawValue = analogRead(encoderPins[motor]);
    float degrees = 0.0;

    // Mapear el valor raw (ej. 0-1023) al rango de grados del encoder
    // Se multiplica por 100 para que la función map pueda usar decimales.
    degrees = map(rawValue, ENCODER_MIN_RAW, ENCODER_MAX_RAW, (long)(ENCODER_MIN_DEG * 100), (long)(ENCODER_MAX_DEG * 100)) / 100.0;
    
    // --- LÓGICA ESPECIAL PARA J6 ---
    // Si el motor es J6, el encoder está midiendo el ángulo del motor.
    // Debemos dividir por la relación de engranajes para obtener el ángulo de la articulación.
    if (motor == J6) {
        if (gearRatios[motor] != 0) { // Evitar división por cero
            return degrees / gearRatios[motor];
        } else {
            return degrees; // Si no hay gear ratio, son lo mismo
        }
    } 
    // Para todos los demás motores (J1-J5), el encoder mide directamente la articulación.
    else {
        return degrees;
    }
}

void printMotorStatus(StepperID motor) {
    if (motor >= numSteppers) return;

    uint8_t status;
    uint32_t tempData, driverStatus;

    // Leer el estado y la temperatura del driver desde SPI
    readRegister(motor, 0x00, &driverStatus, &status);
    readRegister(motor, 0x51, &tempData, &status);
    float temperature = parseTemperature(tempData);

    // --- Generar Reporte ---
    Serial.print("🔹 Estado de J");
    Serial.println(motor + 1);
    Serial.print("  ├─ Velocidad Máx: ");
    Serial.println(Motor[motor].maxSpeed());
    Serial.print("  ├─ Aceleración: ");
    Serial.println(Motor[motor].acceleration());
    Serial.print("  ├─ Posición Actual (pasos): ");
    Serial.println(Motor[motor].currentPosition());
    Serial.print("  ├─ Pasos Restantes: ");
    Serial.println(Motor[motor].distanceToGo());
    Serial.print("  ├─ Estado Driver (binario): 0b");
    Serial.println(driverStatus, BIN);
    Serial.print("  └─ Temperatura Driver: ");
    Serial.print(temperature);
    Serial.println(" °C");
}

void statusAllMotors() {
    Serial.println("=========================================");
    Serial.println("=== REPORTE DE ESTADO DE TODOS LOS EJES ===");
    Serial.println("=========================================");

    for (int i = 0; i < numSteppers; i++) {
        printMotorStatus((StepperID)i);
        if (i < numSteppers - 1) {
            Serial.println("-----------------------------------------");
        }
    }
    Serial.println("=========================================");
    Serial.println("===         FIN DEL REPORTE           ===");
    Serial.println("=========================================");
}

void processReadEncoderCommand(String command) {
    char cmd[20], motorName[5];
    sscanf(command.c_str(), "%s %s", cmd, motorName);
    StepperID motor = getMotorID(motorName);

    if (motor != (StepperID)-1) {
        float angle = readEncoderDegrees(motor);
        Serial.print("Encoder J");
        Serial.print(motor + 1);
        Serial.print(": ");
        Serial.print(angle);
        Serial.println(" grados.");
    } else {
        Serial.println("Error: Motor no encontrado.");
    }
}

void processValidateCommand(String command) {
    char cmd[20], motorName[5];
    sscanf(command.c_str(), "%s %s", cmd, motorName);
    StepperID motor = getMotorID(motorName);

    if (motor != (StepperID)-1) {
        // Posición teórica del motor (en grados de la articulación)
        long currentSteps = Motor[motor].currentPosition();
        // Usamos la función inversa a degreesToSteps
        float stepsPerDegree = (float)degreesToSteps(motor, 1.0);
        float motorAngle = (stepsPerDegree != 0) ? (currentSteps / stepsPerDegree) : 0.0;

        // Posición real del encoder
        float encoderAngle = readEncoderDegrees(motor);

        Serial.print("🔍 Validación de J");
        Serial.println(motor + 1);
        Serial.print("  -> Posición Motor Teórica: ");
        Serial.print(motorAngle, 4);
        Serial.println("°");
        Serial.print("  -> Posición Encoder Real:  ");
        Serial.print(encoderAngle, 4);
        Serial.println("°");
        Serial.print("  -> Diferencia: ");
        Serial.print(abs(motorAngle - encoderAngle), 4);
        Serial.println("°");
    } else {
        Serial.println("Error: Motor no encontrado.");
    }
}

void readSerialCommand() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        String commandUpper = command;
        commandUpper.toUpperCase();

        // --- Comandos de Configuración e Inicialización ---
        if (commandUpper.startsWith("INIT ")) {
            processInitCommand(command);
        } else if (commandUpper == "INITALL") {
            initAllMotors();
        } else if (commandUpper.startsWith("CONFIG ")) {
            processConfigCommand(command);
        }
        
        // --- Comandos de Movimiento ---
        else if (commandUpper.startsWith("MOVE ")) {
            processMoveCommand(command);
        } else if (commandUpper.startsWith("DMOVE ")) {
            processMoveDegreesCommand(command);
        } 
        
        // --- Comandos de Calibración (Homing) ---
        else if (commandUpper.startsWith("HOME")) {
            if (commandUpper == "HOME") {
                homeAllAxes();
            } else {
                processHomeCommand(command);
            }
        } 
        
        // --- Comandos de Estado y Feedback ---
        else if (commandUpper.startsWith("STATUS ")) {
            processStatusCommand(command);
        } else if (commandUpper == "STATUSALL") {
            statusAllMotors();
        } else if (commandUpper.startsWith("READENCODER ")) {
            processReadEncoderCommand(command);
        } else if (commandUpper.startsWith("VALIDATE ")) {
            processValidateCommand(command);
        }
        
        // --- Comando no reconocido ---
        else {
            Serial.println("⚠️ Comando no reconocido.");
        }
    }
}

void processConfigCommand(String command) {
    char cmd[20];
    char motorName[5];
    float maxSpeed, acceleration;

    sscanf(command.c_str(), "%s %s %f %f", cmd, motorName, &maxSpeed, &acceleration);
    StepperID motor = getMotorID(motorName);

    if (motor != -1) {
        configureMotor(motor, maxSpeed, acceleration);
    } else {
        Serial.println("Error: Motor no encontrado.");
    }
}

void processMoveCommand(String command) {
    char commandBuffer[100];  // Crear un buffer mutable
    command.toCharArray(commandBuffer, sizeof(commandBuffer)); // Copiar el comando

    char motorName[5];
    int steps;
    char *token = strtok(commandBuffer, " "); // Tokeniza la cadena

    token = strtok(NULL, " "); // Ignorar "MOVE"
    while (token != NULL) {
        strcpy(motorName, token);
        token = strtok(NULL, " ");
        if (token == NULL) break;

        steps = atoi(token);
        StepperID motor = getMotorID(motorName);

        if (motor != -1) {
            moveMotor(motor, steps);
        } else {
            Serial.print("❌ ERROR: MOTOR ");
            Serial.print(motorName);
            Serial.println(" NO ENCONTRADO.");
        }

        token = strtok(NULL, " ");
    }
}

void processInitCommand(String command) {
    char cmd[10];
    char motorName[5];
    uint8_t currentRange, runCurrent, holdCurrent, microsteps;

    sscanf(command.c_str(), "%s %s %hhu %hhu %hhu %hhu", cmd, motorName, &currentRange, &runCurrent, &holdCurrent, &microsteps);
    StepperID motor = getMotorID(motorName);

    if (motor != -1) {
        bool success = initMotor(motor, runCurrent, holdCurrent, microsteps, currentRange);
        if (success) {
            Serial.print("Motor ");
            Serial.print(motorName);
            Serial.println(" inicializado correctamente.");
        } else {
            Serial.print("Error al inicializar el motor ");
            Serial.println(motorName);
        }
    } else {
        Serial.println("Error: Motor no encontrado.");
    }
}

void processStatusCommand(String command) {
    char cmd[10];
    char motorName[5];

    sscanf(command.c_str(), "%s %s", cmd, motorName);
    StepperID motor = getMotorID(motorName);

    if (motor != (StepperID)-1) {
        // Simplemente llama a la nueva función de impresión
        printMotorStatus(motor);
    } else {
        Serial.println("❌ Error: Motor no encontrado.");
    }
}

void processMoveDegreesCommand(String command) {
    char commandBuffer[100];  // Crear un buffer mutable
    command.toCharArray(commandBuffer, sizeof(commandBuffer)); // Copiar el comando

    char motorName[5];
    float degrees, rpm;
    char *token = strtok(commandBuffer, " "); // Tokeniza la cadena

    token = strtok(NULL, " "); // Ignorar "DMOVE"
    while (token != NULL) {
        strcpy(motorName, token);
        token = strtok(NULL, " ");
        if (token == NULL) break;

        degrees = atof(token);
        token = strtok(NULL, " ");
        if (token == NULL) break;

        rpm = atof(token);
        StepperID motor = getMotorID(motorName);

        if (motor != -1) {
            moveMotorByDegrees(motor, degrees, rpm);
        } else {
            Serial.print("❌ ERROR: MOTOR ");
            Serial.print(motorName);
            Serial.println(" NO ENCONTRADO.");
        }

        token = strtok(NULL, " ");
    }
}

void processHomeCommand(String command) {
    char cmd[10];
    char motorName[5];

    sscanf(command.c_str(), "%s %s", cmd, motorName);
    StepperID motor = getMotorID(motorName);

    if (motor != (StepperID)-1) {
        homeSingleAxis(motor);
    } else {
        Serial.print("❌ ERROR: Motor '");
        Serial.print(motorName);
        Serial.println("' no encontrado para el comando HOME.");
    }
}

StepperID getMotorID(const char* motorName) {
    if (strcmp(motorName, "J1") == 0) return J1;
    if (strcmp(motorName, "J2") == 0) return J2;
    if (strcmp(motorName, "J3") == 0) return J3;
    if (strcmp(motorName, "J4") == 0) return J4;
    if (strcmp(motorName, "J5") == 0) return J5;
    if (strcmp(motorName, "J6") == 0) return J6;
    return (StepperID)-1; // Retorna -1 si el motor no existe
}

float parseTemperature(uint32_t tempData) {
    return (float)((uint16_t)(tempData & 0x00001FFF) - 2038) / 7.7;
}

