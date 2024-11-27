// Inisialisasi pin
const int motorControlPin1 = 7;   
const int motorControlPin2 = 8;   
const int speedControlPin = 9;    
const int encoderSignalPinA = 2;   

// Variabel untuk encoder dan Gearbox
volatile int encoderPulseCount = 0;   
unsigned long previousTime = 0;       
int motorRPM = 0;                      
float gearboxRatio = 9.6;
int adjustedRPM = 0;                  

const int maxMotorRPM = 630;          
const int maxPWMValue = 255;         

// PID variabel
float proportionalGain = 0.18;      
float integralGain = 0.1;            
float derivativeGain = 0.01;        

float targetRPM = 0;                  
float currentError = 0, previousError = 0, cumulativeError = 0, rateOfChangeError = 0; 
int pidOutput = 0;                  
float proportionalOutput = 0;         

// Fungsi untuk pengaturan arah motor
void setMotorDirection(bool clockwise) {
    if (clockwise) {
        digitalWrite(motorControlPin1, HIGH);
        digitalWrite(motorControlPin2, LOW);
    } else {
        digitalWrite(motorControlPin1, LOW);
        digitalWrite(motorControlPin2, HIGH);
    }
}

// Fungsi untuk komunikasi perintah dari GUI
void executeCommand(String command) {
    int delimiterIndex = command.indexOf(':');
    if (delimiterIndex > 0) {
        String paramName = command.substring(0, delimiterIndex);
        String paramValue = command.substring(delimiterIndex + 1);

        if (paramName == "Kp") {
            proportionalGain = paramValue.toFloat();
            Serial.println("Kp updated: " + String(proportionalGain));
        } else if (paramName == "Ki") {
            integralGain = paramValue.toFloat();
            Serial.println("Ki updated: " + String(integralGain));
        } else if (paramName == "Kd") {
            derivativeGain = paramValue.toFloat();
            Serial.println("Kd updated: " + String(derivativeGain));
        } else if (paramName == "RPM") {
            targetRPM = paramValue.toInt();
            Serial.println("Target RPM updated: " + String(targetRPM));
        } else if (paramName == "DIR") {
            if (paramValue == "CW") {
                setMotorDirection(true);
                Serial.println("Direction set to CW");
            } else if (paramValue == "CCW") {
                setMotorDirection(false);
                Serial.println("Direction set to CCW");
            }
        }
    }
}

// Fungsi untuk mengirim data ke GUI
void sendDataToGUI() {
    Serial.print("RPM:"); 
    Serial.println(adjustedRPM);
    Serial.print("Error:"); 
    Serial.println(currentError);
    Serial.print("PID Output:"); 
    Serial.println(proportionalOutput);
}

// Fungsi interrupt untuk menghitung pulse encoder
void countEncoderPulse() {
    encoderPulseCount++;
}

// Fungsi untuk menghitung RPM berdasarkan pulse encoder
float calculateRPM() {
    motorRPM = (encoderPulseCount / 11) * 60; 
    adjustedRPM = motorRPM / gearboxRatio;    
    encoderPulseCount = 0;                    
    return adjustedRPM;
}

// Fungsi rumus PID
int calculatePID() {
    float proportionalComponent, integralComponent, derivativeComponent, output;
    currentError = targetRPM - adjustedRPM;
    proportionalComponent = proportionalGain * currentError;
    cumulativeError += currentError;
    integralComponent = integralGain * cumulativeError;
    derivativeComponent = derivativeGain * rateOfChangeError;
    rateOfChangeError = currentError - previousError;
    previousError = currentError;
    output = proportionalComponent + integralComponent + derivativeComponent;
    pidOutput = constrain(output, 0, maxPWMValue);
    return pidOutput;
}

void setup() {
    Serial.begin(9600); 
    pinMode(motorControlPin1, OUTPUT);
    pinMode(motorControlPin2, OUTPUT);
    pinMode(speedControlPin, OUTPUT);
    pinMode(encoderSignalPinA, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoderSignalPinA), countEncoderPulse, RISING);
}

void loop() {
    unsigned long currentTime = millis();

    // Hitung kecepatan motor setiap 1000 ms
    if (currentTime - previousTime >= 1000) {
        calculateRPM();
        previousTime = currentTime;
        pidOutput = calculatePID();
        analogWrite(speedControlPin, pidOutput);

        
        sendDataToGUI();
    }

    // Cek perintah dari serial
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        executeCommand(command);
    }
}