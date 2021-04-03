#include "mbed.h"
// Biblioteca para lectura de DHT11 creada por Wim De Roeve, disponible en https://os.mbed.com/users/Wimpie/code/DHT/
#include "DHT.h" 
// Biblioteca para lectura de HX711 de la celda de carga, creada por Stephen Laskowski, disponible en https://os.mbed.com/users/laskowsk/code/HX711/
#include "HX711.h"

// Comunicacion serial mediante protocolo UART
Serial pc(USBTX, USBRX);
char dato;
char *token;
const int kMaxBufferSize = 100;
char      buffer[kMaxBufferSize];
int       len = 0;

Ticker deltaT; // Ticker que genera interrupcion cada 1 ms

// PINES SERVOS

PwmOut Servo1(PB_8); // TIMER 2 - CANAL 1
PwmOut Servo2(PC_9); // TIMER 3 - CANAL 4
PwmOut Servo3(PC_8); // TIMER 3 - CANAL 3
PwmOut ServoEfector(PB_9_ALT0); // TIMER 4 - CANAL 4
PwmOut ServoLab(PC_6); // TIMER 3 - CANAL 1

// PINES MOTORES CD

// Motor 1
DigitalOut DIR_CD_1(PC_4);
PwmOut PWM_CD_1(PA_10); // TIMER 1 - CANAL 3
// ENCODER 1
InterruptIn ENC_1A(PC_3);
InterruptIn ENC_1B(PC_2);

// Motor 2
DigitalOut DIR_CD_2(PB_4);
PwmOut PWM_CD_2(PB_5); // TIMER 3 - CANAL 2
// ENCODER 2
InterruptIn ENC_2A(PC_0);
InterruptIn ENC_2B(PC_1);

// Motor 3
DigitalOut DIR_CD_3(PB_13);
PwmOut PWM_CD_3(PB_3); // TIMER 2 - CANAL 2
// ENCODER 3
InterruptIn ENC_3A(PA_8);
InterruptIn ENC_3B(PB_10);

// Sensores de temperatura

AnalogIn adc_temp(ADC_TEMP); // Sensor interno
DHT DHT11_MOTORES(PD_2,SEN11301P); // Use the SEN11301P sensor
DHT DHT11_FUENTE(PC_11,SEN11301P); // Use the SEN11301P sensor

// Celda de carga
//HX711::HX711(PinName pinData, PinName pinSck, uint8_t gain)
HX711 celda(PC_12, PC_10, 128);

// Declaracion de variables

// Valores de temperatura

float TempInterna;
float TempMotores;
float TempFuente;

// Valor de masa de la muestra
float masa = 0;

// Activacion de rutina

bool orden = 0;

// Numero de pulsos por revolucion como encoder de cuadratura
double PPRQ = 5264.0;

// Posiciones angulares de los motores

// Angulos iniciales

// Valores de Home
double theta1Home = 59;
double theta2Home = 54;
double theta3Home = 153;
double theta4Home = 10;
double theta5Home = 10;
double theta6Home = 15;

double theta3Inter = 80.0; // Posicion auxiliar


// Valores de laboratorio

double theta1Lab = 82;
double theta2Lab = 18;
double theta3Lab = 120;
double theta4Lab = 80;
double theta5Lab = 160;
double theta6Lab = 26;

// Ciclo util del PWM del motor
uint16_t pwm = 0;

// Angulos del Servo en posiciones Home

int pulse;

int LaboratorioCerrado = 50;
int LaboratorioAbierto = 0;

int EfectorAbierto = 100; // Para que regrese a la posicion original con mas fuerza, ciclo de 5 segundos
int EfectorCerrado = 80; // Para que cierre con fuerza, ciclo de 5 segundos 
int EfectorDetenido = 90;

// Inicializacion de variables

// Valores crudos del encoder
volatile double encVal1 = -1.0*((theta1Home*PPRQ)/360.0);
volatile double encVal2 = -1.0*((theta2Home*PPRQ)/360.0);
volatile double encVal3 = 1.0*((theta3Home*PPRQ)/360.0);

double theta1 = theta1Home;
double theta2 = theta2Home;
double theta3 = theta3Home;
double theta4 = theta4Home;
double theta5 = theta5Home;
double theta6 = theta6Home;

// Posiciones objetivo para recoger la muestra, recibidos de la GUI por serial

double theta1D;
double theta2D;
double theta3D;
double theta4D;
double theta5D;
double theta6D;

double theta4aux;
double theta5aux;
double theta6aux;

// Variables usadas por el control P

double theta2D_P = theta2Home;
double theta2aux;
double eP = 0.0; // Error que utiliza el contro P
double PWM_P = 0.0;
double kp = 1000.0; // Ganancia Proporcional

// Valores auxiliares de angulos

double DT; // Diferencia entre angulo inicial y deseado
double DTabs; // El perfil de velocidad funciona con valores positivos
double Te; // Error entre el angulo actual y el deseado
double TE; // DT - Te - Te decrece del error máximo a 0, en el control se requiere que crezca de 0 al error maximo

double EA = 1.0; // Error aceptable


// Vectores de interrupcion

// Encoders motor CD_1

void ENC_1A_RISE()
{
    if(ENC_1B == 1) encVal1--; // CCW
    else encVal1++;// CW
}

void ENC_1A_FALL()
{
    if(ENC_1B == 1) encVal1++; // CW
    else encVal1--;// CCW
}

void ENC_1B_RISE()
{
    if(ENC_1A == 1) encVal1++;// CW
    else encVal1--;// CCW
}

void ENC_1B_FALL()
{
    if(ENC_1A == 1) encVal1--;// CCW
    else encVal1++;// CW
}

// Encoders motor CD_2

void ENC_2A_RISE()
{
    if(ENC_2B == 1) encVal2--;// CCW
    else encVal2++;// CW
}

void ENC_2A_FALL()
{
    if(ENC_2B == 1) encVal2++;// CW
    else encVal2--;// CCW
}

void ENC_2B_RISE()
{
    if(ENC_2A == 1) encVal2++;// CW
    else encVal2--;// CCW
}

void ENC_2B_FALL()
{
    if(ENC_2A == 1) encVal2--;// CCW
    else encVal2++;// CW
}

// Encoders motor CD_3

void ENC_3A_RISE()
{
    if(ENC_3B == 1) encVal3--;// CCW
    else encVal3++;// CW
}

void ENC_3A_FALL()
{
    if(ENC_3B == 1) encVal3++;// CW
    else encVal3--;// CCW
}

void ENC_3B_RISE()
{
    if(ENC_3A == 1) encVal3++;// CW
    else encVal3--;// CCW
}

void ENC_3B_FALL()
{
    if(ENC_3A == 1) encVal3--;// CCW
    else encVal3++;// CW
}

void P()
{
    theta2 = (-360.0)*(encVal2/PPRQ); // Conversion a posicion angular 
    eP = theta2D_P - theta2;
    PWM_P = kp * eP;
    
    if(PWM_P < 0) 
    {
        PWM_P = -1.0 * PWM_P;
        DIR_CD_2 = 1; // CW
    }
    else DIR_CD_2 = 0; // CCW
    
    if (PWM_P > 2100) PWM_P = 2100; // 20% del ciclo útil
    
    PWM_CD_2.pulsewidth_us(PWM_P);
}

void callback()
{
    // Recepcion de cadena caracter por caracter
    dato = pc.getc();
      
    buffer[len++] = dato;    
    buffer[len] = '\0';          
    
    // Separacion de string en posiciones angulares deseadas
    if (dato == 'a') // Caracter de fin de cadena
    {
        len = 0;
        
        token = strtok(buffer, ",;");
        theta1D = atoi(token);
        
        token = strtok(NULL, ",;");
        theta2D = atoi(token);
        
        token = strtok(NULL, ",;");
        theta3D = atoi(token);
        
        token = strtok(NULL, ",;");
        theta4D = 90 + atoi(token);
        
        token = strtok(NULL, ",;");
        theta5D = 100 + atoi(token);
        
        token = strtok(NULL, ",;");
        theta6D = 15 + atoi(token);
        
        orden = 1; // Hay una orden por atender
    }
}

// Funciones auxiliares

// Posicion de servos
void ServoWrite(int Servo, int angle)
{
    pulse = 600 + 10*angle;
    if (Servo == 1) Servo1.pulsewidth_us(pulse);
    else if (Servo == 2) Servo2.pulsewidth_us(pulse);
    else if (Servo == 3) Servo3.pulsewidth_us(pulse);
    else if (Servo == 4) ServoEfector.pulsewidth_us(pulse);
    else if (Servo == 5) ServoLab.pulsewidth_us(pulse);
}

void ControlServo(int Servo, int angle)
{
    if (Servo == 1) 
    {
        theta4aux = theta4;
        Te = angle - theta4aux;
        
        while ((Te < -1.0*EA)||(Te > EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            if(Te<0)
            {
                theta4aux--;
            }
            else
            {
                theta4aux++;
            }
            Te = floor(angle - theta4aux);
            ServoWrite(Servo,theta4aux);
            wait(0.05);
        }
        theta4 = theta4aux;
    }
    else if (Servo == 2)
    {
        theta5aux = theta5;
        Te = angle - theta5aux;
        
        while ((Te < -1.0*EA)||(Te > EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            if(Te<0)
            {
                theta5aux--;
            }
            else
            {
                theta5aux++;
            }
            Te = floor(angle - theta5aux);
            ServoWrite(Servo,theta5aux);
            wait(0.05);
        }
        theta5 = theta5aux;
    }
    else if (Servo == 3) 
    {
        theta6aux = theta6;
        Te = angle - theta6aux;
        
        while ((Te < -1.0*EA)||(Te > EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            if(Te<0)
            {
                theta6aux--;
            }
            else
            {
                theta6aux++;
            }
            Te = floor(angle - theta6aux);
            ServoWrite(Servo,theta6aux);
            wait(0.05);
        }
        theta6 = theta6aux;
    }
    ServoWrite(Servo,angle);
}

void AbrirEfector()
{
    ServoWrite(4,EfectorAbierto);
    wait(5);
    ServoWrite(4,EfectorDetenido);
}

void CerrarEfector()
{
    ServoWrite(4,EfectorCerrado);
    wait(5);
    ServoWrite(4,EfectorDetenido);
}

// Rutina de control

void Control(int Motor, double ThD, int pwmMAX, int pwmMIN)
{   
    if(Motor == 1)
    {   
        theta1 = (-360.0)*(encVal1/PPRQ); // Conversion a posicion angular
        Te = ThD - theta1;
        DT = Te;
        
        if(DT < 0) DTabs = -1.0 * DT; // El programa funciona con valores positivos
        else DTabs = DT;
        
        while ((Te < -1.0*EA)||(Te > EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            // Determinacion de sentido de giro
            if (Te < 0) // Error negativo - Girar en sentidio antihorario
            {
                DIR_CD_1 = 0;
            }
            else 
            {
                DIR_CD_1 = 1;
            }
            
            TE = DT - Te;
            
            if(TE < 0) TE = -1.0 * TE; // El programa funciona con valores positivos
            
            if( (0 <= TE) && (TE < (DTabs/3)) )
            {
                if(pwm < pwmMAX) pwm++;
            }
            else if( ((DTabs/3) <= TE) && (TE < (2*DTabs/3)) )
            {
                pwm = pwmMAX;
            }
            else if( ((2*DTabs/3) <= TE) && (TE <= DTabs) )
            {
                if(pwm > pwmMIN) pwm--;
            }
            
            PWM_CD_1.pulsewidth_us(pwm);
            
            theta1 = (-360.0)*(encVal1/PPRQ);
            
            Te = ThD - theta1;
        }
        PWM_CD_1.pulsewidth_us(0);
    }
    else if(Motor == 2)
    {   
        theta2aux = theta2;
        Te = ThD - theta2aux;
        
        while ((Te < -1.0*EA)||(Te > EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            if(Te<0)
            {
                theta2aux--;
            }
            else
            {
                theta2aux++;
            }
            
            theta2D_P = theta2aux;
            Te = floor(ThD - theta2D_P);
            wait(0.15);
        }
    }
    else if(Motor == 3)
    {
        theta3 = (360.0)*(encVal3/PPRQ); // Conversion a posicion angular
        Te = ThD - theta3;
        DT = Te;
        
        if(DT < 0) DTabs = -1.0 * DT; // El programa funciona con valores positivos
        else DTabs = DT;
        
        while ((Te < -2.0*EA)||(Te > 2.0*EA))// Repetir hasta que el error se encuentre en el rango aceptable
        {
            // Determinacion de sentido de giro
            if (Te < 0) // Error negativo - Girar en sentidio antihorario
            {
                DIR_CD_3 = 0;
            }
            else 
            {
                DIR_CD_3 = 1;
            }
            
            TE = DT - Te;
            
            if(TE < 0) TE = -1.0 * TE; // El programa funciona con valores positivos
            
            if( (0 <= TE) && (TE < (DTabs/3.0)) )
            {
                if(pwm < pwmMAX) pwm++;
            }
            else if( ((DTabs/3.0) <= TE) && (TE < (2.0*DTabs/3.0)) )
            {
                pwm = pwmMAX;
            }
            else if( ((2.0*DTabs/3.0) <= TE) && (TE <= DTabs) )
            {
                if(pwm > pwmMIN) pwm -= 2;
            }
            
            PWM_CD_3.pulsewidth_us(pwm);
            theta3 = (360.0)*(encVal3/PPRQ);
            
            Te = ThD - theta3;
        }
        PWM_CD_3.pulsewidth_us(0);
    }
}

void setup()
{
    // Configuracion de Ticker para Control P
    
    deltaT.attach(&P, 0.001); // Interrupcion generada cada 1ms para realizar el control P
    
    // Configuracion de comunicacion serial
    pc.baud(250000);
    pc.format(8, SerialBase::None, 1);
    pc.attach(&callback);
    
    // Interrupciones Encoders
    ENC_1A.rise(&ENC_1A_RISE);
    ENC_1A.fall(&ENC_1A_FALL);
    
    ENC_1B.rise(&ENC_1B_RISE);
    ENC_1B.fall(&ENC_1B_FALL);
    
    ENC_2A.rise(&ENC_2A_RISE);
    ENC_2A.fall(&ENC_2A_FALL);
    
    ENC_2B.rise(&ENC_2B_RISE);
    ENC_2B.fall(&ENC_2B_FALL);
    
    ENC_3A.rise(&ENC_3A_RISE);
    ENC_3A.fall(&ENC_3A_FALL);
    
    ENC_3B.rise(&ENC_3B_RISE);
    ENC_3B.fall(&ENC_3B_FALL);
    
    // Establecimiento de periodo de la senal de pwm

    Servo1.period_ms(20);
    Servo2.period_ms(20);
    Servo3.period_ms(20);
    ServoEfector.period_ms(20);
    ServoLab.period_ms(20);
    PWM_CD_1.period_ms(20);
    PWM_CD_2.period_ms(20);
    PWM_CD_3.period_ms(20);
    
    ControlServo(1,theta4Home);
    ControlServo(2,theta5Home);
    ControlServo(3,theta6Home);
    ServoWrite(4, EfectorDetenido);
    ServoWrite(5, LaboratorioCerrado);
    
    float calibration_factor = 11500; // Valor obtenido de la calibracion
    celda.setScale(0);
    celda.tare(); // Reiniciar el peso a 0
    celda.setScale(calibration_factor); // Ajustar con el valor de calibacion obtenido
    
}

void loop()
{
    if (orden == 1) // Atender la orden recibida
    {
        // Mover a posicion deseada
        Control(3,theta3Inter,2500, 800);
        Control(2,theta2D,0, 0); // El segundo GDL se controla en la interrupcion de control P
        Control(1,theta1D,1500, 600);
        
        ControlServo(1,theta4D);
        ControlServo(2,theta5D);
        
        Control(3,theta3D,1000, 400);
        
        
        CerrarEfector();
        wait(3);
        
        // Mover a laboratorio
        Control(3,theta3Inter,3000, 2000);
        Control(1,theta1Lab,1500, 600);
        
        ControlServo(3,theta6Lab);
        ControlServo(2,theta5Lab);
        ControlServo(1,theta4Lab);
        
        Control(2,theta2Lab,0, 0);
        Control(3,theta3Lab,1000, 400);
        
        
        AbrirEfector();
        
        // Lectura de peso con la celda de carga
        masa = celda.getGram();
        
        // Liberar muestra
        ServoWrite(5,LaboratorioAbierto);
        wait(1);
        ServoWrite(5,LaboratorioCerrado);
        
        
        // Regresar a Home
        
        Control(2,theta2Home,0,0);
        
        ControlServo(3,theta6Home);
        ControlServo(1,theta4Home);
        ControlServo(2,theta5Home);
        
        Control(1,theta1Home,1500, 600);
        Control(3,theta3Home,500, 500);
        
        // Rutina terminada
        
        // Lectura de temperatura
        DHT11_MOTORES.readData();
        DHT11_FUENTE.readData();
        
        TempInterna = adc_temp.read()*100;
        TempMotores = DHT11_MOTORES.ReadTemperature(CELCIUS);
        TempFuente = DHT11_FUENTE.ReadTemperature(CELCIUS);
        
        // Enviar datos obtenidos a la computadora y notificar que termino la rutina para esperar nueva orden
        
        pc.printf("%f,%f,%f,%f\n", TempMotores,TempFuente,TempInterna,masa);
        
        orden = 0; // Se atendio la orden
    }
}

int main()
{
    setup();
    while(1) {
        loop();
    }
}