#define   pino_D0  2
#define analog_pot 0
#define   fwd      4
#define   rev      6
#define led        13

int sensor();
int pid_control(int velocity);
void robot_ahead(int PID);
void robot_back(int PID); 
void contador();   

int           rpm;
volatile byte pulsos;
unsigned long timeold;

int velocity_atual; 
int velocity = 0;                                

//Variáveis PWM
int fwd_state = 0;
int rev_state = 0;
int analog_value = 0;
int pwm_fwd = 0;
int pwm_rev = 0;

unsigned int pulsos_por_volta = 20;

void setup()
{
  Serial.begin(115200);

  pinMode(pino_D0, INPUT);
  pinMode(fwd, OUTPUT);
  pinMode(rev, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(pino_D0),
                  contador,RISING);
  pulsos = 0;
  rpm = 0;
  timeold = 0;
  
}
  
void loop()
{

  sensor();
  pid_control(velocity);
  
}

void robot_ahead(int PID)                               
{
   analogWrite(fwd,  PID);
  // digitalWrite(rev,  pwm_rev);
  
} 

void robot_back(int PID)                                
{
  // analogWrite(fwd, pwm_fd);
   analogWrite(rev, PID);

} 


int sensor(){
  int pid_control(int velocity);

  if (micros() - timeold >= 1000000)
  {
    detachInterrupt(2);
    rpm = (60 * 1000000 / pulsos_por_volta ) / (micros() - timeold) * pulsos;
    timeold = micros();
    pulsos = 0;
    velocity = rpm;
// Serial.print("RPM = ");
// Serial.println(velocity, DEC);
// Serial.println(pulsos);
attachInterrupt(2, contador, FALLING);
return (velocity);
}

}

int pid_control(int velocity)                   //Função para algorimo PID
{

  float    error_meas,                            //armazena o erro
           kp = 1.0,                              //constante kp
           ki = 0.02,                             //constante ki
           kd = 0.0,                              //constante kd
           proportional,                          //armazena valor proporcional
           integral,                            //armazena valor derivativo
           PID,                                   //armazena resultado PID
           ideal_value = 150,                    //valor ideal (setpoint), setado para 20cm
           velocity_atual;                           //armazena última medida

    digitalWrite(led,  HIGH);                     //liga led vermelho
         
    error_meas = velocity - ideal_value;           //calcula erro
    
    proportional = error_meas * kp;               //calcula proporcional
    
    integral += error_meas * ki;                  //calcula integral
    
    // derivative = (lastMeasure - velocity_atual) * kd;    //calcula derivada ***há um erro de semântica aqui
                                                  // ex.: corrija o erro de semântica para o cálculo da derivada!
                          //atualiza medida
    Serial.println(velocity,DEC);
    
    PID = proportional + integral;   //calcula PID
    

 
    if(PID < 0)                                   //PID menor que zero?
    {                                             //sim
      PID = map(PID,  0, -20, 0, 255);            //normaliza para PWM de 8 bits
      robot_back(PID);                               //move robô para trás
    
    }
    
    else                                          //senão
    {                                             //PID maior ou igual a zero
      PID = map(PID,  0, 40, 0, 255);             //normaliza para PWM de 8 bits
      if(PID > 255) PID = 255;                    //se PID maior que 255, mantém em 255
      robot_ahead(PID);                              //move robô para frente
      
    }
    return (PID);

    
}

void contador()
{
  pulsos++;
}

