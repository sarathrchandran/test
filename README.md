#include <SPI.h>
//#include <Timer.h>

SPIClass spi(1);
//Timer timer(1);

uint16_t spi_bytes, byte_z;
float v_out;
float vref = 3.3;
float v_data [500];

int v1, flag = 0;
int STORE_DATA = 0;
volatile int READ_ADC = 1;
volatile float Vth = 0.005;

#define TX_EN 15
#define TX_START_TIME 50 //This value decides when the TX start after ADC read starts
#define ARRY_SZE 500

int CF=15;

float TimeConst = 0.05; //time to capture 1 sample
float SoundSpeed = 1; //m/s 
float Distance; 

volatile int PulseCounter = 0;
volatile int state = 0;
//volatile int counter = 0;
volatile int d_count = 0;

int firstDip = -1;
int secondDip = -1;
int thirdDip = -1;

void PulseGen (){
  PulseCounter++;
}

void setup() {
  spi.begin();
  Serial.begin(115200);
  //digitalWrite(SS, HIGH);
  pinMode(TX_EN
  , OUTPUT);
  digitalWrite(TX_EN, LOW); 
 // timer.initialize(10000); // initialize Timer with a frequency of 40 kHz (80 MHz / 2 / 20)
  //  timer.attachInterrupt(toggleLed, 12); // Enable Timer interrupt after every 12 timer cycles (12us / 400ns)
 // timer.attachInterrupt(PulseGen, 25); // Enable Timer interrupt after every 25 timer cycles (25us / 400ns)
}

void loop() {
  if (READ_ADC){
    spi.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE2));
    //digitalWrite(SS, LOW);
    byte_z = spi.transfer16(0xFF);
    //digitalWrite(SS, HIGH);
    spi.endTransaction();

    spi_bytes = (byte_z & 0b0001111111111111); //0x1FFF)
    v_out = vref * (float(spi_bytes) / 4096.0);
    v_data[d_count++] = v_out;
  }
  
  if (d_count == TX_START_TIME){
     digitalWrite(TX_EN, HIGH); 
  }
  else if (d_count == 100){
     digitalWrite(TX_EN, LOW); 
  }
  else if (d_count >= ARRY_SZE){
//     for(d_count = 0; d_count<ARRY_SZE; d_count++){
//       //Serial.println(v_data[d_count], 4);
//       Serial.println(v_data[d_count]*100);
//     }

//-----------------------------------------------------------------------
    for (int i=100; i<(500-CF); i++){
      if(v_data[i]<v_data[i-CF] && v_data[i]<v_data[i+CF]){
        firstDip = i;
        break;
       }
    }

    if(firstDip != -1){
      for (int i=firstDip+CF; i<(500-CF); i++){
        if(v_data[i]<v_data[i-CF] && v_data[i]<v_data[i+CF]){
            secondDip = i;
            break;
        }
      }
    }

    if(secondDip != -1){
      for (int i=secondDip+CF; i<(500-CF); i++){
        if(v_data[i]<v_data[i-CF] && v_data[i]<v_data[i+CF]){
            thirdDip = i;
            break;
        }
      }
    }
    Distance = firstDip*TimeConst*SoundSpeed;
    //Distance = Distance * SoundSpeed;
    Serial.print("First Dip = "); Serial.print(firstDip); Serial.print(" Distance = ");Serial.println(Distance);
    Distance = secondDip*TimeConst*SoundSpeed;
    Serial.print("Second Dip = "); Serial.print(secondDip); Serial.print(" Distance = ");Serial.println(Distance);
    Distance = thirdDip*TimeConst*SoundSpeed;
    Serial.print("third Dip = "); Serial.print(thirdDip); Serial.print(" Distance = ");Serial.println(Distance);
//-----------------------------------------------------------------------
    d_count = 0; 
    READ_ADC = 0;
  }
    
  if(Serial.read()>0){
    READ_ADC = 1;
  }
}
