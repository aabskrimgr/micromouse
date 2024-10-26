
// this is for the button that doesn't bounce
// const unsigned long limit =5000;
// unsigned long curr =0;
// bool flag =false;

// void setup() {
//   pinMode(10, OUTPUT);
//   pinMode(7, OUTPUT);
//   pinMode(3, INPUT);

//   curr = millis();
  
//   while((millis()-curr)<limit){
//     if(digitalRead(3)==1){
//       digitalWrite(7, HIGH);
//       flag = true;
//       break;
//     }
//   }
  
//   if(!flag){
//     digitalWrite(10, HIGH);
//   }
// }

// void loop() {
//   //

// }


// for the button that bounces we use bit manipulation debouncing function

// variable holding the LED state 



const unsigned long limit =5000;
unsigned long curr =0;
bool flag =false;

bool debounce (int btn){
    static uint16_t state = 0; 
  state = (state << 1) | digitalRead(btn) | 0xfe00; 
  return (state == 0xff00); 
}

void setup() {
  pinMode(10, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, INPUT);

  curr = millis();
  
  while((millis()-curr)<limit){
    if(debounce(3)){
      digitalWrite(7, HIGH);
      flag = true;
      break;
    }
  }
  
  if(!flag){
    digitalWrite(10, HIGH);
  }
}

void loop() {
  //

}



