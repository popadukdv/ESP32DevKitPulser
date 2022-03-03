// the numbers of the pins
const int pulPin = (0b000100); // pull pin 4
const int inPin = (0b010001); // income signal pin 17

const int dirPin = (0b000010);  // dir pin 2
#define CW  GPIO.out_w1ts = dirPin; //Set pin in HIGH
#define CWW  GPIO.out_w1tc = dirPin; //Set pin in LOW

// setting PWM properties
const int maxFreq = 500000;
      int freq = 500000;
const int ledChannel = 0;
const int resolution = 6;
const int dutyCycle = 30;
const int maxSpeedMotor = 0; //rpm
const int oneRevolutionLenth = 251.3274; // lenth, mm
      int strokeLenth = 350; //mm
      int strokeTime = 3000; //ms (millis)
bool Position = 0;
bool firstDimension;

void setup(){
    Serial.println(115200);
    pinMode(inPin, INPUT);
    gpio_config_t io_out_conf; // Configuration of gpios
    io_out_conf.mode = GPIO_MODE_OUTPUT; // Set mode OUTPUT
    io_out_conf.pin_bit_mask = pulPin; // Definition of guides that will be part of this configuratio 
    esp_err_t error_out=gpio_config(&io_out_conf);//configure GPIO with the given settings

    // gpio_config_t io_in_conf; // Configuration of gpios
    // io_in_conf.mode = GPIO_MODE_INPUT; // Set mode INPUT
    // io_in_conf.intr_type = GPIO_INTR_DISABLE; //// interrupt
    // io_in_conf.pin_bit_mask = dirPin; // Definition of guides that will be part of this configuration
    // io_in_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // dis pull-down mode
    // io_in_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up mode
    // esp_err_t error_in=gpio_config(&io_in_conf);//configure GPIO with the given settings
    

    ledcSetup(ledChannel, freq, resolution); // configure PWM functionalitites
    ledcAttachPin(pulPin, ledChannel); // attach the channel to the GPIO to be controlled

   
}
 
void loop(){
   
   
  if (digitalRead(inPin) == HIGH) {
     firstDimension = true;
     delayMicroseconds(5);
     if ((firstDimension == true) && (digitalRead(inPin) == HIGH)) {
       if (Position == 0) {
          motorGoCW();
          Position = 1;
        }
      }
      firstDimension = false;
    }else {
      firstDimension = true;
      delayMicroseconds(5);
      if ((digitalRead(inPin) == LOW) && (firstDimension == true)) {
        if (Position == 1) {
              motorGoCWw();
           Position = 0;
        }
      }
      firstDimension = false;
    }
}
    
void motorGoCW() {
  CW;
  ledcWrite(ledChannel, dutyCycle);
  delay(strokeTime);
  ledcWrite(ledChannel, 0);
}

void motorGoCWw() {
  CWW;
  ledcWrite(ledChannel, dutyCycle);
  delay(strokeTime);
  ledcWrite(ledChannel, 0);
}
