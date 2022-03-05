#include <WiFi.h>

// Network credentials
const char* ssid     = "ESP32-A1-Access-Point";
const char* password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

//Store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;


// the numbers of the pins
const int pulPin = (0b000100); // pull pin 4
const int inPin = (0b010001); // income signal pin 17

const int dirPin = (0b000010);  // dir pin 2
#define CW  GPIO.out_w1ts = dirPin; //Set pin in HIGH
#define CWW  GPIO.out_w1tc = dirPin; //Set pin in LOW

// setting PWM properties
const long maxFreq = 500000;
      long freq = 500000;
const int ledChannel = 0;
const int resolution = 6;
const int dutyCycle = 30;
const int maxSpeedMotor = 3000; //rpm
const float oneRevolutionLenth = 251.3274; // lenth, mm
const int reducer = 10;
      int strokeLenth = 350; //mm
      int strokeTime = 500; //ms (millis)
bool Position = 0;
bool firstDimension;

void speedCulculation();
void motorGoCW();
void motorGoCWw();

void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();

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
    
    speedCulculation();
    ledcSetup(ledChannel, freq, resolution); // configure PWM functionalitites
    ledcAttachPin(pulPin, ledChannel); // attach the channel to the GPIO to be controlled

    
}

void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
              motorGoCW();
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
              motorGoCWw();
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(output27, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(output27, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>A1 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 27  
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button       
            if (output27State=="off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  
   
  if (digitalRead(inPin) == HIGH) {
     firstDimension = true;
     delayMicroseconds(5);
     if ((firstDimension == true) && (digitalRead(inPin) == HIGH)) {
        if (Position == 0) {
          motorGoCW();
          Position = 1;  
      }
      firstDimension = false;
    }
  }else{
      firstDimension = true;
      delayMicroseconds(5);
      if ((firstDimension == true) && (digitalRead(inPin) == LOW)) {
        if (Position == 1) {
              motorGoCWw();
          Position = 0;
        }
      }
      firstDimension = false;
    }
  
}

 //Motor`s rotor go forward   
void motorGoCW() {
  // configure PWM functionalitites
  ledcSetup(ledChannel, freq, resolution); 
  CW;
  ledcWrite(ledChannel, dutyCycle);
  delay(strokeTime);
  ledcWrite(ledChannel, 0);
}
//Motor`s rotor go back 
void motorGoCWw() {
  // configure PWM functionalitites
  ledcSetup(ledChannel, freq, resolution); // configure PWM functionalitites
  CWW;
  ledcWrite(ledChannel, dutyCycle);
  delay(strokeTime);
  ledcWrite(ledChannel, 0);
}
//Culculate the freaqency is needed to stroke necessary length for certain time (milliseconds)
void speedCulculation() {
      // How many motor`s sircle needs to rotorRotation
      float motorRotation = (float)strokeLenth / oneRevolutionLenth * (float)reducer;  
      // Speed is needed for motorRotation in strokeTime
      float speedMotor = (motorRotation * 60000/*millis in one hour*/) / strokeTime;
      // friquency is needed for motorRotation in strokeTime
      long fr = (maxFreq * speedMotor) / maxSpeedMotor;
      freq = (fr < maxFreq) ? fr : maxFreq;
}
