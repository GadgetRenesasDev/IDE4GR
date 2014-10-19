/*
  UploadXively
  This sample upload the values of sensors on Sensor Network Shield
  to Xively site via XBee WiFi transparent mode. So you need to make 
  XBee to associate with your router and set:
    BD to 115200
    IP to TCP
    DL to 216.52.233.120
    DE to 50 (is hex value.)
    

  Note: necessary to import below libraries.
        - SNShield
        - SPI
        - Wire
        
  This example code is in the public domain.
 */
 
#include <string.h>

String xively_feeds= "***"; // fill your feeds ID for Xively
String xively_key="***";    // fill your key for Xively
String xively_id1 = "eye_lux"; // fill your device1 name on Xively
String xively_id2 = "eye_temp";// fill your device2 name on Xively

#define RESPONSE_TIMEOUT 3000 //(ms) this is for waiting HTTP response

#define LED_R 22
#define LED_G 23
#define LED_B 24

SNShield kurumi;
int uploadXivelyData();
void blinkLed(int);

void setup() {
    kurumi.begin();
    Serial.begin(115200); // to USB serial
    Serial2.begin(115200); // to XBee
    
    // LED setting
    pinMode(LED_R, OUTPUT);
    digitalWrite(LED_R, HIGH);//turn off
    pinMode(LED_G, OUTPUT);
    digitalWrite(LED_G, HIGH);//turn off
    pinMode(LED_B, OUTPUT);
    digitalWrite(LED_B, HIGH);//turn off
}

void loop() {
    blinkLed(LED_B);
    if (uploadXivelyData()){
        blinkLed(LED_G);
    } else {
        blinkLed(LED_R);
    }

    delay(1000);
}

int uploadXivelyData()
{
    char data[10] = "0";

    String contents = "{\r\n";
    contents += "\"version\":\"1.0.0\",\r\n";
    contents += "\"datastreams\" : [\r\n";
    contents += " {\"id\":\"";
    contents += xively_id1;
    contents += "\",\"current_value\":\"";
    sprintf(data,"%.2f",kurumi.getLux());
    contents += data;
    contents += "\"},\r\n";
    contents += " {\"id\":\"";
    contents += xively_id2;
    contents += "\",\"current_value\":\"";
    sprintf(data,"%.2f",kurumi.getTemp());
    contents += data;
    contents += "\"}";
    contents += "\r\n";

    contents += " ]\r\n";
    contents += "}\r\n";
    contents += "\r\n";
        
    String header ="PUT /v2/feeds/";
    header += xively_feeds;
    header += ".json HTTP/1.1\r\n";
    header += "Host: api.xively.com\r\n";
    header += "User-Agent: GR-KURUMI/1.0\r\n";
    header += "Connection: close\r\n";
    header += "X-ApiKey:";
    header += xively_key;
    header += "\r\n";
    header += "User-Agent: GR-KURUMI-XBee-wifi/1.0\r\n";
    
    header += "Content-Length:";
    header += contents.length();
    header += "\r\n";
    header += "\r\n";

    header += contents;

    Serial.print(header);
    Serial2.print(header);//send to XBee
    Serial2.flush();

    // wait responce
    String res = "";
    unsigned long start = millis();
    while (int((millis() - start)) < RESPONSE_TIMEOUT) {
        if(Serial2.available()){
            res += (char)Serial2.read();
        }
    }
    Serial.print(res);
    if(res.indexOf("200 OK") > 0){
        return 1;
    }
    return 0;
}

void blinkLed(int led){
    digitalWrite(led, LOW);
    delay(200);
    digitalWrite(led, HIGH);
}
