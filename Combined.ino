#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "esp_camera.h"
#include <base64.h>
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#include <HTTPClient.h> 
#include <WiFi.h> 

#define I2C_SDA 14 // SDA Connected to GPIO 14
#define I2C_SCL 15 // SCL Connected to GPIO 15

const byte MLX90640_address = 0x33;  // Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8                   // Default shift for MLX90640 in open air

float mlx90640To[768];  //buffer for full frame size
paramsMLX90640 mlx90640;
String current_val = "";
String command = "";
unsigned long startTime = 0;
int photo_count = 0;

String image_data;
String station1;
String postData;
char *Link;

/* Set these to your desired credentials. */
const char *ssid = "Insharp_Life";//ENTER YOUR WIFI SETTINGS
const char *password = "*!1%0EHai4Qw$1M8a%";

//const char* Link = "http://192.168.1.72:3000/routeDataFromNodeMCUToServer";


float start;
float end;
float timet;

camera_config_t config; 

//=======================================================================
//                    Power on setup
//=======================================================================

void setup() {

  Wire.begin(I2C_SDA, I2C_SCL, 400000); 
  //Wire.setClock();  // Increase I2C clock speed to 400kHz
  WiFi.begin(ssid, password); 
  Serial.begin(115200);
  camera_init(); 
  delay(1000);
  Serial.print("Connecting");
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("WIFI COnnected");

  //If connection successful show IP address in serial monitor
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  // Initialize the MLX90640 sensor
  if (!isCamConnected()) {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1)
      ;
  }

  // Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  // Once params are extracted, we can release eeMLX90640 array
  MLX90640_SetRefreshRate(MLX90640_address, 0x03);  // Set rate to 4Hz
}

//=======================================================================
//                    Main Program Loop
//=======================================================================
void loop() {

  HttpConnection(TakeRGBPhoto(), take_thermal_picture()); 
  Serial.flush();  // flushing the previous frames
  photo_count++;
  if (photo_count >= 10) photo_count = 2;
  delay(250);
}

String take_thermal_picture() {
  current_val = "";
  start = millis();
  // Get a frame of temperature data from the MLX90640 sensor
  uint16_t mlx90640Frame[834];
  int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);

  // Calculate ambient temperature and object temperature for each pixel
  float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
  float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  float tr = Ta - TA_SHIFT;  // Reflected temperature based on the sensor ambient temperature
  float emissivity = 0.95;
  MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);

  // Print the temperature data to serial monitor
  for (uint8_t h = 0; h < 24; h++) {
    for (uint8_t w = 0; w < 32; w++) {
      float t = mlx90640To[h * 32 + w];

      if (photo_count >= 2) {
        if (w < 31) {
          current_val += String(t, 1) + ",";          

        } else {

          if (h < 23) {
            current_val += String(t, 1) + ",";

          } else {

            current_val += String(t, 1);
            //Serial.println(current_val);
          }
        }
      }
    }
  }
  Serial.print("Thermal: ");
  Serial.println(current_val);

  
  Serial.print("\n\n");
  end = millis();
  Serial.flush();  // flushing the previous frames 
  return current_val;
  
}

bool isCamConnected() {                        //Initializing the connection with the mlx90640 sensor data transmission
  Wire.beginTransmission(MLX90640_address);
  if (Wire.endTransmission() == 0)
    return true;
  else
    return false;
}

void camera_init(){
  Serial.setDebugOutput(true);

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 10000000;                  /* The clock frequency determines the frame rate togeather with the framesize */
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

    // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  Serial.println("Camera init success");
  
}


String TakeRGBPhoto() {
  image_data = ""; 
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return "Error";
  }

  // Convert camera frame to base64 string
  image_data = base64::encode(fb->buf, fb->len);

  Serial.print("RGB: "); 
  Serial.println(image_data);
  esp_camera_fb_return(fb);
  return image_data;


  delay(100);  // Delay for one second before capturing next frame
}

///----HTTP COnnection-----///

void HttpConnection(String image_data, String current_val_new) {

  HTTPClient http;  //Declare object of class HTTPClient

  station1 = "A";      // test data for the variable

  //GET Data
  postData = "{\"station1\" : \"" + station1 + "\", \"current_val\": \"" + current_val_new + "\", \"image_data\": \"" + image_data + "\"}";  //Note "?" added at front
  Link = "http://192.168.1.72:3000/routeDataFromEsp32ToServer";      // Add the current ip address of the working pc and the excuting server file

  http.begin(Link);  //Specify request destination

  int httpCode = http.POST(postData);          //Send the request
  String responce = http.getString();  //Get the response payload

  Serial.println(postData); //payload print
  Serial.println(httpCode);  //Print HTTP return code
  Serial.println(responce);   //Print request response payload
  // Serial.println(current_val);

  http.end();  //Close connection
  delay(100);  //GET Data at every 0.1 seconds

}
