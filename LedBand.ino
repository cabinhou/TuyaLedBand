  /*
   * @FileName: LedBand.ino
   * @Author: Tuya
   * @LastEditors: Kevin
   * @Date: 2021-06-10 11:24:27
   */
  
  #include <TuyaWifi.h>
  #include <SoftwareSerial.h>
  #include <Adafruit_NeoPixel.h>    //NEOPIXEL BEST PRACTICES for most reliable operation
  
  // Which pin on the Arduino is connected to the NeoPixels?
  // On a Trinket or Gemma we suggest changing this to 1:
  #define LED_PIN    6
  
  // How many NeoPixels are attached to the Arduino?
  #define LED_COUNT 60
  
  // Declare our NeoPixel strip object:
  Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
    
  
  TuyaWifi my_device;
    
  /* Current LED status */
  unsigned char led_state = 0;
  unsigned int  work_mode = 0;
  unsigned int  bright_value = 0; 
  unsigned int  colour_data = 0; 
  unsigned int  music_data = 0;  
  unsigned int  dreamlight_scene_mode = 0; 
  unsigned int  dreamlightmic_music_data = 0; 
  unsigned int  lightpixel_number_set = 0; 
  unsigned int  scene_mode=0;
  
  
  /* Connect network button pin */
  int key_pin = 7;
  
  /* Data point define */
  /******************************************************************************
                          1:dp数据点序列号重新定义
            **此为自动生成代码,如在开发平台有相关修改请重新下载MCU_SDK**         
  ******************************************************************************/
  //开关(可下发可上报)
  //备注:
  #define DPID_SWITCH_LED 20
  
  //模式(可下发可上报)
  //备注:
  #define DPID_WORK_MODE 21
  
  //亮度值(可下发可上报)
  //备注:
  #define DPID_BRIGHT_VALUE 22
  
  //冷暖值(可下发可上报)
  //备注:
  #define DPID_TEMP_VALUE 23
  
  //彩光(可下发可上报)
  //备注:类型：字符；
  //Value: 000011112222；
  //0000：H（色度：0-360，0X0000-0X0168）；
  //1111：S (饱和：0-1000, 0X0000-0X03E8）；
  //2222：V (明度：0-1000，0X0000-0X03E8)
  #define DPID_COLOUR_DATA 24
  
  //场景(可下发可上报)
  //备注:类型：字符; 
  //Value: 0011223344445555666677778888;
  //00：情景号;
  //11：单元切换间隔时间（0-100）;
  //22：单元变化时间（0-100）;
  //33：单元变化模式（0静态1跳变2渐变）;
  //4444：H（色度：0-360，0X0000-0X0168）;
  //5555：S (饱和：0-1000, 0X0000-0X03E8);
  //6666：V (明度：0-1000，0X0000-0X03E8);
  //7777：白光亮度（0-1000）;
  //8888：色温值（0-1000）;
  //注：数字1-8的标号对应有多少单元就有多少组
  #define DPID_SCENE_DATA 25
  
  //倒计时剩余时间(可下发可上报)
  //备注:
  #define DPID_COUNTDOWN 26
  
  //音乐灯(只下发)
  //备注:类型：字符串；
  //Value: 011112222333344445555；
  //0：   变化方式，0表示直接输出，1表示渐变；
  //1111：H（色度：0-360，0X0000-0X0168）；
  //2222：S (饱和：0-1000, 0X0000-0X03E8）；
  //3333：V (明度：0-1000，0X0000-0X03E8）；
  //4444：白光亮度（0-1000）；
  //5555：色温值（0-1000）
  #define DPID_MUSIC_DATA 27
  
  //调节(只下发)
  //备注:类型：字符串 ;
  //Value: 011112222333344445555  ;
  //0：   变化方式，0表示直接输出，1表示渐变;
  //1111：H（色度：0-360，0X0000-0X0168）;
  //2222：S (饱和：0-1000, 0X0000-0X03E8);
  //3333：V (明度：0-1000，0X0000-0X03E8);
  //4444：白光亮度（0-1000）;
  //5555：色温值（0-1000）
  #define DPID_CONTROL_DATA 28
  
  //入睡(可下发可上报)
  //备注:灯光按设定的时间淡出直至熄灭
  #define DPID_SLEEP_MODE 31
  
  //唤醒(可下发可上报)
  //备注:灯光按设定的时间逐渐淡入直至设定的亮度
  #define DPID_WAKEUP_MODE 32
  
  //断电记忆(可下发可上报)
  //备注:通电后，灯亮起的状态
  #define DPID_POWER_MEMORY 33
  
  //勿扰模式(可下发可上报)
  //备注:适用经常停电区域，开启通电勿扰，通过APP关灯需连续两次上电才会亮灯
  //Value：ABCCDDEEFFGG
  //A：版本，初始版本0x00；
  //B：模式，0x00初始默认值、0x01恢复记忆值、0x02用户定制；
  //CC：色相 H，0~360；
  //DD：饱和度 S，0~1000；
  //EE：明度 V，0~1000；
  //FF：亮度，0~1000；
  //GG：色温，0~1000；
  #define DPID_DO_NOT_DISTURB 34
  
  //麦克风音乐律动(可下发可上报)
  //备注:类型：  字符串
  //Value：  AABBCCDDEEFFGGGGHHHHIIIIJJJJKKKKLLLLMMMMNNNN
  //AA  版本
  //BB  0-关闭，1-打开
  //CC  模式编号，自定义从201开始
  //DD  变换方式：0 - 呼吸模式，1 -跳变模式 ， 2 - 经典模式
  //EE  变化速度
  //FF  灵敏度
  //GGGG  颜色1-色相饱和度
  //HHHH  颜色2-色相饱和度
  //......
  //NNNN  颜色8-色相饱和度
  #define DPID_MIC_MUSIC_DATA 42
  
  //炫彩情景(可下发可上报)
  //备注:专门用于幻彩灯带场景
  //Value：ABCDEFGHIJJKLLM...
  //A：版本号；
  //B：情景模式编号；
  //C：变化方式（0-静态、1-渐变、2跳变、3呼吸、4-闪烁、10-流水、11-彩虹）
  //D：单元切换间隔时间（0-100）;
  //E：单元变化时间（0-100）；
  //FGH：设置项；
  //I：亮度（亮度V：0~100）；
  //JJ：颜色1（色度H：0-360）；
  //K：饱和度1 (饱和度S：0-100)；
  //LL：颜色2（色度H：0-360）；
  //M：饱和度2（饱和度S：0~100）；
  //注：有多少个颜色单元就有多少组，最多支持20组；
  //每个字母代表一个字节
  #define DPID_DREAMLIGHT_SCENE_MODE 51
  
  //炫彩本地音乐律动(可下发可上报)
  //备注:专门用于幻彩灯带本地音乐
  //Value：ABCDEFGHIJKKLMMN...
  //A：版本号；
  //B：本地麦克风开关（0-关、1-开）；
  //C：音乐模式编号；
  //D：变化方式；
  //E：变化速度（1-100）;
  //F：灵敏度(1-100)；
  //GHI：设置项；
  //J：亮度（亮度V：0~100）；
  //KK：颜色1（色度H：0-360）；
  //L：饱和度1 (饱和度S：0-100)；
  //MM：颜色2（色度H：0-360）；
  //N：饱和度2（饱和度S：0~100）；
  //注：有多少个颜色单元就有多少组，最多支持8组；
  //每个字母代表一个字节
  #define DPID_DREAMLIGHTMIC_MUSIC_DATA 52
  
  //点数/长度设置(可下发可上报)
  //备注:幻彩灯带裁剪之后重新设置长度
  #define DPID_LIGHTPIXEL_NUMBER_SET 25
  
  
  /* WORK_MODE define ：white, colour, scene, music (del: RGB, Flowing, Rainbow, Theater) */
  //
  #define  white  0
  #define  colour 1
  #define  scene 2
  #define  music 3
  #define  rock  4
    
  
  /* Stores all DPs and their types. 
   *  PS: array[][0]:dpid, array[][1]:dp type. 
   *     dp type(TuyaDefs.h) : DP_TYPE_RAW, DP_TYPE_BOOL, DP_TYPE_VALUE, DP_TYPE_STRING, DP_TYPE_ENUM, DP_TYPE_BITMAP
  */
  unsigned char dp_array[][2] =
  {
    {DPID_SWITCH_LED, DP_TYPE_BOOL},
    {DPID_WORK_MODE, DP_TYPE_ENUM},
    {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
    {DPID_TEMP_VALUE, DP_TYPE_VALUE},
    {DPID_COLOUR_DATA, DP_TYPE_STRING},
    {DPID_SCENE_DATA, DP_TYPE_STRING},
    {DPID_COUNTDOWN, DP_TYPE_VALUE},
    {DPID_MUSIC_DATA, DP_TYPE_STRING},
    {DPID_CONTROL_DATA, DP_TYPE_STRING},
    {DPID_SLEEP_MODE, DP_TYPE_RAW},
    {DPID_WAKEUP_MODE, DP_TYPE_RAW},
    {DPID_POWER_MEMORY, DP_TYPE_RAW},
    {DPID_DO_NOT_DISTURB, DP_TYPE_BOOL},
    {DPID_MIC_MUSIC_DATA, DP_TYPE_STRING},
    {DPID_DREAMLIGHT_SCENE_MODE, DP_TYPE_RAW},
    {DPID_DREAMLIGHTMIC_MUSIC_DATA, DP_TYPE_RAW},
    {DPID_LIGHTPIXEL_NUMBER_SET, DP_TYPE_VALUE},
  };
  
  unsigned char pid[] = {"1uadqun4izsvm1jm"};
  unsigned char mcu_ver[] = {"1.0.0"};
  
  /* last time */
  unsigned long last_time = 0;
  unsigned long music_last_time = 0;
  
  void setup() 
  {
    Serial.begin(9600);
  
    //Initialize led port, turn off led.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
  
    //Initialize networking keys.
    pinMode(key_pin, INPUT_PULLUP);
  
    //NeoPixel INIT
    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  
    //Enter the PID and MCU software version
    my_device.init(pid, mcu_ver);
    //incoming all DPs and their types array, DP numbers
    my_device.set_dp_cmd_total(dp_array, 17);    //DP点的数量，根据dp_array的长度来修改
    //register DP download processing callback function
    my_device.dp_process_func_register(dp_process);
    //register upload all DP callback function
    my_device.dp_update_all_func_register(dp_update_all);
  
    last_time = millis();
    music_last_time = millis();
  }
  
  void loop() 
  {
    my_device.uart_service();
  
    //Enter the connection network mode when Pin7 is pressed.
    if (digitalRead(key_pin) == LOW) {
      delay(80);
      if (digitalRead(key_pin) == LOW) {
        my_device.mcu_set_wifi_mode(SMART_CONFIG);
      }
    }
    /* LED blinks when network is being connected */
    if ((my_device.mcu_get_wifi_work_state() != WIFI_LOW_POWER) && (my_device.mcu_get_wifi_work_state() != WIFI_CONN_CLOUD) && (my_device.mcu_get_wifi_work_state() != WIFI_SATE_UNKNOW)) {
      if (millis()- last_time >= 500) {
        last_time = millis();
  
        if (led_state == LOW) {
          led_state = HIGH;
        } else {
          led_state = LOW;
        }
        digitalWrite(LED_BUILTIN, led_state);
      }
    }
  
    delay(10);
  }
  
  /**
   * @description: DP download callback function.
   * @param {unsigned char} dpid
   * @param {const unsigned char} value
   * @param {unsigned short} length
   * @return {unsigned char}
   */
  unsigned char dp_process(unsigned char dpid,const unsigned char value[], unsigned short length)
  {
     Serial.print("dp_process switch dpid:");
     Serial.println(dpid);

     //Serial.print("dp_process  length:");
     //Serial.println(length);
   
     Serial.print("dp_process  value[1]:");
     Serial.println(value[1]);
   
    switch(dpid) {
      case DPID_SWITCH_LED:
        led_state = my_device.mcu_get_dp_download_data(dpid, value, length); // Get the value of the down DP command
        Serial.print("led_state:");
        Serial.println(led_state);
        
        if (led_state) {
          //Turn on
          //digitalWrite(LED_BUILTIN, HIGH);
          colorWipe(strip.Color(  255,  255, 255), 30); // Blue
        } else {
          //Turn off
          colorWipe(strip.Color(  0,   0, 0), 100); // Blue
        }
        
        //Status changes should be reported.
        my_device.mcu_dp_update(dpid, value, length);
        break;
  
      case DPID_WORK_MODE:  //模式  ( 枚举值:white, colour, scene, music )
            work_mode = my_device.mcu_get_dp_download_data(dpid, value, length); // Get the value of the down DP command
            Serial.print("work_mode:");
            Serial.println(work_mode);
            switch(work_mode) {
               case white:  //0
                 colorWipe(strip.Color(255,   0,   0), 50); // Red
                 theaterChaseRainbow(10);      // Rainbow-enhanced theaterChase variant
                 break;
               case colour: //1
                 theaterChase(strip.Color(255,   0,   0), 100); // Red, half brightness
                 break;
               case scene:  //2
                 rainbow(10);
                 break;
               case music:  //3
                 if (millis() - music_last_time >= 50)
                 {
                    music_last_time = millis();
                    visualize_music();
                 }
                 break;
               case rock:   //4
                 if (millis() - music_last_time >= 30)
                 {
                    music_last_time = millis();
                    visualize_music();
                 }
                 break;
               default:
                 theaterChase(strip.Color(255, 255, 255), 50); // White, half brightness
                 break;
            }
        
            //Status changes should be reported.
            my_device.mcu_dp_update(dpid, value, length);
         break;
  
      case DPID_BRIGHT_VALUE:   //亮度值 (value -> 数值范围：10-1000 )
        bright_value = my_device.mcu_get_dp_download_data(dpid, value, length); // Get the value of the down DP command
        my_device.mcu_dp_update(dpid, value, length);
        break;
  
      case DPID_COLOUR_DATA:   //彩光 HSV (String ->  000011112222)
        my_device.mcu_dp_update(dpid, value, length);
        break;
  
     case DPID_COUNTDOWN:  //倒计时
        colorWipe(strip.Color(100,  0,   0), 50); // Red
        my_device.mcu_dp_update(dpid, value, length);
        break;
       
      case DPID_MUSIC_DATA:  //音乐律动  
        Serial.print("DPID_MUSIC_DATA");
        my_device.mcu_dp_update(dpid, value, length);
        colour_data_control(value, length);
        break;
  
      case DPID_DREAMLIGHT_SCENE_MODE: //炫彩情景
        my_device.mcu_dp_update(dpid , value, length);
        //scene_mode = my_device.mcu_get_dp_download_data(dpid, value, length); // Get the value of the down DP command
        scene_mode=value[1];
        Serial.print("scene_mode:");
        Serial.println(scene_mode);
        
        switch(scene_mode){
           case 0:
              colorWipe(strip.Color(  0,   0,   0), 50);    // Black
              break;
            case 1:
              rainbow(10);
              break;
            case 2:
              theaterChase(strip.Color(127, 127, 127), 30); // White
              break;
            case 3:
              colorWipe(strip.Color(  0,   0, 255), 30);    // Blue
              break;
            case 4:
              theaterChaseRainbow(10);      // Rainbow-enhanced theaterChase variant
              break;
            case 5:
              theaterChase(strip.Color(127,   0,   0), 30); // Red
              break;
            case 6:
              theaterChase(strip.Color(  0,   0, 127), 30); // Blue
              break;
            case 7:
              rainbow(10);
              break;
            case 8:
              theaterChaseRainbow(50);
              break;
           }
         my_device.mcu_dp_update(dpid, value, length);
         break;
  
      case DPID_DREAMLIGHTMIC_MUSIC_DATA:
        visualize_music();
        my_device.mcu_dp_update(dpid, value, length);
        break;
  
      case DPID_LIGHTPIXEL_NUMBER_SET:  //长度设置
       
        my_device.mcu_dp_update(dpid, value, length);
        break;
      
      default:
        break;
    }
    
    
    return SUCCESS;
  }
  
  
  /**
   * @description: Upload all DP status of the current device.
   * @param {*}
   * @return {*}
   * 
   */
  void dp_update_all(void)
  {
    my_device.mcu_dp_update(DPID_SWITCH_LED, led_state, 1);
    my_device.mcu_dp_update(DPID_WORK_MODE, work_mode, 1);
  
    my_device.mcu_dp_update(DPID_BRIGHT_VALUE, bright_value, 1);
    my_device.mcu_dp_update(DPID_COLOUR_DATA, colour_data, 1);
    my_device.mcu_dp_update(DPID_MUSIC_DATA, music_data, 1);
    my_device.mcu_dp_update(DPID_DREAMLIGHT_SCENE_MODE, dreamlight_scene_mode, 1);
    my_device.mcu_dp_update(DPID_DREAMLIGHTMIC_MUSIC_DATA, dreamlightmic_music_data, 1);
    my_device.mcu_dp_update(DPID_LIGHTPIXEL_NUMBER_SET, lightpixel_number_set, 1);
  }
  
  
  // Fill strip pixels one after another with a color. Strip is NOT cleared
  // first; anything there will be covered pixel by pixel. Pass in color
  // (as a single 'packed' 32-bit value, which you can get by calling
  // strip.Color(red, green, blue) as shown in the loop() function above),
  // and a delay time (in milliseconds) between pixels.
  void colorWipe(uint32_t color, int wait) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
      strip.show();                          //  Update strip to match
      delay(wait);                           //  Pause for a moment
    }
  }
  
  // Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
  void rainbow(int wait) {
    // Hue of first pixel runs 5 complete loops through the color wheel.
    // Color wheel has a range of 65536 but it's OK if we roll over, so
    // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
    // means we'll make 5*65536/256 = 1280 passes through this outer loop:
    for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
      for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
        // Offset pixel hue by an amount to make one full revolution of the
        // color wheel (range of 65536) along the length of the strip
        // (strip.numPixels() steps):
        int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
        // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
        // optionally add saturation and value (brightness) (each 0 to 255).
        // Here we're using just the single-argument hue variant. The result
        // is passed through strip.gamma32() to provide 'truer' colors
        // before assigning to each pixel:
        strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
  
  // Theater-marquee-style chasing lights. Pass in a color (32-bit value,
  // a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
  // between frames.
  void theaterChase(uint32_t color, int wait) {
    for(int a=0; a<10; a++) {  // Repeat 10 times...
      for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
        strip.clear();         //   Set all pixels in RAM to 0 (off)
        // 'c' counts up from 'b' to end of strip in steps of 3...
        for(int c=b; c<strip.numPixels(); c += 3) {
          strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
        }
        strip.show(); // Update strip with new contents
        delay(wait);  // Pause for a moment
      }
    }
  }
  
  // Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
  void theaterChaseRainbow(int wait) {
    int firstPixelHue = 0;     // First pixel starts at red (hue 0)
    for(int a=0; a<30; a++) {  // Repeat 30 times...
      for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
        strip.clear();         //   Set all pixels in RAM to 0 (off)
        // 'c' counts up from 'b' to end of strip in increments of 3...
        for(int c=b; c<strip.numPixels(); c += 3) {
          // hue of pixel 'c' is offset by an amount to make one full
          // revolution of the color wheel (range 65536) along the length
          // of the strip (strip.numPixels() steps):
          int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
          uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
          strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
        }
        strip.show();                // Update strip with new contents
        delay(wait);                 // Pause for a moment
        firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
      }
    }
  }
  
   void colour_data_control( const unsigned char value[], u16 length)
   {
     u8 string_data[13];
      u16 h, s, v;
      u8 r, g, b;
      u16 hue;
      u8 sat,val;
  
      u32 c=0;
    
      string_data[0] = value[0]; //渐变、直接输出
      string_data[1] = value[1];
      string_data[2] = value[2];
      string_data[3] = value[3];
      string_data[4] = value[4];
      string_data[5] = value[5];
      string_data[6] = value[6];
      string_data[7] = value[7];
      string_data[8] = value[8];
      string_data[9] = value[9];
      string_data[10] = value[10];
      string_data[11] = value[11];
      string_data[12] = value[12];
    
      h = __str2short(__asc2hex(string_data[1]), __asc2hex(string_data[2]), __asc2hex(string_data[3]), __asc2hex(string_data[4]));
      s = __str2short(__asc2hex(string_data[5]), __asc2hex(string_data[6]), __asc2hex(string_data[7]), __asc2hex(string_data[8]));
      v = __str2short(__asc2hex(string_data[9]), __asc2hex(string_data[10]), __asc2hex(string_data[11]), __asc2hex(string_data[12]));
    
      hue=h*182;
      sat=s/4;
      val=v/4;
      c = strip.gamma32(strip.ColorHSV(hue,sat,val)); // hue -> RGB
    
      strip.fill(c,0,LED_COUNT);
      strip.show(); // Update strip with new contents
   }
   
  
  /**
   * @brief  str to short
   * @param[in] {a} Single Point
   * @param[in] {b} Single Point
   * @param[in] {c} Single Point
   * @param[in] {d} Single Point
   * @return Integrated value
   * @note   Null
   */
  u32 __str2short(u32 a, u32 b, u32 c, u32 d)
  {
      return (a << 12) | (b << 8) | (c << 4) | (d & 0xf);
  }

   
  
  /**
    * @brief ASCALL to Hex
    * @param[in] {asccode} 当前ASCALL值
    * @return Corresponding value
    * @retval None
    */
  u8 __asc2hex(u8 asccode)
  {
      u8 ret;
      
      if ('0' <= asccode && asccode <= '9')
          ret = asccode - '0';
      else if ('a' <= asccode && asccode <= 'f')
          ret = asccode - 'a' + 10;
      else if ('A' <= asccode && asccode <= 'F')
          ret = asccode - 'A' + 10;
      else
          ret = 0;
      
      return ret;
  }

  
  //Main function for visualizing the sounds in the lamp
  void visualize_music() {
    int sensor_value, mapped, avg, longavg;
    int light_led_count;
  
    //Actual sensor value
    sensor_value = analogRead(LED_PIN);
    Serial.print("sensor_value:");
    Serial.println(sensor_value);
  
    //点亮的LED个数
    light_led_count = sensor_value/100;
    
    if(light_led_count>LED_COUNT)
      light_led_count=LED_COUNT;
    
    //If 0, discard immediately. Probably not right and save CPU.
    if (sensor_value == 0)
      return;
      
    int R=random(0,255);
    int G=random(0,255);
    int B=random(0,255);
  
    for(int i=0;i<LED_COUNT;i++)
    {
        if(light_led_count==0)
        {
            strip.setPixelColor(i,strip.Color(0,0,0));
        }
        else
        {
            strip.setPixelColor((i-1),strip.Color(R,G,B));
        }
    }
    
    strip.show();
  }
