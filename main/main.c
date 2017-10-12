#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "driver/spi_master.h"
#include "driver/ledc.h"

#include <string.h>
#include <math.h>

#include "lena_std_320_240.h"


const gpio_num_t SPI_PIN_NUM_MISO = GPIO_NUM_19;
const gpio_num_t SPI_PIN_NUM_MOSI = GPIO_NUM_23;
const gpio_num_t SPI_PIN_NUM_CLK  = GPIO_NUM_18;

const gpio_num_t LCD_PIN_NUM_CS   = GPIO_NUM_5;
const gpio_num_t LCD_PIN_NUM_DC   = GPIO_NUM_21;
const gpio_num_t LCD_PIN_NUM_BCKL = GPIO_NUM_14;
const int LCD_BACKLIGHT_ON_VALUE = 1;
const int LCD_SPI_CLOCK_RATE = 40000000;

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_MH 0x04
#define TFT_RGB_BGR 0x08


static spi_transaction_t trans[6];
static spi_device_handle_t spi;

uint16_t FrameBuffer[320 * 240];

/*
 The ILI9341 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct {
    uint8_t cmd;
    uint8_t data[128];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} ili_init_cmd_t;

#define TFT_CMD_SWRESET	0x01

ili_init_cmd_t color_lut;

#if 1
// VCI=2.8V
//************* Reset LCD Driver ****************//
// LCD_nRESET = 1;
// delayms(1);       // Delay 1ms
// LCD_nRESET = 0;
// delayms(10);       // Delay 10ms // This delay time is necessary
// LCD_nRESET = 1;
// delayms(120);       // Delay 120 ms

DRAM_ATTR static const ili_init_cmd_t ili_init_cmds[]={
  {TFT_CMD_SWRESET, {0}, 0x80},

//************* Start Initial Sequence **********//
// LCD_ILI9341_CMD(0xCF);
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0xc3);
// LCD_ILI9341_ Parameter (0X30);
  {0xCF, {0x00, 0xc3, 0x30}, 3},

// LCD_ILI9341_CMD(0xED);
// LCD_ILI9341_ Parameter (0x64);
// LCD_ILI9341_ Parameter (0x03);
// LCD_ILI9341_ Parameter (0X12);
// LCD_ILI9341_ Parameter (0X81);
  {0xED, {0x64, 0x03, 0x12, 0x81}, 4},

// LCD_ILI9341_CMD(0xE8);
// LCD_ILI9341_ Parameter (0x85);
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0x78);
  {0xE8, {0x85, 0x00, 0x78}, 3},

// LCD_ILI9341_CMD(0xCB);
// LCD_ILI9341_ Parameter (0x39);
// LCD_ILI9341_ Parameter (0x2C);
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0x34);
// LCD_ILI9341_ Parameter (0x02);
  {0xCB, {0x39, 0x2c, 0x00, 0x34, 0x02}, 5},

// LCD_ILI9341_CMD(0xF7);
// LCD_ILI9341_ Parameter (0x20);
  {0xF7, {0x20}, 1},

// LCD_ILI9341_CMD(0xEA);
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0x00);
  {0xEA, {0x00, 0x00}, 2},

// LCD_ILI9341_CMD(0xC0);    //Power control
// LCD_ILI9341_ Parameter (0x1B);   //VRH[5:0]
  {0xC0, {0x1B}, 1},

// LCD_ILI9341_CMD(0xC1);    //Power control
// LCD_ILI9341_ Parameter (0x12);   //SAP[2:0];BT[3:0]
  {0xC1, {0x12}, 1},

// LCD_ILI9341_CMD(0xC5);    //VCM control
// LCD_ILI9341_ Parameter (0x32);
// LCD_ILI9341_ Parameter (0x3C);
  {0xC5, {0x32, 0x3C}, 2},

// LCD_ILI9341_CMD(0xC7);    //VCM control2
// LCD_ILI9341_ Parameter (0X91);
  {0xC7, {0x91}, 1},

// LCD_ILI9341_CMD(0x36);    // Memory Access Control
// LCD_ILI9341_ Parameter (0x08);
  //{0x36, {0x08}, 1},
  {0x36, {(MADCTL_MV | MADCTL_MX | TFT_RGB_BGR)}, 1},

// LCD_ILI9341_CMD(0x3A);
// LCD_ILI9341_ Parameter (0x55);
  {0x3A, {0x55}, 1},

// LCD_ILI9341_CMD(0xB1);
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0x1B);
  {0xB1, {0x00, 0x1B}, 2},

// LCD_ILI9341_CMD(0xB6);    // Display Function Control
// LCD_ILI9341_ Parameter (0x0A);
// LCD_ILI9341_ Parameter (0xA2);
  {0xB6, {0x0A, 0xA2}, 2},

// LCD_ILI9341_CMD(0xF6);
// LCD_ILI9341_ Parameter (0x01);
// LCD_ILI9341_ Parameter (0x30);
  {0xF6, {0x01, 0x30}, 2},

// LCD_ILI9341_CMD(0xF2);    // 3Gamma Function Disable
// LCD_ILI9341_ Parameter (0x00);
  {0xF2, {0x00}, 1},

// LCD_ILI9341_CMD(0x26);    //Gamma curve selected
// LCD_ILI9341_ Parameter (0x01);
  {0x26, {0x01}, 1},

#if 0
// LCD_ILI9341_CMD(0xE0);    //Set Gamma
// LCD_ILI9341_ Parameter (0x0F);
// LCD_ILI9341_ Parameter (0x1d);
// LCD_ILI9341_ Parameter (0x1a);
// LCD_ILI9341_ Parameter (0x0a);
// LCD_ILI9341_ Parameter (0x0d);
// LCD_ILI9341_ Parameter (0x07);
// LCD_ILI9341_ Parameter (0x49);
// LCD_ILI9341_ Parameter (0X66);
// LCD_ILI9341_ Parameter (0x3b);
// LCD_ILI9341_ Parameter (0x07);
// LCD_ILI9341_ Parameter (0x11);
// LCD_ILI9341_ Parameter (0x01);
// LCD_ILI9341_ Parameter (0x09);
// LCD_ILI9341_ Parameter (0x05);
// LCD_ILI9341_ Parameter (0x04);
  {0xE0, {0x0f, 0x1d, 0x1a, 0x0a, 0x0d, 0x07, 0x49, 0x66, 0x3b, 0x07, 0x11, 0x01, 0x09, 0x05, 0x04}, 15},

// LCD_ILI9341_CMD(0XE1);    //Set Gamma
// LCD_ILI9341_ Parameter (0x00);
// LCD_ILI9341_ Parameter (0x18);
// LCD_ILI9341_ Parameter (0x1d);
// LCD_ILI9341_ Parameter (0x02);
// LCD_ILI9341_ Parameter (0x0f);
// LCD_ILI9341_ Parameter (0x04);
// LCD_ILI9341_ Parameter (0x36);
// LCD_ILI9341_ Parameter (0x13);
// LCD_ILI9341_ Parameter (0x4c);
// LCD_ILI9341_ Parameter (0x07);
// LCD_ILI9341_ Parameter (0x13);
// LCD_ILI9341_ Parameter (0x0f);
// LCD_ILI9341_ Parameter (0x2E);
// LCD_ILI9341_ Parameter (0x2f);
// LCD_ILI9341_ Parameter (0x05);
  {0xE1, {0x00, 0x18, 0x1d, 0x02, 0x0f, 0x04, 0x36, 0x13, 0x4c, 0x07, 0x13, 0x0f, 0x2e, 0x2f, 0x05}, 15},
#else
  {0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
  {0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},
#endif

//LCD_ILI9341_CMD(0x11);    //Exit Sleep
//Delayms(120);
  {0x11, {0}, 0x80},

//LCD_ILI9341_CMD(0x29);    //Display on
  {0x29, {0}, 0x80},

  {0, {0}, 0xff}
};

#else
// Note: New Gamma curve from https://github.com/gnulabis/UTFT-ESP8266/blob/master/UTFT/tft_drivers/ili9341/16/initlcd.h
DRAM_ATTR static const ili_init_cmd_t ili_init_cmds[]={
    {TFT_CMD_SWRESET, {0}, 0x80},
    {0xCF, {0x00, 0x83, 0X30}, 3},
    {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    //{0x36, {0x28}, 1},  // MADCTL (Memory Access Control)
    {0x36, {(MADCTL_MV | TFT_RGB_BGR)}, 1},

#if 1
    {0x3A, {0x55}, 1},  // 16bit color
#else
    {0x3A, {0x66}, 1},  // 18bit color
#endif

    {0xB1, {0x00, 0x1F}, 2},  // Frame Rate Control (1B=70, 1F=61, 10=119)
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},

#if 0
    // Original
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
#else
    // new
    {0xE0, {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15},
    {0XE1, {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15},
#endif

    {0xF6, {0x01, 0x00, 0x10}, 3},

    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},

    {0, {0}, 0xff},
};
#endif


//Send a command to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
static void ili_cmd(spi_device_handle_t spi, const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Send data to the ILI9341. Uses spi_device_transmit, which waits until the transfer is complete.
static void ili_data(spi_device_handle_t spi, const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void ili_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc=(int)t->user;
    gpio_set_level(LCD_PIN_NUM_DC, dc);
}

//Initialize the display
static void ili_init()
{
    int cmd=0;
    //Initialize non-SPI GPIOs
    gpio_set_direction(LCD_PIN_NUM_DC, GPIO_MODE_OUTPUT);

    //Disable backlight
    gpio_set_direction(LCD_PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_PIN_NUM_BCKL, LCD_BACKLIGHT_ON_VALUE ? 0 : 1);

    //Send all the commands
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili_cmd(spi, ili_init_cmds[cmd].cmd);
        ili_data(spi, ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes&0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

#if 1
    ili_cmd(spi, color_lut.cmd);
    ili_data(spi, color_lut.data, 128);
#endif
    //gpio_set_level(LCD_PIN_NUM_BCKL, LCD_BACKLIGHT_ON_VALUE);
}

static void send_reset_drawing(int left, int top, int width, int height)
{
  esp_err_t ret;

  trans[0].tx_data[0]=0x2A;           //Column Address Set
  trans[1].tx_data[0]=(left) >> 8;              //Start Col High
  trans[1].tx_data[1]=(left) & 0xff;              //Start Col Low
  trans[1].tx_data[2]=(left + width - 1) >> 8;       //End Col High
  trans[1].tx_data[3]=(left + width - 1) & 0xff;     //End Col Low
  trans[2].tx_data[0]=0x2B;           //Page address set
  trans[3].tx_data[0]=top >> 8;        //Start page high
  trans[3].tx_data[1]=top & 0xff;      //start page low
  trans[3].tx_data[2]=(top + height - 1)>>8;    //end page high
  trans[3].tx_data[3]=(top + height - 1)&0xff;  //end page low
  trans[4].tx_data[0]=0x2C;           //memory write

  //Queue all transactions.
  for (int x = 0; x < 5; x++) {
      ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
      assert(ret==ESP_OK);
  }

  // Wait for all transactions
  spi_transaction_t *rtrans;
  for (int x = 0; x < 5; x++) {
      ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
      assert(ret==ESP_OK);
  }
}

static void send_continue_line(uint16_t *line, int width, int lineCount)
{
  esp_err_t ret;

  trans[4].tx_data[0]=0x3C;           //memory write continue

  trans[5].tx_buffer=line;            //finally send the line data
  trans[5].length= width * lineCount * 2 * 8;            //Data length, in bits
  trans[5].flags=0; //undo SPI_TRANS_USE_TXDATA flag

  //Queue all transactions.
  for (int x = 4; x < 6; x++) {
      ret=spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
      assert(ret==ESP_OK);
  }

  // Wait for all transactions
  spi_transaction_t *rtrans;
  for (int x = 0; x < 2; x++) {
      ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
      assert(ret==ESP_OK);
  }
}

void ili9341_write_frame()
{
    int x, y;
    uint16_t* framePtr = FrameBuffer;

    send_reset_drawing(0, 0, 320, 240);

    for (y = 0; y < 240; y++)
    {
      send_continue_line(framePtr, 320, 1);
      framePtr += 320;
    }
}

static void backlight_init()
{
  // (duty range is 0 ~ ((2**bit_num)-1)
  const int DUTY_MAX = 0x1fff;

  //configure timer0
  ledc_timer_config_t ledc_timer;
	memset(&ledc_timer, 0, sizeof(ledc_timer));

  ledc_timer.bit_num = LEDC_TIMER_13_BIT; //set timer counter bit number
  ledc_timer.freq_hz = 5000;              //set frequency of pwm
  ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;   //timer mode,
  ledc_timer.timer_num = LEDC_TIMER_0;    //timer index


  ledc_timer_config(&ledc_timer);


  //set the configuration
  ledc_channel_config_t ledc_channel;
  memset(&ledc_channel, 0, sizeof(ledc_channel));

  //set LEDC channel 0
  ledc_channel.channel = LEDC_CHANNEL_0;
  //set the duty for initialization.(duty range is 0 ~ ((2**bit_num)-1)
  ledc_channel.duty = (LCD_BACKLIGHT_ON_VALUE) ? 0 : DUTY_MAX;
  //GPIO number
  ledc_channel.gpio_num = LCD_PIN_NUM_BCKL;
  //GPIO INTR TYPE, as an example, we enable fade_end interrupt here.
  ledc_channel.intr_type = LEDC_INTR_FADE_END;
  //set LEDC mode, from ledc_mode_t
  ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
  //set LEDC timer source, if different channel use one timer,
  //the frequency and bit_num of these channels should be the same
  ledc_channel.timer_sel = LEDC_TIMER_0;


  ledc_channel_config(&ledc_channel);


  //initialize fade service.
  ledc_fade_func_install(0);

  // duty range is 0 ~ ((2**bit_num)-1)
  ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (LCD_BACKLIGHT_ON_VALUE) ? DUTY_MAX : 0, 500);
  ledc_fade_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, LEDC_FADE_NO_WAIT);
}

void ili9341_init()
{
	// Initialize transactions
    for (int x=0; x<6; x++) {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x&1)==0) {
            //Even transfers are commands
            trans[x].length=8;
            trans[x].user=(void*)0;
        } else {
            //Odd transfers are data
            trans[x].length=8*4;
            trans[x].user=(void*)1;
        }
        trans[x].flags=SPI_TRANS_USE_TXDATA;
    }

    // Initialize SPI
    esp_err_t ret;
    //spi_device_handle_t spi;
    spi_bus_config_t buscfg;
		memset(&buscfg, 0, sizeof(buscfg));

    buscfg.miso_io_num = SPI_PIN_NUM_MISO;
    buscfg.mosi_io_num = SPI_PIN_NUM_MOSI;
    buscfg.sclk_io_num = SPI_PIN_NUM_CLK;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;

    spi_device_interface_config_t devcfg;
		memset(&devcfg, 0, sizeof(devcfg));

    devcfg.clock_speed_hz = LCD_SPI_CLOCK_RATE;
    devcfg.mode = 0;                                //SPI mode 0
    devcfg.spics_io_num = LCD_PIN_NUM_CS;               //CS pin
    devcfg.queue_size = 7;                          //We want to be able to queue 7 transactions at a time
    devcfg.pre_cb = ili_spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
    devcfg.flags = 0; //SPI_DEVICE_HALFDUPLEX;

    //Initialize the SPI bus
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);

    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);

    //Initialize the LCD
    ili_init();
    backlight_init();

    printf("LCD Initialized (%d Hz).\n", LCD_SPI_CLOCK_RATE);
}

void generate_gradient()
{
  for (int y = 0; y < 240; ++y)
  {
    for (int x = 0; x < 320; ++x)
    {
      int l = x / 5;

      uint16_t r = ((l >> 1) << 11);
      uint16_t g = l << 5;
      uint16_t b = (l >> 1);

      uint16_t c = r | g | b;


      uint16_t mask;
      switch (y / 60)
      {
        case 0:
          mask = 0b1111100000000000;
          break;
        case 1:
          mask = 0b0000011111100000;
          break;
        case 2:
          mask = 0b0000000000011111;
          break;
        case 3:
        default:
          mask = 0xffff;
          break;
      }

      c &= mask;

      c = (c >> 8) | ((c << 8) & 0xff00);
      FrameBuffer[y * 320 + x] = c;
    }
  }
}

void generate_solid_color(uint8_t red, uint8_t green, uint8_t blue)
{
  for (int y = 0; y < 240; ++y)
  {
    for (int x = 0; x < 320; ++x)
    {
      uint16_t r = ((red >> 3) << 11);
      uint16_t g = (green >> 2) << 5;
      uint16_t b = (blue >> 3);

      uint16_t c = r | g | b;

      c = (c >> 8) | ((c << 8) & 0xff00);
      FrameBuffer[y * 320 + x] = c;
    }
  }
}

static void HSVToRGB(float H, float S, float V, float* R, float* G, float* B)
{
    if (H == 1.0f)
    {
        H = 0.0f;
    }

    float step = 1.0f / 6.0f;
    float vh = H / step;

    int i = (int)floorf(vh);

    float f = vh - i;
    float p = V * (1.0f - S);
    float q = V * (1.0f - (S * f));
    float t = V * (1.0f - (S * (1.0f - f)));

    switch (i)
    {
        case 0:
            {
                *R = V;
                *G = t;
                *B = p;
                break;
            }
        case 1:
            {
                *R = q;
                *G = V;
                *B = p;
                break;
            }
        case 2:
            {
                *R = p;
                *G = V;
                *B = t;
                break;
            }
        case 3:
            {
                *R = p;
                *G = q;
                *B = V;
                break;
            }
        case 4:
            {
                *R = t;
                *G = p;
                *B = V;
                break;
            }
        case 5:
            {
                *R = V;
                *G = p;
                *B = q;
                break;
            }
        default:
            {
                // not possible - if we get here it is an internal error
            }
    }
}

typedef struct
{
  int X;
  int Y;
} Point;

// Based on http://viziblr.com/news/2011/12/1/drawing-a-color-hue-wheel-with-c.html
void generate_color_wheel()
{
  int inner_radius = 40;
  int outer_radius = 120;

  int bmp_width = 320;
  int bmp_height = 240;

  Point center;
  center.X = bmp_width / 2;
  center.Y = bmp_height / 2;

  // Clear screen to black
  for (int i = 0; i < 240 * 320; ++i)
    FrameBuffer[i] = 0x0000;

  // Render
  for (int y = 0; y < bmp_height; y++)
  {
      int dy = (center.Y - y);

      for (int x = 0; x < bmp_width; x++)
      {
          int dx = (center.X - x);

          float dist = sqrtf(dx * dx + dy * dy);

          if (dist >= inner_radius && dist <= outer_radius)
          {
              float theta = atan2f(dy, dx);
              // theta can go from -pi to pi

              float hue = (theta + M_PI) / (2 * M_PI);

              float dr, dg, db;
              const float sat = 1.0;
              const float val = 1.0;
              HSVToRGB(hue, sat, val, &dr, &dg, &db);

              uint16_t c = ((int)(dr * 255) >> 3) << 11;
              c |= ((int)(dg * 255) >> 2) << 5;
              c |= ((int)(db * 255) >> 3);

              // byte swap
              c = (c >> 8) | ((c << 8) & 0xff00);

              FrameBuffer[y * 320 + x] = c;
          }
      }
  }
}

void generate_image()
{
  uint8_t* ptr = gimp_image.pixel_data;

  for (int y = 0; y < 240; ++y)
  {
    for (int x = 0; x < 320; ++x)
    {
      uint16_t r = ((*ptr++ >> 3) << 11);
      uint16_t g = (*ptr++ >> 2) << 5;
      uint16_t b = (*ptr++ >> 3);

      uint16_t c = r | g | b;

      c = (c >> 8) | ((c << 8) & 0xff00);
      FrameBuffer[y * 320 + x] = c;
    }
  }
}

void app_main(void)
{
  #if 1
  const bool colorCorrect = true;

  color_lut.cmd = 0x2d;
  for (int i = 0; i < 32; ++i)
  {
    color_lut.data[i] = (i << 1) | 1;
  }

  for (int i = 0; i < 64; ++i)
  {
    if (colorCorrect)
    {
      int val = i * (58.0 / 64.0);
      color_lut.data[i + 32] = val < 0 ? 0 : val;
    }
    else
    {
      color_lut.data[i + 32] = i;
    }
  }

  for (int i = 0; i < 32; ++i)
  {
    if (colorCorrect)
    {
      int val = i * (29.0 / 32.0);
      color_lut.data[i + 32 + 64] = (val < 0 ? 0 : val) << 1;
    }
    else
    {
      color_lut.data[i + 32 + 64] = (i << 1) | 1;
    }
  }

  printf("LUT: ");
  for (int i = 0; i < 128; ++i)
  {
    printf("0x%.2x", color_lut.data[i]);
    if (i != 127)
      printf(", ");
    else
      printf("\n");
  }
  #endif

    nvs_flash_init();
    ili9341_init();

    gpio_set_direction(0, GPIO_MODE_INPUT);

    //generate_gradient();
    generate_color_wheel();

    int test = 0;
    int lastButtonState = gpio_get_level(0);

    while (true)
    {
        ili9341_write_frame();

        int button = gpio_get_level(0);
        if (button == 0 && lastButtonState == 1)
        {
          ++test;

          switch (test % 7)
          {
            case 0:
              generate_color_wheel();
              break;

            case 1:
              generate_gradient();
              break;

            case 2:
              generate_solid_color(0xff, 0x00, 0x00);
              break;

            case 3:
              generate_solid_color(0x00, 0xff, 0x00);
              break;

            case 4:
              generate_solid_color(0x00, 0x00, 0xff);
              break;

            case 5:
              generate_solid_color(0xff, 0xff, 0xff);
              break;

            case 6:
              generate_image();
              break;
          }
        }

        lastButtonState = button;

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
