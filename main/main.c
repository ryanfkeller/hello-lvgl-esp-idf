


/*********************************************************************
 * ABOUT THIS PROJECT
 *********************************************************************/
// I am working to create a bluetooth audio player to replace my phone
// for podcast and music listening, both as an experiment in product
// development and as a way to enjoy audio content without the constant
// distractions from my phone. 

// I decided to make this project using an ESP32 chip due to:
// a) already having a cheap dev-board to play with
// b) easy integration of bluetooth and wifi

// As part of building up this project, I needed to learn how to drive
// my chosen LCD display. The absolute easiest way to create a decent
// UI without a major headache seemed to be using LVGL, and Espressif
// provided several demos connecting with this lib. However, I found that 
// I still had trouble understanding what was going on and had to really
// dig to figure out how to get this to work.

// What I ultimately created was an extremely simple menu system that
// is based COMPLETELY on the Espressif spi_lcd_touch_example_main.c

// I figured as a community service project (lol) I would create this
// simple, exhaustively documented single-file application that walks 
// through setting up and starting an LVGL-based GUI application using 
// only ESP-IDF and LVGL drivers. 

// My system:
// -- Host:     An ESP32-WROOM-32D dev-board (cheap board from HiLetGo)
// -- Display:  Waveshare 2" (240x320) LCD Module w/ ST7789V controller
// -- Input:    Keyboard via Serial Interface

/*********************************************************************
 * Dependency Includes
 *********************************************************************/

// "Standard" libs we use for usleep, lock, and MAX/MIN (cuz we're lazy)
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

// Include for FreeRTOS (the OS our app runs on)
#include "freertos/FreeRTOS.h"

// ESP tools/device drivers we need
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"   
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/uart.h"

// LVGL dependency
#include "lvgl.h"

/*********************************************************************
 * LCD DEFINES
 *********************************************************************/

// The ESP32-WROOM-32D which provides 4 SPI peripherals. SPI 3 and 4 
// (HSPI and VSPI respectively) are general purpose SPI that we assign
// to any GPIO-capable pins and use for external communication with 
// things like LCD displays and SD cards. 
// 
// We are going to arbitrarily choose HSPI (SPI3)
#define LCD_HOST                SPI3_HOST

// We communicate over SPI with the LCD controller ST7789V. The 
// datasheet for this part gives us information about how we need
// to set up our SPI interface.
// 
// The maximum SPI frequency of the chip is ~62.5MHz, BUT our wiring 
// situation is not ideal and frankly who needs to go that fast. 
// We're using a nice, commonly used value of 20M. 
#define LCD_PIXEL_CLK_HZ        (20*1000*1000)

// ST7789V datasheet gives the info about SPI mode, CMD bits, and param bits.
#define LCD_CMD_BITS            8
#define LCD_PARAM_BITS          8
#define LCD_SPI_MODE            0

// The backlight control doesn't go through the ST7789 -- it's just 
// a GPIO, and we need to define which value is ON vs OFF 
#define LCD_BK_LIGHT_ON_LVL     1
#define LCD_BK_LIGHT_OFF_LVL    !LCD_BK_LIGHT_ON_LVL

// Here we are just defining the GPIO we used for each pin. 
// It looks pretty random, but these are actually next to each
// other on my dev board
#define LCD_PIN_NUM_SCLK        18
#define LCD_PIN_NUM_MOSI        19
#define LCD_PIN_NUM_DC          17
#define LCD_PIN_NUM_RST         16
#define LCD_PIN_NUM_CS          5
#define LCD_PIN_NUM_BK_LIGHT    4

// This is the number of SPI transactions that we can queue up. 
// Basically, SPI only sends out data so fast, and if a thread
// needs to send something and the SPI bus is busy, the thread
// has to wait until there is space which stalls our app. 
// HOWEVER a bigger queue means more memory usage AND possibly
// worse latency in the interactions. 
//
// Anyways I chose 10 because that's what the Espressif demo
// used. This is a number to keep an eye on though. 
#define LCD_TRANS_QUEUE_DEPTH   10

// The below is the maximum number of "display" lines we will
// allow in a single SPI transaction. We're never
// Going to hit this number (as described below).
#define LCD_MAX_LINES_PER_TXN   80

// The below is our screen resolution
#define LCD_H_RES               240
#define LCD_V_RES               320

/*********************************************************************
 * LVGL DEFINES
 *********************************************************************/

// The below is our buffer draw size. what this corresponds to 
// is essentially how much of the display is rendered by LVGL 
// before being flushed to the display. The larger the buffer,
// the faster the screen can be updated -- i.e., animations and
// UI changes look smoother.
//
// Ideally, this buffer would just be the size of the screen. 
// HOWEVER obviously that would take a lot of RAM, and we're
// trying to make an embedded project for crying out loud.
// In addition, we need two of these buffers so we can be
// populating one while the other is writing. In the Espressif
// demo, they recommend to make the buffer at least 10% of 
// your screen size as a reasonable memory/performance trade.
// (But they actually assign less than 10%...) 
// 
// We're going to go ahead and choose 10%, which would be 
// our Vertical Resolution / 10.
#define LVGL_DRAW_BUF_LINES     32

// The below is the tick period, i.e., at what resolution LVGL 
// is able to track time. This affects smoothness of animations,
// etc. Everything is a trade in performance
// vs memory vs power, etc.. 2ms is what was used in the demo,
// so 2ms is what we use here. If we notice lag, stuttering,
// etc., this is a knob to turn down.. OR if we're running super hot
// or other tasks are getting starved out, we could increase
// this number.
#define LVGL_TICK_PERIOD_MS     2

// The min and max task delay are limits on how frequently
// (or infrequently) LVGL can check to see whether it
// has something to do in its main task loop. LVGL will
// generally try to keep track of when it needs to perform
// its next task and let us know, but in order to ensure
// smoothness we can set a maximum delay (just in case
// something happens before LVGL's specified time). The
// minimum is set to exactly 1 FreeRTOS tick so that
// we always allow other tasks to get a chance to execute
// between LVGL task calls.
#define LVGL_TASK_MAX_DELAY_MS  500
#define LVGL_TASK_MIN_DELAY_MS  1000 / CONFIG_FREERTOS_HZ

// This is the amount of stack we're going to let LVGL use. 
// LVGL recommends >8kB. Here we're going with 10.
#define LVGL_TASK_STACK_SIZE    (10 * 1024)

// Generally LVGL should be given a fairly low task priority
// since in an real time embedded system, a choppy display
// is annoying but a late response to real time stimuli
// could be a device failure. 
#define LVGL_TASK_PRIORITY      2


/*********************************************************************
 * UART DEFINES
 *********************************************************************/

// I use UART as a way to provide input into my system. Generally this
// is a pretty standard and uninteresting configuration.
#define UART_PORT_NUM   UART_NUM_0
#define UART_BAUD_RATE  115200
#define UART_DATA_BITS  UART_DATA_8_BITS
#define UART_PARITY     UART_PARITY_DISABLE
#define UART_STOP_BITS  UART_STOP_BITS_1
#define UART_FLOW_CTRL  UART_HW_FLOWCTRL_DISABLE
#define UART_SOURCE_CLK UART_SCLK_DEFAULT

// We aren't going to be sending anything back, so no need for a TX buffer!
#define UART_RX_BUF_SIZE 256
#define UART_TX_BUF_SIZE 0


/*********************************************************************
 * STATIC SCOPE VARIABLE DEFINES
 *********************************************************************/

// ESP logging tag. Boring name
static const char *TAG = "MAIN";

// Lock mutex for our LVGL usage. LVGL is *not thread-safe* and we have 
// a few different threads that hit it's API, so we use the lock to make
// sure that each thread is able to finish it's operation before another
// thread starts one.
static _lock_t lvgl_api_lock;


/*********************************************************************
 * CALLBACK FUNCTIONS (AKA the real work begins)
 *********************************************************************/

// This is a callback function for our ESP LCD driver that we will 
// register to be called when the panel IO has finished transfering
// color data. We take this opportunity to let LVGL know that the 
// display flush has completed so that LVGL can do whatever it
// needs to do next...
static bool notify_lvgl_flush_ready(
    esp_lcd_panel_io_handle_t panel_io,
    esp_lcd_panel_io_event_data_t *edata,
    void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

// This is a callback function that is called by our timer that we
// set up to go off every {LVGL_TICK_PERIOD_MS}. We take this
// opportunity to let LVGL know what time it is so it can push
// forward any animations, time-based processes, etc. How nice 
// are we for doing that?
static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

// This callback function is used by LVGL when rendering is complete and the display
// buffer content is ready to be pushed to the display. 
// Display points to our LCD panel, area defines where the image will go, and
// px_map shows what the image is. This is the function that actually writes stuff
// to our LCD display. LVGL, for all its complexity, is basically just there
// to prepare buffers for us to push to the LCD ourselves.
static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    // Get the LCD handle and pixel offset
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);
    int offsetx1 = area->x1;
    int offsetx2 = area->x2; 
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // because SPI LCD is big-endian, we need to swap the RGB bytes order
    lv_draw_sw_rgb565_swap(px_map, (offsetx2 + 1 - offsetx1) * (offsety2 + 1 - offsety1));

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, px_map);
}

/*********************************************************************
 * TASKS
 *********************************************************************/

// This is the task that is responsible for launching (and relaunching,
// and relaunching...) lv_timer_handler() so that LVGL will wake up
// and do something. This task uses our mutex to make sure that it has
// exclusive access to LVGL while its using it. 
//
// We talked above about how we want to make sure that this is called with
// the correct frequency -- you can see in the logic how LVGL tells us
// when it wants to be called next, but we put some guards on it. 
// 
// We're also using usleep here to have more fine grained control over
// when this gets called again. FreeRTOS has the frankly amazing feature
// that usleep will release the task if the sleep duration is greater
// than an a FreeRTOS tick, and will busy-wait if not... so we can
// use it in comfort.
static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1) {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a WDT
        time_till_next_ms = MAX(time_till_next_ms, LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, LVGL_TASK_MAX_DELAY_MS);
        usleep(1000* time_till_next_ms);
    }
}

/*********************************************************************
 * Hardware Setup
 *********************************************************************/

// This is boilerplate UART and GPIO initialization for ESP32. We also set
// up SPI here with our LCD-specific interface settings, and allow SPI
// to use the DMA channel. We want to do that so that the CPU doesn't
// need to hand the SPI driver all the data itself.
static void hw_init()
{
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_BITS,
        .parity = UART_PARITY,
        .stop_bits = UART_STOP_BITS,
        .flow_ctrl = UART_FLOW_CTRL,
        .source_clk = UART_SOURCE_CLK,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, 0 ));

    ESP_LOGI(TAG, "UART initialized");

    // Configure GPIO for backlight
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BK_LIGHT, // GPIO pin 4
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    ESP_LOGI(TAG, "GPIO initialized");

    // Configure the SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_PIN_NUM_SCLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES*LCD_MAX_LINES_PER_TXN*sizeof(uint16_t), //transfer 80 lines of pixels (RGB565) at most in one SPI txn
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI initialized");
}


/*********************************************************************
 * LVGL Input Device Callback Definition
 *********************************************************************/

// We need to set up an input callback function for our serial UART input
// so that the things we type are actually registered! This is the function
// format it needs to take. We don't actually use indev_driver, because 
// we really only have one input device so there's no fuss there.
//
// The actual code here is pretty straightforward UART processing once
// you get past the LVGL weirdness.
static void uart_indev_read_cb(lv_indev_t *indev_driver, lv_indev_data_t *data)
{
    // Lets see if data is available
    size_t len;
    uart_get_buffered_data_len(UART_PORT_NUM, &len);

    if (len > 0) {
        // Great, data is available. Lets read the data and store it
        uint8_t buf;
        int read_len = uart_read_bytes(UART_PORT_NUM, &buf, 1, 10); // read just one byte

        if (read_len > 0) {
            // Whew, we read one byte. Lets see what it was and map it to the appropriate
            // LV key for the action
            // Also, if its a valid key, let LVGL know that we pressed a key. 
            switch (buf) {
                case 'w': case 'W':
                    ESP_LOGI(TAG, "w");
                    data->state = LV_INDEV_STATE_PRESSED;
                    data->key = LV_KEY_PREV;
                    break;
                case 's': case 'S':
                    ESP_LOGI(TAG, "s");
                    data->state = LV_INDEV_STATE_PRESSED;
                    data->key = LV_KEY_NEXT;
                    break;
                case '\r': case '\n':
                    ESP_LOGI(TAG, "enter");
                    data->state = LV_INDEV_STATE_PRESSED;
                    data->key = LV_KEY_ENTER;
                    break;
                case 27:
                    ESP_LOGI(TAG, "esc");
                    data->state = LV_INDEV_STATE_PRESSED;
                    data->key = LV_KEY_ESC;
                    break;
                default:
                    break;
            }
                        
            // Set flag to tell LVGL there is more to read so
            // we can call this function some more.
            data->continue_reading = len > read_len;
            return;
        }
    }

    // Nothing left to read! Now this function can finally rest.
    data->state = LV_INDEV_STATE_RELEASED;
    return;
}

/*********************************************************************
 * MAIN APPLICATION -- Lets finally do something will all of that
 *********************************************************************/

void app_main(void)
{
    ESP_LOGI(TAG, "*** Hello LVGL ESP-IDF Main Starting ***");

    ESP_LOGI(TAG, "Initializing Hardware");
    hw_init();

    ESP_LOGI(TAG, "Turn on LCD backlight");
    // We aren't really prepared to display anything yet, but it's nice
    // for the user to see a sign of life from the LCD/device when you
    // start it... so the backlight comes on!. 
    gpio_set_level(LCD_PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LVL);

    ESP_LOGI(TAG, "Install LCD panel IO");
    // Now we need to set up the interface to the LCD. 
    // We do this by passing all of our carefully defined variables
    // into a spi_config struct, then create a new panel IO interface
    // with that config that we can reference with "io_handle". Nice.
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = LCD_SPI_MODE,
        .trans_queue_depth = LCD_TRANS_QUEUE_DEPTH,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    // There are a coupple of config things that are not strictly SPI config related
    // i.e., which GPIO is tied to RESET, the order of our RGB data, and how many
    // bits per pixel. We define those in our struct, then pass both the IO handle
    // and our panel_config into the (thankfully!) available setup function for
    // our exact LCD driver, and are given a panel_handle that we can use to 
    // interface with our LCD! 
    // BY THE WAY -- setting the bits per pixel to 16 means the driver will be 
    // set up to use RGB565 format. 18 would be RGB666. You can check the driver on that.
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // Now that we have our LCD panel handle, lets do some housekeeping. First
    // we reset the panel and initialize it
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    // I discovered that unlink the Espressif demo, my LCD panel
    // Needed the pixel order to be RGB *and* that I needed to 
    // invert the color to get accurate color representation on my display. 
    // So... keep an eye on those two places if you have trouble!
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // Then we set the panel orientation. Basically I want the panel
    // To be rotated 180 degrees, so we double flip it.
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));

    // We actually don't want the display on yet because we have nothing 
    // to show. This would be a great opportunity to show a boot screen...
    // Maybe later :) 
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, false));

    ESP_LOGI(TAG, "Initalize LVGL library");
    // Now we need to set up LVGL lib, define our menu, and hook it all
    // up to our LCD handle

    // Start by initilializing the library.
    lv_init();
    
    // Now we create an LVGL "display" that represents our LCD display
    // as a target for LVGL rendering 
    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);

    // As we mentioned, LVGL needs buffers for its render output
    // We have a ping/pong setup here where one buffer can be used by the LCD
    // driver to drive output while the other buffer is being prepared by
    // LVGL. 
    size_t draw_buffer_sz = LCD_H_RES * LVGL_DRAW_BUF_LINES * sizeof(lv_color16_t);

    // Since these buffers are used by the ESP SPI interface, it's important
    // that we allocate this memory with the "spi specific" call so that 
    // the region allocated is accessible to the SPI HW.
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf2);

    // Now we've allocated those buffers, we hand them over to LVGL.
    // Since the buffers are NOT the full size of the screen, we have to
    // set them up with "PARTIAL". If we decide to make the buffers the
    // full size of the display, we could go with "DIRECT" (or "FULL" if we
    // want to force a full screen redraw on a single pixel change)
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_PARTIAL);
    // One could even set up a third buffer to reduce handoff idle time...
    // If you're truely not resource constrained, why not. But we are.

    // This is where we actually hand the LCD panel handle control over 
    // to LVGL to use. LVGL now will associate the panel_handle with
    // the LVGL display we created
    lv_display_set_user_data(display, panel_handle);

    // We need to let LVGL know the format of our pixel data! As mentioned,
    // we've set up our LCD driver to use RGB565. 
    lv_display_set_color_format(display, LV_COLOR_FORMAT_RGB565);

    // Now we get to associate the callback function we defined above -- again,
    // this is the function that goes off when LVGL rendering is complete and
    // the buffers are ready to be pushed to the LCD display
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // We talked about how LVGL needs some way to be able to keep track of
    // elapsed time. We create this ESP timer, and set it up with our LVGL
    // callback function so that we call increase_lvgl_tick every LVGL_TICK_PERIOD_MS.
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    
    ESP_LOGI(TAG, "Register io panel event callback for LVGL flush ready notification");
    // Here, we are setting back the callback function for when the LCD panel is done flushing
    // the data! We described the function above, but basically we just need to let LVGL know
    // that this step has been completed so it can switch display buffers and keep going
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = notify_lvgl_flush_ready,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display));

    ESP_LOGI(TAG, "Create LVGL task");
    // Now we create the LVGL task that runs whenever LVGL has something new to do! We're not
    // just going to do that in a superloop in our main thread... what if we had a more complicated
    // application? Multithreading is nice for keeping concerns separate, letting the OS handle
    // scheduling the appropriate actions, and mentally blocking out what you don't want to think about
    // when working on unrelated code. Just make sure that you don't accidentally starve other tasks
    // out or touch non-thread-safe stuff without a semaphore/mutex :')
    xTaskCreate(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);


    // AT THIS POINT the setup of LVGL and tying it to the LCD is basically complete! HOORAY! Now
    // we can actually set up LVGL to do the things we want our application to do! This is the 
    // point where your code will start to diverge in a big way. Don't forget, for our application,
    // we are basically just creating a dummy menu system. From here you can start to look at the
    // LVGL API and dream big.

    ESP_LOGI(TAG, "Setting up LVGL Menu");

    // Don't forget -- we're in the main task here, but we launched the LVGL port task. That means
    // we have two things that would really like to access the NOT THREAD SAFE LVGL API. So,
    // let's grab our lock before we get ourselves into trouble.
    _lock_acquire(&lvgl_api_lock);

    // Here, we are creating our group. This is a collection of "Widgets" that we can use an input
    // device to select between. First we will create the group, associate it with our input device,
    // then all all our menu item widgets to it!
    lv_group_t *group = lv_group_create();
    lv_group_set_default(group);

    // Now we actually need to set up our serial input device! We create a new in(put)dev(ice) of
    // type KEYPAD, connect it to our input callback we defined earlier, assosciate it with our
    // group, and enable it. Simple as that. You could do the same thing with an encoder, buttons,
    // touchpades, etc. If you're using a touchpad, you'll want to have a callback function that
    // uses the LVGL touch recognizer functions. Thank goodness we don't need that here. 
    lv_indev_t *uart_indev = lv_indev_create();
    lv_indev_set_type(uart_indev, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_read_cb(uart_indev, uart_indev_read_cb);
    lv_indev_set_group(uart_indev, group);
    lv_indev_enable(uart_indev, true);

    // Now it's time to structure our menu!
    // Since we're building an audio playback device, we're going to go with a pretty simple test 
    // based on one of the LVGL sample menus. It should look like: 

    // Main
    // |- Audio
    // |- Settings
    //    |- Bluetooh
    //       |- List Devices
    //       |- Pair New Device
    //          |- (Display text "To be implemented!")
    // |- Extras
    

    // First we make our overall menu holder widget, taking up the full screen and be dead center
    lv_obj_t *menu = lv_menu_create(lv_screen_active());
    lv_obj_set_size(menu, lv_display_get_horizontal_resolution(NULL), lv_display_get_vertical_resolution(NULL));
    lv_obj_center(menu);


    // Now we'll make some reuseable LVGL object items that we'll use to create the menu items
    lv_obj_t *cont;
    lv_obj_t *label;

    // The way we're going to do this is by first making all the menu/submene pages and content -- then 
    // we'll link them all together in a sensible way. 

    // First we'll start with our root menu page
    lv_obj_t *root_page = lv_menu_page_create(menu, "Root");

    // Now our Settings sub-page
    lv_obj_t *settings_page = lv_menu_page_create(menu, "Settings");

    // Now our Bluetooth sub-page
    lv_obj_t *bt_page = lv_menu_page_create(menu, "Bluetooth");

    // "List Devices" isn't really menu at this point, its just a 
    // menu option that goes nowhere. So we don't need to do anything with
    // it yet. 

    // "Pair New Devices" is essentially a new menu that just displays some
    // text, so we'll set that one up. 
    lv_obj_t *bt_pair_new_page = lv_menu_page_create(menu, "Bluetooth Pair New Device");


    // NICE, now we have all our menu pages. Now lets add our linkages as well as our 
    // currently unused/unimplemented options. We can reuse our cont/label pointers
    // so we don't need to make a ton of variables that we never use again. 

    // First we'll create our Audio label as content for our root page
    // and add it to our group to make sure it is selectable
    cont = lv_menu_cont_create(root_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Audio");

    // Now we'll make our Settings label and tie it to our settings submenu!
    cont = lv_menu_cont_create(root_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Settings");
    lv_menu_set_load_page_event(menu, cont, settings_page);

    // Great, now we'll populate our Settings menu with our Bluetooth option
    // and link that to our Bluetooth submenu
    cont = lv_menu_cont_create(settings_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Bluetooth");
    lv_menu_set_load_page_event(menu, cont, bt_page);

    // Now lets add "List Device" and "Pair New Device" to our Bluetooth page.
    // For the "Pair New Device" item, be sure to hook it up to the menu 
    // we set up earlier.
    cont = lv_menu_cont_create(bt_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "List Devices");

    cont = lv_menu_cont_create(bt_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Pair New Device");
    lv_menu_set_load_page_event(menu, cont, bt_pair_new_page);

    // Now lets add the "To be implemented" text to the 
    // Pair new device page. Since we don't actually want to be 
    // able to click this text, we won't add it to our group.
    cont = lv_menu_cont_create(bt_pair_new_page);
    label = lv_label_create(cont);
    lv_label_set_text(label, "To be implemented");

    // Finally, lets add our Extras label to our root menu
    cont = lv_menu_cont_create(root_page);
    lv_group_add_obj(group, cont);
    label = lv_label_create(cont);
    lv_label_set_text(label, "Extras");

    
    // And we're done! Since we're going through all this trouble to set up a graphics library, lets at least try
    // to make the menu look like we want it to. First we'll define some colors and settings. First
    // we'll define some styles, then we'll apply them to the lv_objects (for us, the menu and header)

    // For the header, we'll go with a nice cyan background and black text
    static lv_style_t header_style;
    lv_style_init(&header_style);
    lv_style_set_bg_color(&header_style, lv_color_make(0, 255, 160));
    lv_style_set_bg_opa(&header_style, LV_OPA_COVER);
    lv_style_set_text_color(&header_style, lv_color_black());
    lv_style_set_pad_all(&header_style, 10);

    // For the menu itself, we'll go black background and white text
    static lv_style_t menu_style;
    lv_style_init(&menu_style);
    lv_style_set_bg_color(&menu_style, lv_color_black());
    lv_style_set_bg_opa(&menu_style, LV_OPA_COVER);
    lv_style_set_text_color(&menu_style, lv_color_white());

    // Now lets add that style. First we'll apply the menu style
    lv_obj_add_style(menu, &menu_style, LV_STATE_DEFAULT);

    // Now get the pointer to the header and add style to that.
    lv_obj_t *menu_header = lv_menu_get_main_header(menu);
    lv_obj_add_style(menu_header, &header_style, LV_PART_MAIN | LV_STATE_DEFAULT);


    // Great -- we've set up our menu, styled it, so now we set the base page,
    // and it's ready to rumble. We'll go ahead and set it, then release
    // the LVGL lock, because our main thread is FINALLY DONE with LVGL!
    lv_menu_set_page(menu, root_page);
    _lock_release(&lvgl_api_lock);
    
    // Now we're actually ready to turn the LCD display on! Here it goes!
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));


    ESP_LOGI(TAG, "*** Hello LVGL ESP-IDF Main Completed! ***");

    // If for some reason you had more stuff to do in your main task, you 
    // could keep it alive here. But, we've kicked off our timer and
    // our LVGL task, so it's fine to just let this task end and move
    // on with our lives.

    // If you got this far, thank you so much for reading! I appreciate
    // your time and hope you found this helpful. If you have feedback,
    // corrections, or questions, please feel free to open an issue
    // or discussion. I'd love to hear from you.
}
