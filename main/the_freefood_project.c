/*
The free food project
*/

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_event_loop.h"

#include "i2c.h"
#include "stdbool.h"
#include "string.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "apps/sntp/sntp.h"
#include "camera.h"
#include "bitmap.h"
#include "http_server.h"

#include "include/dht11.h"
#include "include/fonts.h"
#include "include/ssd1306.h"

#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/dhcp.h"

#include <netinet/in.h>
#include <arpa/inet.h>

//Wifi
#define CONNECTED_BIT   BIT0
#define WIFI_SSID       "Vodafone-119E56"
#define WIFI_PASS       "CRL693FDP"
#define DEVICE_IP       "192.168.1.100"         
#define DEVICE_GW       "192.168.1.254"
#define DEVICE_NETMASK  "255.255.255.0"
#define DNS_SERVER_IP   "192.168.1.254"

//OLED
#define OLED_I2C_SCL_PIN    4       //OLED i2c scl pin
#define OLED_I2C_SDA_PIN    5       //OLED i2c sda pin

//DHT
#define DHT_PIN             16      //DHT11 sensor data pin

//LEDs
#define LEDS_TIMER          LEDC_TIMER_1
#define LEDS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDS_CHANNEL        LEDC_CHANNEL_1
#define LEDS_PIN            0       //LED lights pin
#define LEDS_PWM_FREQ       1000    //LEDs PWM frequency

//Atomizer
#define ATOMIZER_TIMER      LEDC_TIMER_0
#define ATOMIZER_MODE       LEDC_HIGH_SPEED_MODE
#define ATOMIZER_CHANNEL    LEDC_CHANNEL_0
#define ATOMIZER_PIN        2         //Atomizer pin
#define ATOMIZER_FREQ       113600    //Atomizer operating frequency

//TCP server
#define MAX_CLIENTS         5
#define TCP_PORT            3000

//Camera
#define CAMERA_PIXEL_FORMAT CAMERA_PF_JPEG
#define CAMERA_FRAME_SIZE   CAMERA_FS_QVGA

//initialize global variables
static EventGroupHandle_t wifi_event_group;
int LEDs_intensity = 100;     //values accepted [0-100]
struct tm timeinfo = { 0 };
int chamber_T;
int chamber_RH;
int atomizer_state = 1;       //[ON] - atomizer_state=1, [OFF] - atomizer_state=0
//static ip4_addr_t s_ip_addr;
//static camera_pixelformat_t s_pixel_format;
//static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=123456789000000000000987654321";

//static const char *STREAM_BOUNDARY = "--123456789000000000000987654321";

// Wifi event handler
static esp_err_t event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {

    case SYSTEM_EVENT_STA_START:
        ESP_LOGI("WIFI", "Connecting to %s", WIFI_SSID);
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;

	case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;

	case SYSTEM_EVENT_STA_DISCONNECTED:
	    ESP_LOGI("WIFI", "Disconnected. Reconnecting to %s", WIFI_SSID);
	    ESP_ERROR_CHECK(esp_wifi_start());
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
		ESP_LOGI("WIFI", "SYSTEM_EVENT_ETH_GOT_IP");
		break;
	default:
        break;
    }
	return ESP_OK;
}

void connect_wifi() {
    tcpip_adapter_ip_info_t ip_info;
    tcpip_adapter_dns_info_t dns_info;
    
    //disable the default wifi logging
	esp_log_level_set("wifi", ESP_LOG_NONE);

	//initialize NVS
	ESP_ERROR_CHECK(nvs_flash_init());

	//create the event group to handle wifi events
	wifi_event_group = xEventGroupCreate();

	//initialize the tcp stack
	tcpip_adapter_init();
    
    //initialize the wifi event handler
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    //stop DHCP server
    tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA); 

    //set IP addresses
    ipaddr_aton(DNS_SERVER_IP, &dns_info.ip);
    inet_pton(AF_INET, DEVICE_IP, &ip_info.ip);
    inet_pton(AF_INET, DEVICE_GW, &ip_info.gw);
    inet_pton(AF_INET, DEVICE_NETMASK, &ip_info.netmask);
    tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info);
    dns_setserver(0, &dns_info.ip);

	//initialize the wifi stack in STAtion mode with config in RAM
	wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	//configure the wifi connection and start the interface
	wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    //wait for connection
	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
    ssd1306_draw_string(0, 0, 0, "Connected to Wifi", 1, 0);
    ssd1306_refresh(0, true);

	/*print IP addresses 
	ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip_info));
    ESP_ERROR_CHECK(tcpip_adapter_get_dns_info(TCPIP_ADAPTER_IF_STA, TCPIP_ADAPTER_DNS_MAIN, &dns_info));
    printf("IPv4:        %s\n", ip4addr_ntoa(&ip_info.ip));
    printf("Subnet mask: %s\n", ip4addr_ntoa(&ip_info.netmask));
	printf("Gateway:     %s\n", ip4addr_ntoa(&ip_info.gw));
    printf("DNS:         %s\n", ipaddr_ntoa(&dns.ip));
    */
}

static void clock_task(void *pvParam) {
    time_t now = 0;
    int minute = 0;
    char strftime_buf[16];

    //initialize SNTP
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    //set to Lisbon timezone
    setenv("TZ", "WET0WEST,M3.5.0/1,M10.5.0", 1);
    tzset();

    //set time for first time
    while(timeinfo.tm_year < (2016 - 1900)) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    ESP_LOGI("TIME", "Successfully set RTC");
    
    //wait for time to be set
    while(1) {
        time(&now);
        localtime_r(&now, &timeinfo);
        if(timeinfo.tm_min != minute) {
            minute = timeinfo.tm_min;
            strftime(strftime_buf, sizeof(strftime_buf), "%R", &timeinfo);
            ssd1306_fill_rectangle(0, 0, 54, 127, 10, 0);
            ssd1306_draw_string(0, 0, 54, strftime_buf, 1, 0);
            ssd1306_refresh(0, true);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void atomizer_task(void *pvParam) {
    int atomizer_prev_state = atomizer_state;

    //initialize channel config struct
    ledc_channel_config_t atomizer_channel = {
        .channel    = ATOMIZER_CHANNEL,       //controller's channel number
        .duty       = atomizer_state,         //output duty cycle
        .gpio_num   = ATOMIZER_PIN,           //GPIO output
        .speed_mode = ATOMIZER_MODE,          //speed mode, either high or low
        .timer_sel  = ATOMIZER_TIMER          //timer servicing selected channel
    };
    //initialize timer config struct
    ledc_timer_config_t atomizer_timer = {
        .duty_resolution = 1,                 // resolution of PWM duty
        .freq_hz         = ATOMIZER_FREQ,     // frequency of PWM signal
        .speed_mode      = ATOMIZER_MODE,     // timer mode
        .timer_num       = ATOMIZER_TIMER     // timer index
    };

    //set PWM channel config
    ESP_ERROR_CHECK(ledc_channel_config(&atomizer_channel));

    //set PWM timer config
    ESP_ERROR_CHECK(ledc_timer_config(&atomizer_timer));
    ESP_LOGI("ATOMIZER", "Initialized successfully")

    while(1) {
        if(atomizer_state != atomizer_prev_state) {
            ledc_set_duty(ATOMIZER_MODE, ATOMIZER_CHANNEL, atomizer_state);
            ledc_update_duty(ATOMIZER_MODE, ATOMIZER_CHANNEL);
            atomizer_prev_state = atomizer_state;
            if(atomizer_state == 1) {
                ESP_LOGI("ATOMIZER", "Atomizer turned on");
            } else {
                ESP_LOGI("ATOMIZER", "Atomizer turned off");
            }
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

static void initialize_OLED() {
    //initialize OLED
    if(ssd1306_init(0, OLED_I2C_SCL_PIN, OLED_I2C_SDA_PIN)) {
        ESP_LOGI("OLED", "Initialized successfully")
    } else {
        ESP_LOGE("OLED", "Initialization failed");
    }
    ssd1306_select_font(0, 1);
}

void LEDs_task(void *pvParam) {
    int LEDs_prev_state = LEDs_intensity;

    //initialize channel config struct
    ledc_channel_config_t LEDs_channel = {
        .channel    = LEDS_CHANNEL,          //controller's channel number
        .duty       = LEDs_intensity,        //output duty cycle
        .gpio_num   = LEDS_PIN,              //GPIO output
        .speed_mode = LEDS_MODE,             //speed mode, either high or low
        .timer_sel  = LEDS_TIMER             //timer servicing selected channel
    };
    //initialize timer config struct
    ledc_timer_config_t LEDs_timer = {
        .duty_resolution = 10,               // resolution of PWM duty
        .freq_hz         = LEDS_PWM_FREQ,    // frequency of PWM signal
        .speed_mode      = LEDS_MODE,        // timer mode
        .timer_num       = LEDS_TIMER        // timer index
    };

    //set PWM channel config
    ESP_ERROR_CHECK(ledc_channel_config(&LEDs_channel));

    //set PWM timer config
    ESP_ERROR_CHECK(ledc_timer_config(&LEDs_timer));
    ESP_LOGI("LEDS", "Initialized successfully")

    while(1) {
        if(LEDs_intensity != LEDs_prev_state) {
            ledc_set_duty(LEDS_MODE, LEDS_CHANNEL, LEDs_intensity*1023/100);
            ledc_update_duty(LEDS_MODE, LEDS_CHANNEL);
            LEDs_prev_state = LEDs_intensity;
            ESP_LOGI("LED", "LEDs intensity set to: %d", LEDs_intensity);
        }
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void dht_task(void *pvParam) {
    char dht[16];

    while(1) {
        while(dht_read(DHT_PIN, &chamber_RH, &chamber_T)!=0) {
            ESP_LOGW("DHT", "Failed to get reading. Retrying...");
            vTaskDelay(200/portTICK_PERIOD_MS);
        }
        sprintf(dht, "T=%d RH=%d", chamber_T, chamber_RH);
        ESP_LOGI("DHT", "Reading: %s", dht);
        ssd1306_fill_rectangle(0, 0, 11, 126, 10, 0);
        ssd1306_draw_string(0, 0, 10, dht, 1, 0);
        ssd1306_refresh(0, true);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

char * server_process_msg(char *msg) {
	int aux = 0;
    static char data[64];
	
	//get data
	//chamber_T
	if(strncmp(msg, "get chamber_T", 13) == 0) {
		sprintf(data, "chamber_T: %dºC\n", chamber_T);
		return data;
	}
	//chamber_RH
	else if(strncmp(msg, "get chamber_RH", 14) == 0) {
		sprintf(data, "chamber_RH: %d%%\n", chamber_RH);
		return data;
	}
	//LEDs_intensity
	else if(strncmp(msg, "get LEDs_intensity", 18) == 0) {
		sprintf(data, "LEDs_intensity: %d%%\n", LEDs_intensity);
		return data;
	}
	//atomizer_state
	else if(strncmp(msg, "get atomizer_state", 18) == 0) {
		if(atomizer_state == 1) {
			sprintf(data, "atomizer_state: ON\n");
		} else {
			sprintf(data, "atomizer_state: OFF\n");
		}
		return data;
	}
	//set parameters
	//LEDs_intensity
	else if(strncmp(msg, "set LEDs_intensity", 18) == 0) {
		sscanf(msg, "%s %s %d", data, data, &aux);
		if(aux >= 0 || aux <= 100) {
			LEDs_intensity = aux;
			sprintf(data, "LEDs_intensity set to: %d%%\n", LEDs_intensity);
			return data;
		} else {
			sprintf(data, "Invalid LEDs_intensity value\n");
			return data;
		}
	}
	//atomizer_state
	else if(strncmp(msg, "set atomizer_state", 14) == 0) {
		sscanf(msg, "%s %s %d", data, data, &aux);
		if(aux == 1) {
			atomizer_state = aux;
            //printf("aux = %d", aux);
			sprintf(data, "atomizer_state set to: ON\n");
			return data;
		} else if(aux == 0) {
			atomizer_state = aux;
            //printf("aux = %d", aux);
			sprintf(data, "atomizer_state set to: OFF\n");
			return data;
		} else {
			sprintf(data, "Invalid atomizer_state value\n");
			return data;
		}
	}
	//unknown command
	else {
		sprintf(data, "Unknown command bitch!!!\n");
	}
	return data;
}

void tcp_server(void *pvParam) {
    struct sockaddr_in tcpServerAddr;
    tcpServerAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tcpServerAddr.sin_family = AF_INET;
    tcpServerAddr.sin_port = htons(TCP_PORT);

    char buffer[64];
    char data[64];
    static struct sockaddr_in remote_addr;
    static unsigned int socklen = sizeof(remote_addr);
    int server_socket;
    int client_socket[MAX_CLIENTS];
    int new_socket;
    int sd, max_sd;
    int valread;
    int activity;
    int i;
    int opt = 1;
    fd_set readfds;


    while(1) {
        //initialise all client_socket to 0 so not checked
        for (i = 0; i < MAX_CLIENTS; i++) {
            client_socket[i] = 0;
        }
        
        //allocate socket
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if(server_socket < 0) {
            ESP_LOGE("TCP", "Failed to create server socket.");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        //bind socket
        if(bind(server_socket, (struct sockaddr *)&tcpServerAddr, sizeof(tcpServerAddr)) != 0) {
            ESP_LOGE("TCP", "Server socket bind failed errno=%d.", errno);
            close(server_socket);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        //set master socket to allow multiple connections
        if(setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0) {
            ESP_LOGE("TCP", "Failed to configure server socket errno=%d.", errno);
            close(server_socket);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }
        
        //listen for incoming connections
        if(listen(server_socket, 5) != 0) {
            ESP_LOGE("TCP", "Server socket listen failed errno=%d.", errno);
            close(server_socket);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI("TCP", "Server initialization successful.");
        while(1) {
            //clear the socket set
            FD_ZERO(&readfds);

            //add server socket to set
            FD_SET(server_socket, &readfds);
            max_sd = server_socket;

            //add child sockets to set
            for(i = 0; i < MAX_CLIENTS; i++) {
                //socket descriptor
                sd = client_socket[i];

                //if valid socket descriptor then add to read list
                if(sd > 0)
                    FD_SET(sd, &readfds);

                //highest file descriptor number, need it for the select function
                if(sd > max_sd)
                    max_sd = sd;
            }
            
            //wait for an activity on one of the sockets
            activity = select(max_sd + 1 , &readfds , NULL , NULL , NULL);
            if(activity < 0) {
                ESP_LOGE("TCP", "Failed during select() errno=%d.", errno);
            }

            //If something happened on the server socket, then its an incoming connection
            if(FD_ISSET(server_socket, &readfds)) {
                if((new_socket = accept(server_socket, (struct sockaddr *)&remote_addr, &socklen)) < 0) {
                    ESP_LOGE("TCP", "Failed to accept connection errno=%d.", errno);
                    continue;
                }
                
                //Print client IP and Port
                ESP_LOGI("TCP", "New connection: IP: %s Port: %d", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));

                //add new socket to array of sockets
                for(i = 0; i < MAX_CLIENTS; i++) {
                    //if position is empty
                    if(client_socket[i] == 0) {
                        client_socket[i] = new_socket;
                        ESP_LOGI("TCP", "Adding to list of sockets as %d" , i);
                        break;
                    }
                }
                if(i == MAX_CLIENTS) {
                    ESP_LOGW("TCP", "Server reached max number of connections, rejecting IP: %s Port: %d" , inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));
                    close(new_socket);
                    continue;
                } else {
                    //send new connection greeting message
                    if(send(new_socket, "What up nigga?\n", 14, 0) != 14) {
                        ESP_LOGE("TCP", "Could not send greetings errno=%d.", errno);
                    }
                }
            }

            //else its some IO operation on some other socket
            for(i = 0; i < MAX_CLIENTS; i++) {
                sd = client_socket[i];

                if(FD_ISSET(sd , &readfds)) {
                    //Check if it was for closing and read the incoming message
                    if((valread = read(sd, buffer, sizeof(buffer))) < 0) {
                        ESP_LOGE("TCP", "Failed to receive data from %s:%d", inet_ntoa(remote_addr.sin_addr), htons(remote_addr.sin_port));
                    } else if(valread == 0) {
                        //Somebody disconnected, get his details and print
                        getpeername(sd ,(struct sockaddr*)&remote_addr , &socklen);
                        ESP_LOGW("TCP", "Client disconnected: IP: %s Port %d" , inet_ntoa(remote_addr.sin_addr) , htons(remote_addr.sin_port));

                        //Close the socket and mark as 0 in list for reuse
                        close(sd);
                        client_socket[i] = 0;
                    } else {
                        //Process message and perform task
                        strcpy(data,server_process_msg(buffer));
                        if(write(sd, data, strlen(data)) < 0) {
                            ESP_LOGE("TCP", "Send failed errno=%d.", errno);
                            continue;
                        }
                    }
                    bzero(buffer, sizeof(buffer)); 
                }
            }
        }
        ESP_LOGI("TCP", "Server will be opened in 5 seconds.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

//camera http shit
/*
static void handle_jpg_stream(http_context_t http_ctx, void* ctx) {
    http_response_begin(http_ctx, 200, STREAM_CONTENT_TYPE, HTTP_RESPONSE_SIZE_UNKNOWN);

    while (true) {
        esp_err_t err = camera_run();
        if (err != ESP_OK) {
            ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
            return;
        }
        err = http_response_begin_multipart(http_ctx, "image/jpg",
                camera_get_data_size());
        if (err != ESP_OK) {
            break;
        }
        err = write_frame(http_ctx);
        if (err != ESP_OK) {
            break;
        }
        err = http_response_end_multipart(http_ctx, STREAM_BOUNDARY);
        if (err != ESP_OK) {
            break;
        }
    }
    http_response_end(http_ctx);
}

static void handle_jpg(http_context_t http_ctx, void* ctx) {
    esp_err_t err = camera_run();
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Camera capture failed with error = %d", err);
        return;
    }

    http_response_begin(http_ctx, 200, "image/jpeg", camera_get_data_size());
    http_response_set_header(http_ctx, "Content-disposition", "inline; filename=capture.jpg");
    write_frame(http_ctx);
    http_response_end(http_ctx);
}

static esp_err_t write_frame(http_context_t http_ctx) {
    http_buffer_t fb_data = {
            .data = camera_get_fb(),
            .size = camera_get_data_size(),
            .data_is_persistent = true
    };
    return http_response_write(http_ctx, &fb_data);
}

void HTTP_server() {
    http_server_t server;
    http_server_options_t http_options = HTTP_SERVER_OPTIONS_DEFAULT();
    
    esp_log_level_set("gpio", ESP_LOG_WARN);
    
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    camera_config_t camera_config = {
        .ledc_channel = LEDC_CHANNEL_2,
        .ledc_timer = LEDC_TIMER_2,
        .pin_d0 = CONFIG_D0,
        .pin_d1 = CONFIG_D1,
        .pin_d2 = CONFIG_D2,
        .pin_d3 = CONFIG_D3,
        .pin_d4 = CONFIG_D4,
        .pin_d5 = CONFIG_D5,
        .pin_d6 = CONFIG_D6,
        .pin_d7 = CONFIG_D7,
        .pin_xclk = CONFIG_XCLK,
        .pin_pclk = CONFIG_PCLK,
        .pin_vsync = CONFIG_VSYNC,
        .pin_href = CONFIG_HREF,
        .pin_sscb_sda = CONFIG_SDA,
        .pin_sscb_scl = CONFIG_SCL,
        .pin_reset = CONFIG_RESET,
        .xclk_freq_hz = CONFIG_XCLK_FREQ,
    };

    camera_model_t camera_model;
    err = camera_probe(&camera_config, &camera_model);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera probe failed with error 0x%x", err);
        return;
    }

    if (camera_model == CAMERA_OV2640) {
        ESP_LOGI("CAM", "Detected OV2640 camera, using JPEG format");
        s_pixel_format = CAMERA_PF_JPEG;
        camera_config.frame_size = CAMERA_FRAME_SIZE;
        camera_config.jpeg_quality = 15;
    } else {
        ESP_LOGE("CAM", "Camera not supported");
        return;
    }

    camera_config.pixel_format = s_pixel_format;
    err = camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera init failed with error 0x%x", err);
        return;
    }

    ESP_ERROR_CHECK(http_server_start(&http_options, &server));
    
    ESP_ERROR_CHECK( http_register_handler(server, "/jpg", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg, NULL) );
    ESP_LOGI(TAG, "Open http://" IPSTR "/jpg for single image/jpg image", IP2STR(&s_ip_addr));
    ESP_ERROR_CHECK( http_register_handler(server, "/jpg_stream", HTTP_GET, HTTP_HANDLE_RESPONSE, &handle_jpg_stream, NULL) );
    ESP_LOGI(TAG, "Open http://" IPSTR "/jpg_stream for multipart/x-mixed-replace stream of JPEGs", IP2STR(&s_ip_addr));
    
    ESP_LOGI(TAG, "Free heap: %u", xPortGetFreeHeapSize());
    ESP_LOGI(TAG, "Camera demo ready");

}*/


//void camera_task

void app_main() {
    //initialize OLED
    initialize_OLED();

    //connect to wifi
    connect_wifi(); //testar desconexão

	xTaskCreate(&LEDs_task, "LEDs_task", 2048, NULL, 5, NULL);
	xTaskCreate(&atomizer_task, "atomizer_task", 2048, NULL, 5, NULL);
	xTaskCreate(&dht_task, "dht_task", 2048, NULL, 5, NULL);
	xTaskCreate(&clock_task, "clock_task", 2048, NULL, 5, NULL);
	xTaskCreate(&tcp_server, "tcp_server", 2048, NULL, 5, NULL);
}