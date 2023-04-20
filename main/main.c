//Example developed by Martins J. - Bobsien Channel (youtube.com/Bobsien)


#include <stdio.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include <esp_task.h>
#include "esp_log.h"
#include <string.h>   

#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h" 

#include <ds1338.h>



#define SDA_CPIO 21
#define SCL_GPIO 22

#define NTP_SERVER "ca.pool.ntp.org"
#define TAG_SNTP "SNTP SERVICE"

t_time relogio;
bool SNTP_STAT = 0;
xSemaphoreHandle mutexI2CBUS;

//PROTOTIPOS DE FUNCOES
void rtcTask(void *params);
void obtain_time(void);
void initialize_sntp(void);
void sntpTask(void *params);



void setup(){
  ESP_ERROR_CHECK(i2cdev_init());

  ESP_ERROR_CHECK(nvs_flash_init());

  vTaskDelay(pdMS_TO_TICKS(5000));
  mutexI2CBUS = xSemaphoreCreateMutex();

  xTaskCreate(rtcTask,"rtcTask", 5*1024,NULL,2,NULL);
}

void app_main(void)
{
    setup();

    while (1)
    {
      printf("%04d/%02d/%02d %02d:%02d:%02d \n",relogio.tm_year, relogio.tm_mon, relogio.tm_mday, relogio.tm_hour, relogio.tm_min, relogio.tm_sec);
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    

}

void rtcTask(void *params){
  i2c_dev_t ds1338;
  bool RTCSTAT = 0;

  memset(&ds1338, 0, sizeof(i2c_dev_t));

  ESP_ERROR_CHECK(ds1338_init_desc(&ds1338, 0, SDA_CPIO,SCL_GPIO));

  ESP_ERROR_CHECK(ds1338_is_running(&ds1338, &RTCSTAT));

  if(!RTCSTAT){
    ESP_ERROR_CHECK(ds1338_start(&ds1338,0b1));
  }

  while(1){
    if(xSemaphoreTake(mutexI2CBUS,350/portTICK_PERIOD_MS)){
       ds1338_get_time(&ds1338, &relogio);

       xSemaphoreGive(mutexI2CBUS);
       vTaskDelay(pdMS_TO_TICKS(500));
    }else{
      ESP_LOGE("DS11138", "I2C TIMED OUT");
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}