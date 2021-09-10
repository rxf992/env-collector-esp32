/* BME280 BMP280 BMX180 example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>


#include "BME280_BMP280_BMP180.h"

static const char *TAG = "+AIR-PRESSURE-SENSOR-BMP280+";

ATMOSPHERE_DATA atmosphere_data;

/**
*   code auto select ic
*   0->BMP280
*   1->BME280
*   2->BMP180
*/
static uint8_t sensor_type = 0;

static uint8_t bmp180_mode = BMP180_OSS_1TIMES_RATE;

static unsigned long int hum_raw, temp_raw, pres_raw;
static signed long int t_fine;

static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static int8_t  dig_H1;
static int16_t dig_H2;
static int8_t  dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t  dig_H6;

static int16_t dig_ac1;
static int16_t dig_ac2;
static int16_t dig_ac3;
static uint16_t dig_ac4;
static uint16_t dig_ac5;
static uint16_t dig_ac6;
static int16_t dig_b1;
static int16_t dig_b2;
static int16_t dig_mb;
static int16_t dig_mc;
static int16_t dig_md;
static int32_t dig_b5;


/*  软复位芯片 */
static void chip_reset(i2c_port_t i2c_num)
{
    uint8_t rbuf = BMX280_RESET_VALUE;
    ESP_ERROR_CHECK(i2c_m_write(i2c_num,BMX280_RESET_REG, &rbuf, 1));
    vTaskDelay(100 / portTICK_RATE_MS);
}

/**
*
*/
static void readTrim(i2c_port_t i2c_num)
{
    uint8_t data[32]= {0};

    if(sensor_type==2)  //select BMP180
    {

        ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP280_DIG_AC1_MSB_REG, &data[0], 22));
        dig_ac1 = (data[0] << 8) | data[1];
        dig_ac2 = (data[2] << 8) | data[3];
        dig_ac3 = (data[4] << 8) | data[5];
        dig_ac4 = (data[6] << 8) | data[7];
        dig_ac5 = (data[8] << 8) | data[9];
        dig_ac6 = (data[10] << 8) | data[11];
        dig_b1 = (data[12] << 8) | data[13];
        dig_b2 = (data[14] << 8) | data[15];
        dig_mb = (data[16] << 8) | data[17];
        dig_mc = (data[18] << 8) | data[19];
        dig_md = (data[20] << 8) | data[21];

    }
    else    //select BMP280 or BME280
    {
        ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP280_DIG_T1_LSB_REG, &data[0], 24));

        if (sensor_type == 1)
        {
            ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP280_DIG_H1_LSB_REG, &data[24], 1));
            ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP280_DIG_H2_LSB_REG, &data[25], 7));
            dig_H1 = data[24];
            dig_H2 = (data[26] << 8) | data[25];
            dig_H3 = data[27];
            dig_H4 = (data[28] << 4) | (0x0F & data[29]);
            dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
            dig_H6 = data[31];
        }
        dig_T1 = (data[1] << 8) | data[0];
        dig_T2 = (data[3] << 8) | data[2];
        dig_T3 = (data[5] << 8) | data[4];
        dig_P1 = (data[7] << 8) | data[6];
        dig_P2 = (data[9] << 8) | data[8];
        dig_P3 = (data[11] << 8) | data[10];
        dig_P4 = (data[13] << 8) | data[12];
        dig_P5 = (data[15] << 8) | data[14];
        dig_P6 = (data[17] << 8) | data[16];
        dig_P7 = (data[19] << 8) | data[18];
        dig_P8 = (data[21] << 8) | data[20];
        dig_P9 = (data[23] << 8) | data[22];
    }
}

/** @brief init bme(p)280 chip 此处主要是启动芯片和做兼容判断
*   BME280多了湿度参数，处理数据时需要加上
*/
static esp_err_t bme280_bmp280_init(i2c_port_t i2c_num)
{
    uint8_t rbuf;

    uint8_t osrs_t = 1;             //Temperature oversampling x 1
    uint8_t osrs_p = 1;             //Pressure oversampling x 1
    uint8_t osrs_h = 1;             //Humidity oversampling x 1
    uint8_t mode = 0;               //Sleep mode
    uint8_t t_sb = 7;               //Tstandby 4000ms
    uint8_t filter = 0;             //Filter off
    uint8_t spi3w_en = 0;           //3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    i2c_addr = BMX280_I2C_ADDR;

    vTaskDelay(100 / portTICK_RATE_MS);

    // if(need_init_i2c_master){
    //     ESP_ERROR_CHECK(i2c_master_init());
    // }

    /* 读芯片ID，做BME280和BMP280的兼容*/
    ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMX280_CHIPID_REG, &rbuf, 1)); //id
    if (rbuf == BMP280_CHIP_ID)
    {
        sensor_type = 0;
        ESP_LOGI(TAG, "Get BMP280 sensor...\n");
    }
    else if (rbuf == BME280_CHIP_ID)
    {
        sensor_type = 1;
        ESP_LOGI(TAG, "Get BME280 sensor...\n");
    }
    else if (rbuf == BMP180_CHIP_ID)
    {
        sensor_type = 2;
        ESP_LOGI(TAG, "Get BMP180 sensor...\n");
    }
    else
    {
        ESP_LOGE(TAG, "No chip id, Get fail sensor.....\n");
    }

    readTrim(i2c_num);

    chip_reset(i2c_num);


    /*手册提到只要2ms就可以启动，可根据时间情况加减 */
    vTaskDelay(4 / portTICK_RATE_MS);

    if (sensor_type == 1 )  //select  BME280
    {
        ESP_ERROR_CHECK(i2c_m_write(i2c_num,BME280_CTRLHUM_REG, &ctrl_hum_reg, 1));
    }
    if(sensor_type < 2 )    //select BMP280 or BME280
    {

        ESP_ERROR_CHECK(i2c_m_write(i2c_num,BMX280_CTRLMEAS_REG, &ctrl_meas_reg, 1));
        ESP_ERROR_CHECK(i2c_m_write(i2c_num,BMX280_CONFIG_REG, &config_reg, 1));

        /* 解决开机第一次数据不稳的问题 */
        vTaskDelay(2000 / portTICK_RATE_MS);
    }
    return ESP_OK;
}


static signed long int calibration_T(signed long int adc_T)
{
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
    var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
    var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
    if (P < 0x80000000)
    {
        P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
    var2 = (((signed long int)(P >> 2)) * ((signed long int)dig_P8)) >> 13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

static unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) - (((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
                      (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
                      ((signed long int) dig_H2) + 8192) >> 14));
    v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
    v_x1 = (v_x1 < 0 ? 0 : v_x1);
    v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
    return (unsigned long int)(v_x1 >> 12);
}


static void readData(i2c_port_t i2c_num)
{
    uint8_t data[8];


    if (sensor_type == 1 )
    {
        ESP_ERROR_CHECK(i2c_m_read(I2C_MASTER_NUM, BMX280_PRESSURE_MSB_REG, &data[0], 8));
        hum_raw  = (data[6] << 8) | data[7];
    }
    else
    {
        ESP_ERROR_CHECK(i2c_m_read(I2C_MASTER_NUM, BMX280_PRESSURE_MSB_REG, &data[0], 6));
    }

    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);

}

/* read uncompensated temperature value */
static uint16_t readUT(i2c_port_t i2c_num)
{
    uint8_t buf[2]= {0};

    //buf[0] = BMP180_READ_TEMPERATURE|BMP180_START_CONVERSION|(bmp180_mode<<6);
    buf[0] =0x2e;
    ESP_ERROR_CHECK(i2c_m_write(i2c_num, BMX280_CTRLMEAS_REG, &buf[0], 1));

    vTaskDelay(10 / portTICK_RATE_MS);


    ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP180_DATA_MSB_REG, buf, 2));


    return (buf[0]<<8|buf[1]);
}


/* read uncompensated pressure value */
static int32_t readUP(i2c_port_t i2c_num)
{
    uint8_t buf[3]= {0};
    int32_t wait_time = 40;
    int32_t ret = 0;

    if(bmp180_mode ==0)
    {
        wait_time=10;
    }
    else if(bmp180_mode ==1)
    {
        wait_time=20;
    }
    else if(bmp180_mode ==2)
    {
        wait_time=30;
    }
    else if(bmp180_mode ==3)
    {
        wait_time=40;
    }

    buf[0] = BMP180_READ_PRESSURE | BMP180_START_CONVERSION | (bmp180_mode<<6);

    ESP_ERROR_CHECK(i2c_m_write(i2c_num, BMX280_CTRLMEAS_REG, &buf[0], 1));

    vTaskDelay(wait_time / portTICK_RATE_MS);

    ESP_ERROR_CHECK(i2c_m_read(i2c_num, BMP180_DATA_MSB_REG, buf, 3));

    ret =  (buf[0]<<16|buf[1]<<8|buf[0])>>(8-bmp180_mode);

    return ret;
}

int32_t computeB5(int32_t ut)
{
    int32_t x1, x2;

    x1 = (((long)ut - (long)dig_ac6)*(long)dig_ac5) >> 15;
    x2 = ((long)dig_mc << 11)/(x1 + dig_md);
    return x1 + x2;
}


// Calculate temperature in deg C
static float calculateTrueTemperature(void)
{

    float temp;
    int32_t ut;

    ut = readUT(I2C_MASTER_NUM);

    dig_b5  = computeB5(ut);
    temp = ((dig_b5 + 8)>>4);
    temp /= 10;

    return temp;
}


// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
static int32_t calculateTruePressure(void)
{
    int32_t up, x1, x2, x3, b3, b6, p;
    uint32_t b4, b7;

    up = readUP(I2C_MASTER_NUM);

    b6 = dig_b5 - 4000;
    // Calculate B3
    x1 = ((int)dig_b2 * (b6 * b6)>>12)>>11;
    x2 = ((int)dig_ac2 * b6)>>11;
    x3 = x1 + x2;
    b3 = (((((int)dig_ac1)*4 + x3)<<bmp180_mode) + 2)>>2;

    // Calculate B4
    x1 = ((int)dig_ac3 * b6)>>13;
    x2 = ((int)dig_b1 * ((b6 * b6)>>12))>>16;
    x3 = ((x1 + x2) + 2)>>2;
    b4 = ((unsigned int)dig_ac4 * (unsigned long)(x3 + 32768))>>15;

    b7 = (((unsigned int)up - b3) * (unsigned int)(50000>>bmp180_mode));
    if (b7 < 0x80000000)
        p = (b7<<1)/b4;
    else
        p = (b7/b4)<<1;

    x1 = (p>>8) * (p>>8);
    x1 = (x1 * 3038)>>16;
    x2 = (-7357 * p)>>16;
    p += (x1 + x2 + 3791)>>4;

    return p;
}

int32_t calcSealevelPressure(float altitude_meters)
{
    float pressure = calculateTruePressure();
    return (int32_t)(pressure / pow(1.0-altitude_meters/44330, 5.255));
}

float calcAltitude(int32_t pressure)
{

    float A = (float)pressure/101325.0f;
    float B = 1.0f/5.25588f;
    float C = pow(A,B);
    C = 1.0f - C;
    C *= 44330.0f;

    return C;
}

void i2c_task_bme280_bmp280_bmp180(void *arg)
{

    uint8_t rbuf;
    double temp_act = 0.0, press_act = 0.0, hum_act = 0.0;
    signed long int temp_cal;
    unsigned long int press_cal = 0;
    unsigned long int hum_cal = 0;
    int32_t pressure;
    int32_t atm = 0; // "standard atmosphere"
    float altitude = 0; //Uncompensated caculation - in Meters
    //int32_t temp_udata;
    float temp_fdata;

    /* For print flaot data*/
    int32_t for_print_temp,for_print_temp1;

    bme280_bmp280_init(I2C_MASTER_NUM);

    while(1)
    {
        if(sensor_type <2)  //select BMP280 or BME280
        {
            // Set forced mode again and again.
            ESP_ERROR_CHECK(i2c_m_read(I2C_MASTER_NUM, BMX280_CTRLMEAS_REG, &rbuf, 1)); //status
            vTaskDelay(4 / portTICK_RATE_MS);
            rbuf &= (~0x03);
            rbuf &= 0xff;
            rbuf |= 0x1;
            // write to Forced mode
            ESP_ERROR_CHECK(i2c_m_write(I2C_MASTER_NUM, BMX280_CTRLMEAS_REG, &rbuf, 1));
            vTaskDelay(20 / portTICK_RATE_MS);

            //status 不为0时，可能数据没有准备好，或数据不稳定
            ESP_ERROR_CHECK(i2c_m_read(I2C_MASTER_NUM,BMX280_STATUS_REG, &rbuf, 1));
            if (rbuf != 0)
            {
                ESP_LOGE(TAG, "Data not ready.. skip [status =0x%02x]\n", rbuf);
                vTaskDelay(2000 / portTICK_RATE_MS);
            }
            else
            {
                readData(I2C_MASTER_NUM);

                temp_cal = calibration_T(temp_raw);
                press_cal = calibration_P(pres_raw);
                temp_act = (double)temp_cal / 100.0;
                press_act = (double)press_cal / 100.0;
            #if 0
                // for_print_temp = (int)temp_act;
                // for_print_temp1 = (int)press_act;
                // ESP_LOGI(TAG, "TEMP : %d.%d°C PRESS : %d.%dhPa ",
                //          for_print_temp,(int)((temp_act-for_print_temp)*100),
                //          for_print_temp1, (int)((press_act-for_print_temp1)*100));
            #else
                ESP_LOGI(TAG, "TEMP : %.2f°C PRESS : %.2fhPa ", temp_act,press_act);
            #endif      
                atmosphere_data.temp = temp_act;
                atmosphere_data.pressure = press_act;


                if (sensor_type == 1 )   //BME280
                {
                    hum_cal = calibration_H(hum_raw);
                    hum_act = (double)hum_cal / 1024.0;

                    for_print_temp = (int)hum_act;
                    ESP_LOGI(TAG,"HUM : %d.%d%%\n",for_print_temp, (int)((hum_act-for_print_temp)*100));
                }
                else    //BMP280
                {
                    //ESP_LOGI(TAG, " \n");
                }
                vTaskDelay(1000 / portTICK_RATE_MS);
            }

        }
        else    //select BMP180
        {
            temp_fdata = calculateTrueTemperature();
            pressure = calculateTruePressure();
            atm = calcSealevelPressure(200.0f);//200 meters's pressure
            altitude =  calcAltitude(pressure);

            for_print_temp = (int)temp_fdata;
            ESP_LOGI(TAG,"temperature : %d.%dC\n",for_print_temp, (int)((temp_fdata - for_print_temp)*100));

            ESP_LOGI(TAG,"pressure : %dPa\n",pressure);

            ESP_LOGI(TAG,"If the altitude of this place is 200 meters，standard atmosphere is %dPa \n",atm);

            for_print_temp = (int)altitude;
            ESP_LOGI(TAG,"altitude: %d.%dm\n",for_print_temp, (int)((altitude - for_print_temp)*100));

            vTaskDelay(5000 / portTICK_RATE_MS);
        }
    }
    i2c_driver_delete(I2C_MASTER_NUM);
}