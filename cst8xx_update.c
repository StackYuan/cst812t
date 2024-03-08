#include <rtthread.h>
#include <rtdevice.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <board.h>

#include "capacitive_hynitron_cst812t_update.h"
#include "cst812t.h"

#define DBG_LEVEL DBG_LOG // DBG_LOG //
#define LOG_TAG "update.cst812t"
#include <drv_log.h>

#define REG_LEN_1B 1
#define REG_LEN_2B 2
#define UP_ADDR 0x6A
#define TOUCH_BOOT_IIC_ADDR 0x6A
#define HYN_TRUE (0)
#define HYN_FAULT (1)

#define delay_ms(ms) rt_thread_mdelay(ms)

#define PER_LEN 512
static uint8_t data_send[PER_LEN];

bool TP_HRS_WriteBytes_updata(uint8_t device_addr, uint16_t reg, uint8_t *data, uint16_t len, uint8_t lenth)
{
    uint32_t ret;

    ret = cst816_i2c_write(device_addr, reg, data, len);

    if (ret < 0)
    {
        return HYN_FAULT;
    }
    return HYN_TRUE;
}

bool TP_HRS_read_updata(uint8_t device_addr, uint16_t reg, uint8_t *data, uint16_t len, uint8_t lenth)
{
    uint32_t ret;

    ret = cst816_i2c_read(device_addr, reg, data, len);

    if (ret < 0)
    {
        return HYN_FAULT;
    }
    return HYN_TRUE;
}

static int cst816s_enter_bootmode(void) // 使触摸IC进入boot模式
{
    uint8_t retryCnt = 0;

    for (retryCnt = 0; retryCnt < 10; retryCnt++)
    {
        uint8_t cmd[3];
        uint8_t read_buff[2];

        cst8xx_reset(20);

        rt_thread_mdelay(5 + retryCnt); // 5~20ms 这个延时必须准确

        cmd[0] = 0xAB; // 0xAB:CST816T/CST816D/CST816S/HYN19X2/CSK0XX		//	 0xAA；CST716
        if (TP_HRS_WriteBytes_updata(TOUCH_BOOT_IIC_ADDR, 0xA001, &cmd[0], 1, REG_LEN_2B) == HYN_FAULT)
        {
            continue;
        }

        rt_thread_mdelay(1);
        if (TP_HRS_read_updata(TOUCH_BOOT_IIC_ADDR, 0XA003, read_buff, 1, REG_LEN_2B) == HYN_FAULT)
        {
            continue;
        }

        if (read_buff[0] != 0xC1)
        { // 0xC1:CST816T、CST816D、CSK0XT    //   0x55: CST716 、CST816S、CSK0XS
            continue;
        }
        else
        {
            rt_kprintf("Enter update mode\n");
            return HYN_TRUE;
        }
    }
    if (retryCnt >= 10)
    {
        rt_kprintf("Enter update mode failed\n");
        return HYN_FAULT;
    }
    rt_kprintf("Enter update mode\n");
    return HYN_TRUE;
}

static int cst816s_update(uint16_t startAddr, uint16_t len, const unsigned char *src)
{

    uint32_t sum_len;
    uint8_t cmd[10];

    sum_len = 0;
    uint32_t k_data = 0;
    k_data = len / PER_LEN;

    if (cst816s_enter_bootmode() == 0)
    {
        // return -1;
    }
    for (uint32_t i = 0; i < k_data; i++)
    {
        cmd[0] = startAddr & 0xFF;
        cmd[1] = startAddr >> 8;
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA014, cmd, 2, REG_LEN_2B);
        memcpy(data_send, src, PER_LEN);
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA018, data_send, PER_LEN, REG_LEN_2B);
        delay_ms(10);
        cmd[0] = 0xEE;
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA004, cmd, 1, REG_LEN_2B);
        delay_ms(50);

        {
            uint8_t retrycnt = 50;
            uint16_t kcnt = 0;
            while (retrycnt--)
            {
                cmd[0] = 0;
                TP_HRS_read_updata(UP_ADDR, 0xA005, cmd, 1, REG_LEN_2B);
                LOG_D("cmd=%x \r\n", cmd[0]);

                if (cmd[0] == 0x55)
                {
                    kcnt = 0;
                    cmd[0] = 0;
                    // success
                    break;
                }
                else
                {
                    kcnt++;
                    LOG_E("error \r\n");
                }
                delay_ms(10);
            }
            if (kcnt >= 40)
                LOG_E("error \r\n");
        }
        startAddr += PER_LEN;
        src += PER_LEN;
        sum_len += PER_LEN;
    }

    k_data = len % PER_LEN;
    if (k_data > 0)
    {
        cmd[0] = startAddr & 0xFF;
        cmd[1] = startAddr >> 8;
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA014, cmd, 2, REG_LEN_2B);
        memcpy(data_send, src, k_data);
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA018, data_send, k_data, REG_LEN_2B);

        cmd[0] = 0xEE;
        TP_HRS_WriteBytes_updata(UP_ADDR, 0xA004, cmd, 1, REG_LEN_2B);

        delay_ms(100);

        {
            uint8_t retrycnt = 50;
            while (retrycnt--)
            {
                cmd[0] = 0;
                TP_HRS_read_updata(UP_ADDR, 0xA005, cmd, 1, REG_LEN_2B);
                if (cmd[0] == 0x55)
                {
                    // success
                    break;
                }
                delay_ms(10);
            }
        }
        startAddr += k_data;
        src += k_data;
        sum_len += k_data;
    }

    // exit program mode
    cmd[0] = 0x00;
    TP_HRS_WriteBytes_updata(UP_ADDR, 0xA003, cmd, 1, REG_LEN_2B);

    // cst8xx_reset(100);

    return 0;
}

/*
 *
 */
static uint32_t cst816s_read_checksum(uint16_t startAddr, uint16_t len)
{
    union
    {
        uint32_t sum;
        uint8_t buf[4];
    } checksum;
    uint8_t cmd[3];

    if (cst816s_enter_bootmode() == 0)
    {
        // return -1;
    }

    cmd[0] = 0;
    TP_HRS_WriteBytes_updata(UP_ADDR, 0xA003, cmd, 1, REG_LEN_2B);
    delay_ms(500);

    checksum.sum = 0;
    TP_HRS_read_updata(UP_ADDR, 0xA008, checksum.buf, 2, REG_LEN_2B);
    //   return -1;
    LOG_I("checksum.sum=%x \r\n", checksum.sum);
    return checksum.sum;
}

uint16_t checksum = 0;

uint32_t ctp_hynitron_update(void)
{
    if (cst816s_enter_bootmode() == 0)
    {
        LOG_I("cst816s_enter_bootmode()=%x \r\n", cst816s_enter_bootmode());
        if (sizeof(app_bin) > 10)
        {
            uint16_t startAddr = app_bin[1];
            uint16_t length = app_bin[3];

            checksum = app_bin[5];
            startAddr <<= 8;
            startAddr |= app_bin[0];
            length <<= 8;
            length |= app_bin[2];
            checksum <<= 8;
            checksum |= app_bin[4];
            if (cst816s_read_checksum(startAddr, length) != checksum)
            {
                LOG_I("startAddrO=%d \r\n", startAddr);
                LOG_I("checksum=%x \r\n", checksum);

                cst816s_update(startAddr, length, &app_bin[6]);

                LOG_I("startAddrT=%d \r\n", startAddr);
                LOG_I("checksum_new=%x \r\n", checksum);
                cst816s_read_checksum(startAddr, length);
            }
        }
        cst8xx_reset(20);
        delay_ms(30);
        return true;
    }

    return false;
}
