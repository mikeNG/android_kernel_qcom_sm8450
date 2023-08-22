#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/wait.h>

#include "bf_fp_platform.h"

extern struct bf_device *g_bf_dev;
extern wait_queue_head_t waiting_spi_prepare;
extern atomic_t suspended;
u32 g_chip_type = 0;
static DEFINE_MUTEX(spi_lock);


int spi_set_dma_en(int mode)
{
    return 0;
}

/*----------------------------------------------------------------------------*/
int spi_send_cmd(struct bf_device *bf_dev, u8 *tx, u8 *rx, u16 spilen)
{
    int ret = 0;
    struct spi_message m;
    struct spi_transfer t = {
        .tx_buf = tx,
        .rx_buf = rx,
        .len = spilen,
        .tx_dma = 0,
        .rx_dma = 0,

    };
#ifndef KERNEL_4_9
    mutex_lock(&spi_lock);
#endif
    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    ret = spi_sync(bf_dev->spi, &m);

#ifndef KERNEL_4_9
    mutex_unlock(&spi_lock);
#endif
    return ret;
}

/*----------------------------------------------------------------------------*/
u8 bf_spi_write_reg(u8 reg, u8 value)
{
    u8 nAddr;
    u8 data_tx[4] = {0};
    u8 data_rx[4] = {0};

    nAddr = reg << 1;
    nAddr |= 0x80;

    data_tx[0] = nAddr;
    data_tx[1] = value;

    spi_send_cmd(g_bf_dev, data_tx, data_rx, 2);
    return data_rx[1];
}

//-------------------------------------------------------------------------------------------------
u8 bf_spi_read_reg(u8 reg)
{
    u8 nAddr;
    u8 data_tx[4] = {0};
    u8 data_rx[4] = {0};

    nAddr = reg << 1;
    nAddr &= 0x7F;

    data_tx[0] = nAddr;
    data_tx[1] = 0xff;

    spi_send_cmd(g_bf_dev, data_tx, data_rx, 2);
    return data_rx[1];
}


/*----------------------------------------------------------------------------*/
u8 bf_spi_write_reg_bit(u8 nRegID, u8 bit, u8 value)
{
    u8 tempvalue = 0;

    tempvalue = bf_spi_read_reg(nRegID);
    tempvalue &= ~(1 << bit);
    tempvalue |= (value << bit);
    bf_spi_write_reg(nRegID, tempvalue);

    return 0;
}

int bf_read_chipid(void)
{
    u8 val_low = 0, val_high = 0, version = 0;
    int chip_id = 0;
    u8	reg_value = 0;

    if(g_chip_type == BF3290) {
        bf_spi_write_reg (0x13, 0x00);
        reg_value = bf_spi_read_reg (0x3a);
        bf_spi_write_reg (0x3a, reg_value | 0x80);

        val_low = bf_spi_read_reg (0x10); //id reg low
        BF_LOG ("val_low=0x%x \n", val_low);

        val_high = bf_spi_read_reg (0x11); //id reg high
        BF_LOG ("val_high=0x%x \n", val_high);

        version = bf_spi_read_reg (0x12); //ic type
        BF_LOG ("version=0x%x \n", version);
        chip_id = (val_high << 8) | (val_low & 0xff);
        BF_LOG ("chip_id=%x \n", chip_id);
        bf_spi_write_reg (0x3a, reg_value);
    } else if(g_chip_type == BF3182 || g_chip_type == BF3390) {
        //enable 0x10 bit5
        reg_value = bf_spi_read_reg(0x10);
        reg_value &= ~(1 << 5);
        bf_spi_write_reg(0x10, reg_value);

        val_high = bf_spi_read_reg(0x37);
        val_low = bf_spi_read_reg(0x36);
        chip_id = (val_high << 8) | val_low;
        version = bf_spi_read_reg(0x38);

        //disabl 0x10 bit5
        reg_value |= (1 << 5);
        bf_spi_write_reg(0x10, reg_value);
    } else {
        val_high = bf_spi_read_reg(0x37);
        val_low = bf_spi_read_reg(0x36);
        chip_id = (val_high << 8) | val_low;
        version = bf_spi_read_reg(0x38);

        if(chip_id != BF3582P && chip_id != BF3582S && chip_id != 0x5883 && chip_id != 0x5B83 
                && chip_id != 0x5683 && chip_id != 0x5a83 && g_chip_type != 0x6183) {
            //enable 0x10 bit5
            reg_value = bf_spi_read_reg(0x10);
            reg_value &= ~(1 << 5);
            bf_spi_write_reg(0x10, reg_value);

            val_high = bf_spi_read_reg(0x37);
            val_low = bf_spi_read_reg(0x36);
            chip_id = (val_high << 8) | val_low;
            version = bf_spi_read_reg(0x38);

            //disabl 0x10 bit5
            reg_value |= (1 << 5);
            bf_spi_write_reg(0x10, reg_value);
            if(chip_id != BF3182 && chip_id != BF3390) {
                bf_spi_write_reg (0x13, 0x00);
                reg_value = bf_spi_read_reg (0x3a);
                bf_spi_write_reg (0x3a, reg_value | 0x80);

                val_low = bf_spi_read_reg (0x10); //id reg low
                BF_LOG ("val_low=0x%x \n", val_low);

                val_high = bf_spi_read_reg (0x11); //id reg high
                BF_LOG ("val_high=0x%x \n", val_high);

                version = bf_spi_read_reg (0x12); //ic type
                BF_LOG ("version=0x%x \n", version);
                chip_id = (val_high << 8) | (val_low & 0xff);
                BF_LOG ("chip_id=%x \n", chip_id);
                bf_spi_write_reg (0x3a, reg_value);
            }
        }
    }

    BF_LOG(" chip_id=0x%x,version=0x%x\n", chip_id, version);
    return chip_id;
}

void bf_chip_info(void)
{
    BF_LOG("data:2017-12-18\n");
    g_chip_type = bf_read_chipid();
    BF_LOG("BTL: chipid:%x\r\n", g_chip_type);
}

#if !defined(PLATFORM_SPRD) && !defined(PLATFORM_QCOM) && !defined(PLATFORM_RK)
#if defined(BF_PINCTL)
static int32_t bf_platform_pinctrl_spi_init(struct bf_device *bf_dev)
{
    int32_t error = 0;


    error = pinctrl_select_state(bf_dev->pinctrl_gpios, bf_dev->pins_spi_default);
    if (error) {
        dev_err(&bf_dev->pdev->dev, "failed to activate pins_spi_default state\n");
    }

    return error;
}

#else
static int32_t bf_platform_gpio_spi_init(struct bf_device *bf_dev)
{
    //please setup and check in dws
    return 0;
}
#endif
#endif

static int bf_spi_suspend (struct device *dev)
{
    BF_LOG("bf_spi_suspend+++\n");
    atomic_set(&suspended, 1);
    BF_LOG("bf_spi_suspend----\n");
    return 0;
}

static int bf_spi_resume (struct device *dev)
{
    BF_LOG("bf_spi_resume+++\n");
    atomic_set(&suspended, 0);
    wake_up_interruptible(&waiting_spi_prepare);
    BF_LOG("bf_spi_resume------\n");
    return 0;
}

static int bf_spi_remove(struct spi_device *spi)
{
    return 0;
}

int32_t bf_platform_uninit(struct bf_device *bf_dev)
{

    bf_remove(g_bf_dev->pdev);
    return 0;
}
//extern int IS_FINGER_USED;
static int bf_spi_probe (struct spi_device *spi)
{
    //int status = -EINVAL;

    /* Initialize the driver data */
    BF_LOG( "bf config spi ");
    g_bf_dev->spi = spi;
    /* setup SPI parameters */
    g_bf_dev->spi->mode = SPI_MODE_0;
    g_bf_dev->spi->bits_per_word = 8;
    g_bf_dev->spi->max_speed_hz = 8 * 1000 * 1000;

    spi_set_drvdata(spi, g_bf_dev);

    BF_LOG ("--- spi probe ok --");
    return 0;
}

static const struct dev_pm_ops bf_pm = {
    .suspend = bf_spi_suspend,
    .resume =  bf_spi_resume
};

#ifdef CONFIG_OF
static struct of_device_id bf_of_spi_table[] = {
    {.compatible = "betterlife,spi",},
    {},
};
//MODULE_DEVICE_TABLE(of, bf_of_spi_table);
#endif
//static struct spi_device_id bf_spi_device_id = {};
static struct spi_driver bf_spi_driver = {
    .driver = {
        .name = BF_DEV_NAME,
        .bus	= &spi_bus_type,
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = bf_of_spi_table,
#endif
        .pm = &bf_pm,
    },
    .probe = bf_spi_probe,
    .remove = bf_spi_remove,
};

int32_t bf_spi_init(struct bf_device *bf_dev)
{
    int32_t error = 0;

    spi_register_driver(&bf_spi_driver);

    return error;
}

void bf_spi_unregister()
{
    BF_LOG("%s ++", __func__);
    spi_unregister_driver(&bf_spi_driver);
    BF_LOG("%s --", __func__);
}

