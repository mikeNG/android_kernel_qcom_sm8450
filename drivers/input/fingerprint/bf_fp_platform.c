#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/irqreturn.h>
#include <linux/types.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/wait.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/signal.h>
#include <linux/ctype.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <net/sock.h>
#include <linux/delay.h>

#include <linux/kernel.h>

#include <linux/interrupt.h>
#include <linux/gpio.h>


#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#elif defined(CONFIG_DRM)
#include <drm/drm_panel.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#include "bf_fp_platform.h"

//lxm
#include <linux/regulator/consumer.h>

static int bf_create_inputdev(void);

#define BF_IOCTL_MAGIC_NO                   0xFC
#define BF_IOCTL_INIT_ARGS                  _IOWR(BF_IOCTL_MAGIC_NO, 0,uint32_t)
#define BF_IOCTL_REGISTER_READ_WRITE        _IOWR(BF_IOCTL_MAGIC_NO,  1, uint32_t)
#define BF_IOCTL_RESET                      _IO(BF_IOCTL_MAGIC_NO,  2)
#define BF_IOCTL_DISABLE_INTERRUPT          _IO(BF_IOCTL_MAGIC_NO,  3)
#define BF_IOCTL_GAIN_ADJUST                _IOWR(BF_IOCTL_MAGIC_NO, 4,uint32_t)
#define BF_IOCTL_ENABLE_POWER               _IO(BF_IOCTL_MAGIC_NO, 5)
#define BF_IOCTL_DISABLE_POWER              _IO(BF_IOCTL_MAGIC_NO, 6)
#define BF_IOCTL_ENABLE_SPI_CLOCK           _IOW(BF_IOCTL_MAGIC_NO,  7,uint32_t)
#define BF_IOCTL_DISABLE_SPI_CLOCK          _IOW(BF_IOCTL_MAGIC_NO,  8,uint32_t)
#define BF_IOCTL_GET_ID                     _IOWR(BF_IOCTL_MAGIC_NO, 9, uint32_t)
#define BF_IOCTL_INIT_DEVICE                _IOW(BF_IOCTL_MAGIC_NO,  10,uint32_t)
#define BF_IOCTL_REMOVE_DEVICE              _IOW(BF_IOCTL_MAGIC_NO,  11,uint32_t)
#define BF_IOCTL_INPUT_KEY                  _IOW(BF_IOCTL_MAGIC_NO,  12,uint32_t)
#define BF_IOCTL_ENBACKLIGHT                _IOW(BF_IOCTL_MAGIC_NO,  13,uint32_t)
#define BF_IOCTL_ISBACKLIGHT                _IOWR(BF_IOCTL_MAGIC_NO, 14,uint32_t)
#define BF_IOCTL_DISPALY_STATUS             _IOW(BF_IOCTL_MAGIC_NO,  15,uint32_t)
#define BF_IOCTL_SET_PID                    _IOW(BF_IOCTL_MAGIC_NO,  16,uint32_t)
#define BF_IOCTL_INPUT_KEY_DOWN             _IOW(BF_IOCTL_MAGIC_NO,  17,uint32_t)
#define BF_IOCTL_INPUT_KEY_UP               _IOW(BF_IOCTL_MAGIC_NO,  18,uint32_t)
#define BF_IOCTL_LOW_RESET                  _IO(BF_IOCTL_MAGIC_NO,  19)
#define BF_IOCTL_HIGH_RESET                 _IO(BF_IOCTL_MAGIC_NO,  20)
#define BF_IOCTL_NETLINK_INIT               _IOW(BF_IOCTL_MAGIC_NO,  21,uint32_t)
#define BF_IOCTL_TRANS_IC_INFO              _IOW(BF_IOCTL_MAGIC_NO,  22,uint32_t)
#define BF_IOCTL_ENABLE_INTERRUPT           _IO(BF_IOCTL_MAGIC_NO,  23)
#define BF_IOCTL_RESET_FLAG                 _IOW(BF_IOCTL_MAGIC_NO,  24,uint32_t)
#define BF_IOCTL_IS_OPT_POWER_ON2V8       	_IOWR(BF_IOCTL_MAGIC_NO,  25, uint32_t)
#define BF_IOCTL_CREATE_INPUT               _IO(BF_IOCTL_MAGIC_NO,  26)
#define BF_IOCTL_COMPATIBLE_IN_HAL          _IOWR(BF_IOCTL_MAGIC_NO,  27, uint32_t)
#define BF_IOCTL_POWER_GPIO_STATUS          _IOWR(BF_IOCTL_MAGIC_NO,  28, uint32_t)
#define BF_IOCTL_RESET_GPIO_STATUS          _IOWR(BF_IOCTL_MAGIC_NO,  29, uint32_t)
#define BF_IOCTL_WAKE_LOCK                             _IO(BF_IOCTL_MAGIC_NO,  30)
#define BF_IOCTL_WAKE_UNLOCK                           _IO(BF_IOCTL_MAGIC_NO,  31)
//lxm111
static const u16 bf_key[7] = {
			KEY_F10, //click once
			KEY_F11, //double click
			KEY_F12, //long press
			KEY_UP, //KEY_UP,
			KEY_DOWN, //KEY_DOWN,
			KEY_LEFT, //KEY_LEFT,
			KEY_RIGHT //KEY_RIGHT
			};

/* for netlink use */
static int g_pid;
static int g_netlink_port = NETLINK_BF;
struct bf_device *g_bf_dev = NULL;
static struct input_dev *bf_inputdev = NULL;
bl_ic_info_t ic_info;
static uint32_t bf_key_need_report = 0;
static void bf_unregister(void);

#ifdef KERNEL_4_9
//lxm
/*
static struct wakeup_source fp_suspend_lock;
static struct wakeup_source hw_reset_lock;
static struct wakeup_source adjust_wake_lock;
*/
static struct wakeup_source *fp_suspend_lock;
static struct wakeup_source *hw_reset_lock;
static struct wakeup_source *adjust_wake_lock;
#else
static struct wake_lock fp_suspend_lock;
static struct wake_lock hw_reset_lock;
static struct wake_lock adjust_wake_lock;
#endif
static struct kobject *bf_kobj = NULL;
static DEFINE_MUTEX(irq_count_lock);
static DEFINE_MUTEX(g_dev_lock);
static irqreturn_t bf_eint_handler (int irq, void *data);
int g_bl229x_enbacklight = 1;

//extern u32 g_chip_type;
//lxm add for 8475
u32 g_chip_type = 0;

DECLARE_WAIT_QUEUE_HEAD (waiting_spi_prepare);
atomic_t suspended;

static int bf_hw_power(struct bf_device *bf_dev, bool enable)
{
    return 0;
}

static int bf_hw_reset(struct bf_device *bf_dev)
{

    gpio_direction_output (bf_dev->reset_gpio, 0);
    mdelay(5);
    gpio_direction_output (bf_dev->reset_gpio, 1);

    return 0;
}

static int bf_hw_reset_level (struct bf_device *bf_dev, bool enable)
{
    BF_LOG("bf_hw_reset_level %d", enable);

    if (enable) {
        gpio_direction_output(bf_dev->reset_gpio, 1);
    } else {
        gpio_direction_output(bf_dev->reset_gpio, 0);
    }

    return 0;
}

static ssize_t bf_show_hwreset(struct device *ddri, struct device_attribute *attr, char *buf)
{
    u32 pin_val = -1;
    pin_val = gpio_get_value(g_bf_dev->reset_gpio);
    BF_LOG("reset pin_val=%d\n", pin_val);

    return sprintf(buf, "reset pin_val=%d\n", pin_val);
}
static ssize_t bf_store_hwreset(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    u8 hwreset_flag  = simple_strtoul(buf, &next, 10);
    BF_LOG("hwreset_flag %d\n", hwreset_flag);
    if(hwreset_flag) {
        bf_hw_reset_level(g_bf_dev, 1);
    } else {
        bf_hw_reset_level(g_bf_dev, 0);
    }

    return size;
}
static DEVICE_ATTR(reset, 0664, bf_show_hwreset, bf_store_hwreset);



static ssize_t bf_show_chipid(struct device *ddri, struct device_attribute *attr, char *buf)
{

    return sprintf(buf, "%x", ic_info.ic_chipid);
}
static ssize_t bf_store_chipid(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    BF_LOG("bf_store_senorchipid ");

    return 0;
}
static DEVICE_ATTR(chipid, 0664, bf_show_chipid, bf_store_chipid);

static ssize_t bf_show_version(struct device *ddri, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s", ic_info.ca_ver);
}
static ssize_t bf_store_version(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    char *next;
    BF_LOG("bf_store_senorversion");

    return 0;
}
static DEVICE_ATTR(version, 0664, bf_show_version, bf_store_version);

/*----------------------------------------------------------------------------*/
/*static struct device_attribute *bf_attr_list[] = {
    &dev_attr_reset,
    &dev_attr_chipid,
    &dev_attr_version,
#if defined(NEED_OPT_POWER_ON2V8) || defined(NEED_OPT_POWER_ON1V8)
    &dev_attr_power
#endif
};*/
/*----------------------------------------------------------------------------*/
/*
static void bf_create_attributes(struct device *dev)
{
    int num = (int)(sizeof(bf_attr_list) / sizeof(bf_attr_list[0]));
    for (; num > 0;)
        device_create_file(dev, bf_attr_list[--num]);
}
*/
#if 0
static int bf_sysfs_init(void)
{
    int ret = 0;
    bf_kobj = kobject_create_and_add("bf_sysfs", NULL);
    if(bf_kobj == NULL) {
        BF_LOG("subsystem_register failed\n");
        ret = -ENOMEM;
        return ret;
    }

    ret = sysfs_create_file(bf_kobj, &dev_attr_reset.attr);
    ret = sysfs_create_file(bf_kobj, &dev_attr_chipid.attr);
    ret = sysfs_create_file(bf_kobj, &dev_attr_version.attr);
#if defined(NEED_OPT_POWER_ON2V8) || defined(NEED_OPT_POWER_ON1V8)
    ret = sysfs_create_file(bf_kobj, &dev_attr_power.attr);
#endif
    if(ret) {
        BF_LOG("sysfs_create_file failed\n");
    }
	BF_LOG("sysfs_create_file ok !!\n");

    kobject_uevent(bf_kobj, KOBJ_ADD);
    return ret;
}

static int bf_sysfs_uninit(void)
{
    int ret = 0;

    if(bf_kobj == NULL) {
        BF_LOG("bf_kobj don't exist \n");
        ret = -EEXIST;
        return ret;
    }

    sysfs_remove_file(bf_kobj, &dev_attr_reset.attr);
    sysfs_remove_file(bf_kobj, &dev_attr_chipid.attr);
    sysfs_remove_file(bf_kobj, &dev_attr_version.attr);
#if defined(NEED_OPT_POWER_ON2V8) || defined(NEED_OPT_POWER_ON1V8)
    sysfs_remove_file(bf_kobj, &dev_attr_power.attr);
#endif
    kobject_del(bf_kobj);
    return ret;
}
#endif
/*
static void bf_remove_attributes(struct device *dev)
{
    int num = (int)(sizeof(bf_attr_list) / sizeof(bf_attr_list[0]));
    for (; num > 0;)
        device_remove_file(dev, bf_attr_list[--num]);
}
*/
/**
 * get gpio information from device tree
 */
static int bf_main_get_gpio_info (struct bf_device *bf_dev)
{
    //struct device_node *node = NULL;
	//lxm
	struct device *dev = &bf_dev->pdev->dev;
	struct device_node *node = dev->of_node;
    //int32_t ret = 0;

    //node = bf_dev->pdev->dev.of_node;
    if (node) {
        bf_dev->reset_gpio = of_get_named_gpio(node, "fpreset-gpio", 0);
        if(bf_dev->reset_gpio < 0) {
            BF_LOG("get fpreset-gpio fail:%d", bf_dev->reset_gpio);
            return bf_dev->reset_gpio;
        }

        bf_dev->irq_gpio =  of_get_named_gpio(node, "fpint-gpio", 0);
        if(bf_dev->irq_gpio < 0) {
            BF_LOG("get fpint-gpio fail:%d", bf_dev->irq_gpio);
            return bf_dev->irq_gpio;
        }

        //if gpio_to_irq cause irq has problem then chang to use irq_of_parse_and_map func and dts needto modify
        //bf_dev->irq_num = irq_of_parse_and_map(bf_dev->pdev->dev.of_node, 0);
        bf_dev->irq_num = gpio_to_irq(bf_dev->irq_gpio);
        if(!bf_dev->irq_num) {
            BF_LOG("get irq number fail:%d", bf_dev->irq_num);
            return -ENXIO;
        }
        BF_LOG("fpreset-gpio:%d, fpint-gpio:%d, irq_num:%d", bf_dev->reset_gpio, bf_dev->irq_gpio, bf_dev->irq_num);

//lxm
	bf_dev->vcc = devm_regulator_get_optional(dev, "vcc");
	if (IS_ERR_OR_NULL(bf_dev->vcc )) {
		BF_LOG("Can't retrieve VCC reg\n");
		bf_dev->vcc = NULL;
	}
	if (bf_dev->vcc) {
		regulator_enable(bf_dev->vcc);
		BF_LOG("Reg enabled\n");
	}
//
    } else {
        BF_LOG( "device of_node is null");
        return -EINVAL;
    }
    return 0;
}
/*
 *gpio方式初始化
 */
static int32_t bf_main_gpio_init(struct bf_device *bf_dev)
{
    int error = 0;
    /*reset pin*/
    if (gpio_is_valid(bf_dev->reset_gpio)) {
        error = gpio_request(bf_dev->reset_gpio, "bf reset");
        if (error) {
            dev_err(&bf_dev->pdev->dev, "unable to request reset GPIO %d\n", bf_dev->reset_gpio);
            goto out;
        } else {
            mdelay(5);
            gpio_direction_output (bf_dev->reset_gpio, 1);
            mdelay(5);
            gpio_direction_output (bf_dev->reset_gpio, 0);
            mdelay(5);
            gpio_direction_output (bf_dev->reset_gpio, 1);
        }
    } else {
        dev_err(&bf_dev->pdev->dev, "invalid reset GPIO %d\n", bf_dev->reset_gpio);
        error = -1;
        goto out;
    }

    /*irq pin*/
    if (gpio_is_valid(bf_dev->irq_gpio)) {
        error = gpio_request(bf_dev->irq_gpio, "bf irq_gpio");
        if (error) {
            dev_err(&bf_dev->pdev->dev, "unable to request irq_gpio GPIO %d\n", bf_dev->irq_gpio);
            goto out1;
        } else {
            gpio_direction_input(bf_dev->irq_gpio);
        }
    } else {
        dev_err(&bf_dev->pdev->dev, "invalid irq_gpio GPIO %d\n", bf_dev->irq_gpio);
        error = -1;
        goto out1;
    }


    return 0;


out3:

out2:
    gpio_free(bf_dev->irq_gpio);
out1:
    gpio_free(bf_dev->reset_gpio);
out:
    return error;
}

static void bf_main_gpio_uninit(struct bf_device *bf_dev)
{
    if (gpio_is_valid(bf_dev->irq_gpio))
        gpio_free(bf_dev->irq_gpio);
    if (gpio_is_valid(bf_dev->reset_gpio))
        gpio_free(bf_dev->reset_gpio);
}

static void bf_main_pin_uninit(struct bf_device *bf_dev)
{
#ifdef BF_PINCTL
    if(bf_dev->pinctrl_gpios)
        devm_pinctrl_put(bf_dev->pinctrl_gpios);
#endif
}

static int32_t bf_main_pin_init(struct bf_device *bf_dev)
{
    int32_t error = 0;

    error = bf_main_gpio_init(bf_dev);
    if(error) {
        BF_LOG("bf_main_gpio_init fail!");
    }

    return error;
}

/* -------------------------------------------------------------------- */
/* netlink functions                 */
/* -------------------------------------------------------------------- */
void bf_send_netlink_msg(struct bf_device *bf_dev, const int command)
{
    struct nlmsghdr *nlh = NULL;
    struct sk_buff *skb = NULL;
    int ret;
    char data_buffer[2];

    BF_LOG("enter, send command %d", command);
    memset(data_buffer, 0, 2);
    data_buffer[0] = (char)command;
    if (NULL == bf_dev->netlink_socket) {
        BF_LOG("invalid socket");
        return;
    }

    if (0 == g_pid) {
        BF_LOG("invalid native process pid");
        return;
    }

    /*alloc data buffer for sending to native*/
    skb = alloc_skb(MAX_NL_MSG_LEN, GFP_ATOMIC);
    if (skb == NULL) {
        return;
    }

    nlh = nlmsg_put(skb, 0, 0, 0, MAX_NL_MSG_LEN, 0);
    if (!nlh) {
        BF_LOG("nlmsg_put failed");
        kfree_skb(skb);
        return;
    }

    NETLINK_CB(skb).portid = 0;
    NETLINK_CB(skb).dst_group = 0;

    *(char *)NLMSG_DATA(nlh) = command;
    *((char *)NLMSG_DATA(nlh) + 1) = 0;
    ret = netlink_unicast(bf_dev->netlink_socket, skb, g_pid, MSG_DONTWAIT);
    if (ret < 0) {
        BF_LOG("send failed");
        return;
    }

    BF_LOG("send done, data length is %d", ret);
    return ;
}

static void bf_recv_netlink_msg(struct sk_buff *__skb)
{
    struct sk_buff *skb = NULL;
    struct nlmsghdr *nlh = NULL;
    char str[128];

    skb = skb_get(__skb);
    if (skb == NULL) {
        BF_LOG("skb_get return NULL");
        return;
    }

    if (skb->len >= NLMSG_SPACE(0)) {
        nlh = nlmsg_hdr(skb);
        //add by wangdongbo
        //memcpy(str, NLMSG_DATA(nlh), sizeof(str));
        g_pid = nlh->nlmsg_pid;
        BF_LOG("pid: %d, msg: %s", g_pid, str);
        mutex_lock(&irq_count_lock);
        g_bf_dev->irq_count = 1;
        mutex_unlock(&irq_count_lock);
    } else {
        BF_LOG("not enough data length");
    }

    kfree_skb(__skb);
}

/*
#ifndef MTK_ANDROID_L
static int bf_destroy_inputdev(void)
{
    if (bf_inputdev) {
        input_unregister_device(bf_inputdev);
        //input_free_device(bf_inputdev);
        bf_inputdev = NULL;
    }
    return 0;
}
#endif
*/
static int bf_close_netlink(struct bf_device *bf_dev)
{
    if (bf_dev->netlink_socket != NULL) {
        netlink_kernel_release(bf_dev->netlink_socket);
        bf_dev->netlink_socket = NULL;
        return 0;
    }

    BF_LOG("no netlink socket yet");
    return -1;
}

static int bf_init_netlink(struct bf_device *bf_dev)
{
    struct netlink_kernel_cfg cfg;

    BF_LOG("bf_init_netlink g_netlink_port: %d", g_netlink_port);

    memset(&cfg, 0, sizeof(struct netlink_kernel_cfg));
    cfg.input = bf_recv_netlink_msg;

    bf_dev->netlink_socket = netlink_kernel_create(&init_net, g_netlink_port, &cfg);
    if (bf_dev->netlink_socket == NULL) {
        BF_LOG("netlink create failed");
        return -1;
    }
    BF_LOG("netlink create success");
    return 0;
}

static irqreturn_t bf_eint_handler (int irq, void *data)
{
    struct bf_device *bf_dev = (struct bf_device *)data;

    wait_event_interruptible_timeout(waiting_spi_prepare, !atomic_read(&suspended), msecs_to_jiffies (100));
#ifdef KERNEL_4_9
//lxm
    //__pm_wakeup_event(&fp_suspend_lock, msecs_to_jiffies (5000));
	__pm_wakeup_event(fp_suspend_lock, msecs_to_jiffies (5000));

#else
    wake_lock_timeout(&fp_suspend_lock, msecs_to_jiffies (5000));
#endif
    //BF_LOG("++++irq_handler netlink send+++++,%d,%d", g_bf_dev->irq_count, bf_dev->doing_reset);
    if(g_bf_dev->irq_count) {
        if(!bf_dev->doing_reset) {
            mutex_lock(&irq_count_lock);
            g_bf_dev->irq_count = 0;
            mutex_unlock(&irq_count_lock);
        }
        bf_send_netlink_msg(bf_dev, BF_NETLINK_CMD_IRQ);
    }
    //BF_LOG("-----irq_handler netlink -----");
    return IRQ_HANDLED;
}

/* -------------------------------------------------------------------- */
/* file operation function                                                                                */
/* -------------------------------------------------------------------- */
static long bf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int error = 0;
    u32 bl229x_enbacklight = 0;
    struct bf_device *bf_dev = NULL;
    unsigned int key_event = 0;
    unsigned int value = 0;
    BF_LOG("bf_ioctl.");

    bf_dev = (struct bf_device *)filp->private_data;
    if (_IOC_TYPE(cmd) != BF_IOCTL_MAGIC_NO) {
        BF_LOG("Not blestech fingerprint cmd.");
        return -EINVAL;
    }

    if (_IOC_DIR(cmd) & _IOC_READ)
        error = !access_ok(/*VERIFY_WRITE,*/ (void __user *)arg, _IOC_SIZE(cmd));

    if (error == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
        error = !access_ok(/*VERIFY_READ,*/ (void __user *)arg, _IOC_SIZE(cmd));

    if (error) {
        BF_LOG("Not blestech fingerprint cmd direction.");
        return -EINVAL;
    }

    switch (cmd) {
    case BF_IOCTL_RESET:
        BF_LOG("BF_IOCTL_RESET: chip reset command\n");
        bf_hw_reset(bf_dev);
        break;
    case BF_IOCTL_ENABLE_INTERRUPT:
        BF_LOG("BF_IOCTL_ENABLE_INTERRUPT:  command,%d\n", bf_dev->irq_count);
        mutex_lock(&irq_count_lock);
        bf_dev->irq_count = 1;
        mutex_unlock(&irq_count_lock);
        break;
    case BF_IOCTL_DISABLE_INTERRUPT:
        BF_LOG("BF_IOCTL_DISABLE_INTERRUPT:  command,%d\n", bf_dev->irq_count);
        mutex_lock(&irq_count_lock);
        bf_dev->irq_count = 0;
        mutex_unlock(&irq_count_lock);
        break;
    case BF_IOCTL_ENABLE_POWER:
        BF_LOG("BF_IOCTL_ENABLE_POWER:  command\n");
        bf_hw_power(bf_dev, 1);
        break;
    case BF_IOCTL_DISABLE_POWER:
        BF_LOG("BF_IOCTL_DISABLE_POWER:  command\n");
        bf_hw_power(bf_dev, 0);
        break;
    case BF_IOCTL_INPUT_KEY:
		key_event = (unsigned int)arg;
		BF_LOG("lxm111 key_event:%d\n", key_event);
		if((key_event > 7) || (key_event < 1))
			break;
		input_report_key(bf_inputdev, bf_key[key_event-1], 1);
		input_sync(bf_inputdev);
		input_report_key(bf_inputdev, bf_key[key_event-1], 0);
		input_sync(bf_inputdev);
        break;
    case BF_IOCTL_ENBACKLIGHT:
        BF_LOG("BF_IOCTL_ENBACKLIGHT arg:%d\n", (int)arg);
        g_bl229x_enbacklight = (int)arg;
        break;
    case BF_IOCTL_ISBACKLIGHT:
        BF_LOG("BF_IOCTL_ISBACKLIGHT\n");
        bl229x_enbacklight = g_bl229x_enbacklight;
        if (copy_to_user((void __user*)arg, &bl229x_enbacklight, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }
        break;
    case BF_IOCTL_GET_ID:
        if (copy_to_user((void __user*)arg, &g_chip_type, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }
        break;
    case BF_IOCTL_INPUT_KEY_DOWN:
        if(g_bl229x_enbacklight && g_bf_dev->need_report == 0 && bf_key_need_report == 0) {
            bf_key_need_report = 1;
            key_event = (int)arg;
            input_report_key(bf_inputdev, key_event, 1);
            input_sync(bf_inputdev);
        }
        break;
    case BF_IOCTL_INPUT_KEY_UP:
        if(bf_key_need_report == 1) {
            bf_key_need_report = 0;
            key_event = (int)arg;
            input_report_key(bf_inputdev, key_event, 0);
            input_sync(bf_inputdev);
        }
        break;
    case BF_IOCTL_LOW_RESET:
        BF_LOG("BF_IOCTL_LOW_RESET:  command\n");
#ifdef KERNEL_4_9
//lxm
       // __pm_wakeup_event(&hw_reset_lock, 2 * HZ);
       __pm_wakeup_event(hw_reset_lock, 2 * HZ);
#else
        wake_lock_timeout(&hw_reset_lock, 2 * HZ);
#endif
        bf_hw_reset_level(g_bf_dev, 0);
        break;

    case BF_IOCTL_HIGH_RESET:
        BF_LOG("BF_IOCTL_HIGH_RESET:  command\n");
        bf_hw_reset_level(g_bf_dev, 1);
        break;

    case BF_IOCTL_NETLINK_INIT:
        BF_LOG("BF_IOCTL_NETLINK_INIT:  command\n");
        g_netlink_port = (int)arg;
        bf_close_netlink(g_bf_dev);
        error = bf_init_netlink(g_bf_dev);
        if (error < 0) {
            BF_LOG("BF_IOCTL_NETLINK_INIT:  error\n");
        }
        break;

    case BF_IOCTL_TRANS_IC_INFO:
        BF_LOG("lxm BTL:BF_IOCTL_TRANS_IC_INFO\n");
        if (!copy_from_user(&ic_info, (bl_ic_info_t *)arg, sizeof(bl_ic_info_t))) {
            BF_LOG("lxm ic_info:  name = %s, chipid = %x \n",ic_info.ic_name, ic_info.ic_chipid);
        }
        break;

    case BF_IOCTL_INIT_DEVICE:
        BF_LOG("BF_IOCTL_INIT_DEVICE:  command\n");
        error = bf_init_dts_and_irq(g_bf_dev);
        break;

    case BF_IOCTL_REMOVE_DEVICE:
        BF_LOG("BF_IOCTL_REMOVE_DEVICE:  command\n");
        //bf_remove(g_bf_dev->pdev);
        bf_unregister();
        break;

    case BF_IOCTL_RESET_FLAG:
        bf_dev->doing_reset = (u8)arg;
        BF_LOG("BF_IOCTL_RESET_FLAG:  command,%d\n", bf_dev->doing_reset);
        break;
    case BF_IOCTL_IS_OPT_POWER_ON2V8:
        BF_LOG("BF_IOCTL_IS_OPT_POWER_ON2V8:  command\n");

        /*key_event = 0;

        if (copy_to_user((void __user*)arg, &key_event, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }*/
        break;
    case BF_IOCTL_CREATE_INPUT:
        BF_LOG("BF_IOCTL_CREATE_INPUT:  command\n");
        error = bf_create_inputdev();
        if (error) {
            BF_LOG("BF_IOCTL_CREATE_INPUT:  error\n");
        }
        break;
    case BF_IOCTL_COMPATIBLE_IN_HAL:
        BF_LOG("BF_IOCTL_COMPATIBLE_IN_HAL:  command\n");
#ifdef COMPATIBLE_IN_HAL
        value = 1;
#else
        value = 0;
#endif
        if (copy_to_user((void __user*)arg, &value, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }
        break;
    case BF_IOCTL_POWER_GPIO_STATUS:
        BF_LOG("BF_IOCTL_POWER_GPIO_STATUS:  command");

        value = 2;
        if (copy_to_user((void __user*)arg, &value, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }


        break;
    case BF_IOCTL_RESET_GPIO_STATUS:
        BF_LOG("BF_IOCTL_RESET_GPIO_STATUS:  command");
        value = gpio_get_value(g_bf_dev->reset_gpio);
        BF_LOG("gpio reset status value : %d", value);
        if (copy_to_user((void __user*)arg, &value, sizeof(u32) * 1) != 0 ) {
            error = -EFAULT;
        }

        break;
    case BF_IOCTL_WAKE_LOCK:
        BF_LOG("BF_IOCTL_WAKE_LOCK:  command\n");
#ifdef KERNEL_4_9
//lxm
       // __pm_stay_awake(&adjust_wake_lock);
		__pm_stay_awake(adjust_wake_lock);
#else
        wake_lock(&adjust_wake_lock);
#endif
        break;
    case BF_IOCTL_WAKE_UNLOCK:
        BF_LOG("BF_IOCTL_WAKE_UNLOCK:  command\n");
#ifdef KERNEL_4_9
//lxm
        __pm_relax(adjust_wake_lock);
#else
        wake_unlock(&adjust_wake_lock);
#endif
        break;
    default:
        BF_LOG("Supportn't this command(%x)\n", cmd);
        break;
    }

    return error;
}


#ifdef CONFIG_COMPAT
static long bf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int retval = 0;

    retval = bf_ioctl(filp, cmd, arg);

    return retval;
}
#endif

/*----------------------------------------------------------------------------*/
static int bf_open (struct inode *inode, struct file *filp)
{
    struct bf_device *bf_dev = g_bf_dev;
    int status = 0;

    filp->private_data = bf_dev;
    BF_LOG( " Success to open device.");

    return status;
}


/* -------------------------------------------------------------------- */
static ssize_t bf_write (struct file *file, const char *buff, size_t count, loff_t *ppos)
{
    return -ENOMEM;
}

/* -------------------------------------------------------------------- */
static ssize_t bf_read (struct file *filp, char  *buff, size_t count, loff_t *ppos)
{

    ssize_t status = 0;
    BF_LOG("status: %d \n", (int)status);
    BF_LOG("  --\n");
    return status;

}

/* -------------------------------------------------------------------- */
static int bf_release (struct inode *inode, struct file *file)
{
    int status = 0 ;
    return status;
}
static int bf_suspend (struct platform_device *pdev, pm_message_t state)
{
    BF_LOG("  ++\n");
    mutex_lock(&g_dev_lock);
    if(g_bf_dev != NULL)
        g_bf_dev->need_report = 1;
    mutex_unlock(&g_dev_lock);
    BF_LOG("\n");
    return 0;
}
static int bf_resume (struct platform_device *pdev)
{
    BF_LOG("  ++\n");
    mutex_lock(&g_dev_lock);
    if(g_bf_dev != NULL)
        g_bf_dev->need_report = 0;
    mutex_unlock(&g_dev_lock);
    BF_LOG("\n");
    return 0;
}

/*----------------------------------------------------------------------------*/
static const struct file_operations bf_fops = {
    .owner = THIS_MODULE,
    .open  = bf_open,
    .write = bf_write,
    .read  = bf_read,
    .release =  bf_release,
    .unlocked_ioctl = bf_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = bf_compat_ioctl,
#endif
};

static struct miscdevice bf_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = BF_DEV_NAME,
    .fops = &bf_fops,
};

int bf_remove(struct platform_device *pdev)
{
    struct bf_device *bf_dev = g_bf_dev;
    BF_LOG("bf_remove++++");
    mutex_lock(&g_dev_lock);
    //bf_remove_attributes(&bf_dev->pdev->dev);
    //bf_sysfs_uninit();

    bf_hw_power(bf_dev, 0);

    if (bf_dev->irq_num) {
        free_irq(bf_dev->irq_num, bf_dev);
        bf_dev->irq_count = 0;
        bf_dev->irq_num = 0;
    }

    bf_main_gpio_uninit(bf_dev);
    bf_main_pin_uninit(bf_dev);
    misc_deregister(&bf_misc_device);
    bf_close_netlink(bf_dev);


    platform_set_drvdata(bf_dev->pdev, NULL);
    if(NULL != bf_dev)
        kfree(bf_dev);
    bf_dev      = NULL;
    g_bf_dev    = NULL;
    mutex_unlock(&g_dev_lock);
    BF_LOG("bf_remove----");
    return 0;
}

static int bf_create_inputdev(void)
{
    if (bf_inputdev) {
        BF_LOG("bf_inputdev in not null, already created!\n");
        return 0;
    }
    bf_inputdev = input_allocate_device();
    if (!bf_inputdev) {
        BF_LOG("bf_inputdev create faile!\n");
        return -ENOMEM;
    }
	//lxm111
    __set_bit(EV_KEY, bf_inputdev->evbit);
    __set_bit(KEY_F10, bf_inputdev->keybit);    //68 0x44
    __set_bit(KEY_F11, bf_inputdev->keybit);    //87 0x57
    __set_bit(KEY_F12, bf_inputdev->keybit);    //88 0x58
    __set_bit(KEY_UP, bf_inputdev->keybit);     //103 0x67
    __set_bit(KEY_DOWN, bf_inputdev->keybit);   //108 0x6c
    __set_bit(KEY_LEFT, bf_inputdev->keybit);   //105 0x69
    __set_bit(KEY_RIGHT, bf_inputdev->keybit);  //106 0x6a

	BF_LOG("lxm117 1 register inputdev\n");

    bf_inputdev->id.bustype = BUS_HOST;
//    bf_inputdev->id.vendor = 0x0008;
//    bf_inputdev->id.product = 0x0002;
    bf_inputdev->name = "betterlife_inputdev";
    if (input_register_device(bf_inputdev)) {
        BF_LOG("register inputdev failed");
        input_free_device(bf_inputdev);
        return -ENOMEM;
    }
    return 0;
}

int bf_init_dts_and_irq(struct bf_device *bf_dev)
{
    static int initialized = 0;
    int32_t status = -EINVAL;
    int ret;
    BF_LOG( "    ++++");
    if(initialized != 1) {
        status = bf_main_get_gpio_info(bf_dev);
        if(status) {
            BF_LOG("bf_main_get_gpio_info fail:%d", status);
            return -1;
        }

        status = bf_main_pin_init(bf_dev);
        if(status) {
            BF_LOG("bf_main_init fail:%d", status);
            bf_main_gpio_uninit(bf_dev);
            bf_main_pin_uninit(bf_dev);
            return -2;
        }

        status = request_threaded_irq (bf_dev->irq_num, NULL, bf_eint_handler,  IRQ_TYPE_EDGE_RISING /*IRQF_TRIGGER_RISING*/ | IRQF_ONESHOT, BF_DEV_NAME, bf_dev);

        if (status) {
            BF_LOG("irq thread request failed, retval=%d\n", status);
            bf_main_gpio_uninit(bf_dev);
            bf_main_pin_uninit(bf_dev);
            return -3;
        }

        bf_hw_power(bf_dev, 1);
        //bf_hw_reset(bf_dev);

        enable_irq_wake(bf_dev->irq_num);
        initialized = 1;

    } else {
        BF_LOG( " has initilized, do nothing !");
    }

    BF_LOG( "    ----");
    return 0;
}

static void bf_platform_info(void)
{
    char bl_platform[128];
    char tee_platform[128];

    memcpy(bl_platform, "PLATFORM_QCOM", sizeof("PLATFORM_QCOM"));
    memcpy(tee_platform, "undefined", sizeof("undefined"));
	
    BF_LOG("bf_platform_info: CPU_PLATFORM:%s TEE_PLATFORM:%s \r\n", bl_platform , tee_platform);
}

#if 1
static ssize_t bf_chipid_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", ic_info.ic_chipid);
}

static ssize_t bf_chipid_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}

static ssize_t bf_name_show(
    struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ic_info.ic_name);
}

static ssize_t bf_name_store(
    struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    return count;
}


static DEVICE_ATTR(bf_chipid, S_IRUGO | S_IWUSR, bf_chipid_show, bf_chipid_store);
static DEVICE_ATTR(bf_name, S_IRUGO | S_IWUSR, bf_name_show, bf_name_store);

static struct attribute *bf_attributes[] = {
    &dev_attr_bf_chipid.attr,
    &dev_attr_bf_name.attr,
    NULL
};

static struct attribute_group bf_attribute_group = {
    .attrs = bf_attributes
};
#endif
static int bf_probe(struct platform_device *pdev)
{
    struct bf_device *bf_dev = NULL;
    int32_t status = -EINVAL;
	int ret;
	
    BF_LOG( "bf_probe ++++++++++++");
	pr_err("lxm bf_probe \n");
    bf_platform_info();

    bf_dev = kzalloc(sizeof (struct bf_device), GFP_KERNEL);
    if (NULL == bf_dev) {
        BF_LOG( "kzalloc bf_dev failed.");
        status = -ENOMEM;
        goto err0;
    }

    bf_dev->pdev = pdev;
    bf_dev->irq_count = 0;
    bf_dev->doing_reset = 0;
    bf_dev->report_key = KEY_F10;
    bf_dev->reset_gpio = -1;
    bf_dev->irq_gpio = -1;
#ifdef KERNEL_4_9
//lxm
/*
    wakeup_source_init(&fp_suspend_lock, "fp_wakelock");
    wakeup_source_init(&hw_reset_lock, "fp_reset_wakelock");
    wakeup_source_init(&adjust_wake_lock, "adjust_wake_lock");
*/
	fp_suspend_lock = wakeup_source_register(&bf_dev->pdev->dev, "fp_wakelock");
	hw_reset_lock = wakeup_source_register(&bf_dev->pdev->dev, "fp_reset_wakelock");
	adjust_wake_lock = wakeup_source_register(&bf_dev->pdev->dev, "adjust_wake_lock");

#else
    wake_lock_init(&fp_suspend_lock, WAKE_LOCK_SUSPEND, "fp_wakelock");
    wake_lock_init(&hw_reset_lock, WAKE_LOCK_SUSPEND, "fp_reset_wakelock");
    wake_lock_init(&adjust_wake_lock, WAKE_LOCK_SUSPEND, "adjust_wake_lock");
#endif
    atomic_set(&suspended, 0);
    g_bf_dev = bf_dev;
    platform_set_drvdata(pdev, bf_dev);

#if defined(COMPATIBLE_IN_HAL) || defined(COMPATIBLE)
    BF_LOG("compatible in hal or COMPATIBLE, do not init gpio pin hw reset and init_irq.");
#else
    status = bf_init_dts_and_irq(bf_dev);
    if (status) {
        goto err2;
    }
#endif

    /* netlink interface init */
    /*BF_LOG ("bf netlink config");
    if (bf_init_netlink(bf_dev) < 0) {
        BF_LOG ("bf_netlink create failed");
        status = -EINVAL;
        goto err5;
    }*/

    status = misc_register(&bf_misc_device);
    if(status) {
        BF_LOG("bf_misc_device register failed\n");
        goto err6;
    }
#if 0
    status = bf_sysfs_init();
    if(status) {
        BF_LOG("bf_sysfs_init failed\n");
        goto err8;
    }
#else
    ret = sysfs_create_group(&bf_dev->pdev->dev.kobj, &bf_attribute_group);
    if (ret) {
        BF_LOG("[EX]: sysfs_create_group() failed!!");
        sysfs_remove_group(&bf_dev->pdev->dev.kobj, &bf_attribute_group);
    } else {
        BF_LOG("[EX]: sysfs_create_group() succeeded!!");
    }
#endif
    //bf_create_attributes(&bf_dev->pdev->dev);

#if !defined(COMPATIBLE_IN_HAL) && !defined(COMPATIBLE)
    //bf_hw_power(bf_dev, 1);
    //bf_hw_reset(bf_dev);
#endif

    pr_err("lxm bf_probe success!");
    return 0;

err9:
    //bf_remove_attributes(&bf_dev->pdev->dev);
    //bf_sysfs_uninit();
err8:
#if defined(CONFIG_FB)
    fb_unregister_client(&bf_dev->fb_notify);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend(&bf_dev->early_suspend);
#endif

err7:
    misc_deregister(&bf_misc_device);
err6:
    bf_close_netlink(bf_dev);
err5:
    free_irq(bf_dev->irq_num, bf_dev);
err2:
err1:
    platform_set_drvdata(bf_dev->pdev, NULL);
    kfree(bf_dev);
err0:
    BF_LOG("bf_probe occured error \n");
    return status;
}

static const struct of_device_id bf_of_table[] = {
    { .compatible = BF_PLATDEV_NAME },
    {},
};
MODULE_DEVICE_TABLE(of, bf_of_table);

static struct platform_driver bf_plt_driver = {
    .driver = {
        .name = BF_DEV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = bf_of_table,
    },

    .probe = bf_probe,
    .remove = bf_remove,
    .resume = bf_resume,
    .suspend = bf_suspend,
};

void bf_unregister()
{
    platform_driver_unregister(&bf_plt_driver);
}

static int  bf_plt_init(void)
{
    int ret = 0;
	BF_LOG ("bf_plt_init !");
	pr_err("lxm bf_plt_init \n");

    ret = platform_driver_register(&bf_plt_driver);
    if(ret)
    {
        pr_err ("lxm platform_driver_register for blfp failed!");
        return -1;
    }
	pr_err("lxm bf_plt_init end\n");

    return ret;
}

static void  bf_plt_exit(void)
{
    BF_LOG ("exit !");
    bf_unregister();
}

late_initcall(bf_plt_init);
module_exit(bf_plt_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR ("betterlife@blestech.com");
MODULE_DESCRIPTION ("Betterlife fingerprint sensor driver.");
