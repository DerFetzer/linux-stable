/*
 * VTL CTP driver
 *
 * Copyright (C) 2013 VTL Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include "linux/gpio/consumer.h"
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/of_gpio.h>
#include <linux/vmalloc.h>

#include "vtl_ts.h"
#include "chip.h"
#include "apk.h"
#include "tp_fw.h"

#define TS_THREAD_PRIO 90

// ****************************************************************************
// Globel or static variables
// ****************************************************************************
static struct ts_driver g_driver;
static int vtl_first_init_flag = 1;
struct ts_info g_ts = {
	.driver = &g_driver,
	.debug = DEBUG_ENABLE,
};
struct ts_info *pg_ts = &g_ts;

static struct i2c_device_id vtl_ts_id[] = { { DRIVER_NAME, 0 }, {} };
MODULE_DEVICE_TABLE(i2c, vtl_ts_id);

// ****************************************************************************
// Function declaration
// ****************************************************************************
unsigned char *gtpfw;

static int vtl_ts_read_xy_data(struct ts_info *ts)
{
	struct i2c_msg msgs;
	int ret;

	DEBUG();

	msgs.addr = ts->driver->client->addr;
	msgs.flags = 0x01; // 0x00: write 0x01:read
	msgs.len = sizeof(ts->xy_data.buf);
	msgs.buf = ts->xy_data.buf;
	ret = i2c_transfer(ts->driver->client->adapter, &msgs, 1);
	if (ret != 1) {
		printk("___%s:i2c read xy_data err___\n", __func__);
		return -1;
	}
	return 0;
}

static void vtl_ts_report_xy_coord(struct ts_info *ts)
{
	int id;
	int sync;
	int x, y;
	unsigned int press;
	unsigned char touch_point_number;
	static unsigned int release = 0;
	struct input_dev *input_dev;
	union ts_xy_data *xy_data;

	DEBUG();

	xy_data = &ts->xy_data;
	input_dev = ts->driver->input_dev;
	touch_point_number = ts->config_info.touch_point_number;

	/* report points */
	sync = 0;
	press = 0;
	for (id = 0; id < touch_point_number; id++) //down
	{
		if ((xy_data->point[id].xhi != 0xFF) &&
		    (xy_data->point[id].yhi != 0xFF) &&
		    ((xy_data->point[id].status == 1) ||
		     (xy_data->point[id].status == 2))) {
			if (ts->config_info.xy_swap == 1) {
				x = (xy_data->point[id].yhi << 4) |
				    (xy_data->point[id].ylo & 0xF);
				y = (xy_data->point[id].xhi << 4) |
				    (xy_data->point[id].xlo & 0xF);
			} else {
				x = (xy_data->point[id].xhi << 4) |
				    (xy_data->point[id].xlo & 0xF);
				y = (xy_data->point[id].yhi << 4) |
				    (xy_data->point[id].ylo & 0xF);
			}
			if (ts->config_info.x_reverse)
				x = ts->config_info.screen_max_x - x;
			if (ts->config_info.y_reverse)
				y = ts->config_info.screen_max_y - y;

			x = ts->config_info.x_mul * x;
			y = ts->config_info.x_mul * y;

			if (ts->debug) {
				printk("id = %d,status = %d,X = %d,Y = %d\n",
				       xy_data->point[id].id,
				       xy_data->point[id].status, x, y);
			}
			input_mt_slot(input_dev, xy_data->point[id].id - 1);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID,
					 xy_data->point[id].id - 1);
			input_report_abs(input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);

			press |= 0x01 << (xy_data->point[id].id - 1);
			sync = 1;
		}
	}
	release &= (release ^ press); //release point flag

	for (id = 0; id < touch_point_number; id++) //up
	{
		if (release & (0x01 << id)) {
			input_mt_slot(input_dev, id);
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, -1);
			sync = 1;
		}
	}

	release = press;
	if (sync) {
		input_sync(input_dev);
	}
}

static irqreturn_t vtl_ts_irq(int irq, void *dev)
{
	struct ts_info *ts;
	int err;

	ts = pg_ts;

	DEBUG();

	err = vtl_ts_read_xy_data(ts);
	if (err < 0) {
		dev_err(&ts->driver->client->dev, "[%s] err:%d\n", __func__,
			err);
		return IRQ_NONE;
	}
	vtl_ts_report_xy_coord(ts);

	return IRQ_HANDLED;
}

static int vtl_ts_config(struct ts_info *ts)
{
	struct device *dev = &ts->driver->client->dev;
	int err;
	struct device_node *np = dev->of_node;
	int val;
	gtpfw = tp_fw;

	DEBUG();
	/* ts config */
	ts->config_info.touch_point_number = TOUCH_POINT_NUM;
	if (dev->platform_data != NULL) {
		return -1;
	} else {
		if (of_property_read_u32(np, "screen_max_x", &val)) {
			dev_err(&ts->driver->client->dev,
				"no screen_max_x defined\n");
			return -EINVAL;
		}
		ts->config_info.screen_max_x = val;

		if (of_property_read_u32(np, "screen_max_y", &val)) {
			dev_err(&ts->driver->client->dev,
				"no screen_max_y defined\n");
			return -EINVAL;
		}
		ts->config_info.screen_max_y = val;

		if (of_property_read_u32(np, "xy_swap", &val)) {
			val = 0;
		}
		ts->config_info.xy_swap = val;

		if (of_property_read_u32(np, "x_reverse", &val)) {
			val = 0;
		}
		ts->config_info.x_reverse = val;

		if (of_property_read_u32(np, "y_reverse", &val)) {
			val = 0;
		}
		ts->config_info.y_reverse = val;

		if (of_property_read_u32(np, "x_mul", &val)) {
			val = 1;
		}
		ts->config_info.x_mul = val;

		if (of_property_read_u32(np, "y_mul", &val)) {
			val = 1;
		}
		ts->config_info.y_mul = val;

		if (of_property_read_u32(np, "bin_ver", &val)) {
			val = 0;
		}
		ts->config_info.bin_ver = val;
		printk("--->>> vtl_ts : xy_swap %d, x_reverse %d, y_reverse %d, x_mul %d, y_mul %d, bin_ver %d\n",
		       ts->config_info.xy_swap, ts->config_info.x_reverse,
		       ts->config_info.y_reverse, ts->config_info.x_mul,
		       ts->config_info.y_mul, ts->config_info.bin_ver);
		printk("the screen_x is %d , screen_y is %d \n",
		       ts->config_info.screen_max_x,
		       ts->config_info.screen_max_y);

		err = devm_request_threaded_irq(dev,
						ts->driver->client->irq,
						NULL, vtl_ts_irq, IRQF_ONESHOT,
						"vtl_touch_irq", ts);
		if (err) {
			dev_err(dev, "request threaded irq failed: %d\n", err);
			return err;
		}

		ts->config_info.rst_gpio =
			devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
		if (IS_ERR(ts->config_info.rst_gpio)) {
			err = PTR_ERR(ts->config_info.rst_gpio);
			dev_err(dev, "request gpiod failed: %d", err);
			return err;
		}
	}
	if (err) {
		return -EIO;
	}

	return 0;
}

struct ts_info *vtl_ts_get_object(void)
{
	DEBUG();

	return pg_ts;
}

void vtl_ts_hw_reset(void)
{
	struct ts_info *ts;
	ts = pg_ts;
	DEBUG();

	gpiod_set_value(ts->config_info.rst_gpio, 1);
	msleep(50);
	gpiod_set_value(ts->config_info.rst_gpio, 0);
	msleep(5);
	chip_solfware_reset(ts->driver->client); //20140306
}

static void vtl_ts_wakeup(void)
{
	struct ts_info *ts;
	ts = pg_ts;
	DEBUG();

	gpiod_set_value(ts->config_info.rst_gpio, 0);
	//msleep(50);
	msleep(20);
	gpiod_set_value(ts->config_info.rst_gpio, 1);
	msleep(5);
	chip_solfware_reset(ts->driver->client); //20140306
}

int vtl_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ts_info *ts;
	unsigned char i;
	ts = pg_ts;
	DEBUG();
	if (ts->config_info.ctp_used) {
		vtl_first_init_flag = 0;
		disable_irq(client->irq);
		chip_enter_sleep_mode();

		for (i = 0; i < ts->config_info.touch_point_number; i++) {
			input_mt_slot(ts->driver->input_dev, i);
			input_report_abs(ts->driver->input_dev,
					 ABS_MT_TRACKING_ID, -1);
		}
		input_sync(ts->driver->input_dev);
	}
	return 0;
}

int vtl_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ts_info *ts;
	unsigned char i;
	ts = pg_ts;

	DEBUG();
	if (ts->config_info.ctp_used) {
		/* Hardware reset */
		vtl_ts_wakeup();
		for (i = 0; i < ts->config_info.touch_point_number; i++) {
			input_mt_slot(ts->driver->input_dev, i);
			input_report_abs(ts->driver->input_dev,
					 ABS_MT_TRACKING_ID, -1);
		}
		input_sync(ts->driver->input_dev);
		if (vtl_first_init_flag == 0)
			enable_irq(client->irq);
	}
	return 0;
}

static int vtl_ts_init_input_dev(struct ts_info *ts)
{
	struct input_dev *input_dev;
	struct device *dev;
	int err;

	DEBUG();

	dev = &ts->driver->client->dev;

	/* allocate input device */
	ts->driver->input_dev = input_allocate_device();
	if (ts->driver->input_dev == NULL) {
		dev_err(dev, "Unable to allocate input device for device %s.\n",
			DRIVER_NAME);
		return -1;
	}

	input_dev = ts->driver->input_dev;

	input_dev->name = DRIVER_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0xaaaa;
	input_dev->id.product = 0x5555;
	input_dev->id.version = 0x0001;

	/* config input device */
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);

	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, TOUCH_POINT_NUM, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     ts->config_info.screen_max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     ts->config_info.screen_max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0,
			     ts->config_info.touch_point_number, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	/* register input device */
	err = input_register_device(input_dev);
	if (err) {
		input_free_device(ts->driver->input_dev);
		ts->driver->input_dev = NULL;
		dev_err(dev, "Unable to register input device for device %s.\n",
			DRIVER_NAME);
		return -1;
	}

	return 0;
}

static int vtl_ts_handler(struct ts_info *data)
{
	int ret;
	struct device *dev;
	struct ts_info *ts;
	DEBUG();

	ts = data;
	dev = &ts->driver->client->dev;

	/* Request platform resources (gpio/interrupt pins) */
	ret = vtl_ts_config(ts);
	if (ret) {
		dev_err(dev, "VTL touch screen config Failed.\n");
		goto ERR_TS_CONFIG;
	}

	vtl_ts_hw_reset();

	ret = chip_init();
	if (ret) {
		dev_err(dev, "vtl ts chip init failed.\n");
		goto ERR_CHIP_INIT;
	}

	/*init input dev*/
	ret = vtl_ts_init_input_dev(ts);
	if (ret) {
		dev_err(dev, "init input dev failed.\n");
		goto ERR_INIT_INPUT;
	}

	ts->config_info.ctp_used = 1;

	return 0;

ERR_INIT_INPUT:
ERR_CHIP_INIT:
ERR_TS_CONFIG:
	ts->config_info.ctp_used = 0;
	return 0;
}

int vtl_ts_probe(struct i2c_client *client)
{
	int err = -1;
	struct ts_info *ts;
	struct device *dev;

	DEBUG();

	ts = pg_ts;
	ts->driver->client = client;
	dev = &ts->driver->client->dev;

	/* Check I2C Functionality */
	err = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!err) {
		dev_err(dev, "Check I2C Functionality Failed.\n");
		return ENODEV;
	}

	err = vtl_ts_handler(ts);
	if (err) {
		dev_err(dev, "input dev init failed: %d", err);
		return err;
	}

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(vtl_pm_ops, vtl_ts_suspend, vtl_ts_resume);

static struct of_device_id vtl_ts_dt_ids[] = { { .compatible = "ct,vtl_ts" },
					       {} };

struct i2c_driver vtl_ts_driver  = {
	
	.driver = {
		.owner	= THIS_MODULE,
		.name	= DRIVER_NAME,
		.pm = pm_sleep_ptr(&vtl_pm_ops),
		.of_match_table = of_match_ptr(vtl_ts_dt_ids),
	},
	.id_table	= vtl_ts_id,
	.probe      	= vtl_ts_probe,
};

int __init vtl_ts_init(void)
{
	DEBUG();
	return i2c_add_driver(&vtl_ts_driver);
}

void __exit vtl_ts_exit(void)
{
	DEBUG();
	i2c_del_driver(&vtl_ts_driver);
}

module_init(vtl_ts_init);
module_exit(vtl_ts_exit);

MODULE_AUTHOR("yangdechu@vtl.com.cn");
MODULE_DESCRIPTION("VTL touchscreen driver for rockchip,V1.0");
MODULE_LICENSE("GPL");
