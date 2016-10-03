/*
 * Driver for OV5642 CMOS Image Sensor from Omnivision
 *
 * Copyright (C) 2011, Bastian Hecht <hechtb@gmail.com>
 *
 * Based on Sony IMX074 Camera Driver
 * Copyright (C) 2010, Guennadi Liakhovetski <g.liakhovetski@gmx.de>
 *
 * Based on Omnivision OV7670 Camera Driver
 * Copyright (C) 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/module.h>
#include <linux/v4l2-mediabus.h>

#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>


#include <media/soc_camera.h>
#include <media/v4l2-clk.h>
#include <media/v4l2-subdev.h>

/* OV5642 registers */
#define REG_CHIP_ID_HIGH		0x300a
#define REG_CHIP_ID_LOW			0x300b

#define REG_WINDOW_START_X_HIGH		0x3800
#define REG_WINDOW_START_X_LOW		0x3801
#define REG_WINDOW_START_Y_HIGH		0x3802
#define REG_WINDOW_START_Y_LOW		0x3803
#define REG_WINDOW_WIDTH_HIGH		0x3804
#define REG_WINDOW_WIDTH_LOW		0x3805
#define REG_WINDOW_HEIGHT_HIGH		0x3806
#define REG_WINDOW_HEIGHT_LOW		0x3807
#define REG_OUT_WIDTH_HIGH		0x3808
#define REG_OUT_WIDTH_LOW		0x3809
#define REG_OUT_HEIGHT_HIGH		0x380a
#define REG_OUT_HEIGHT_LOW		0x380b
#define REG_OUT_TOTAL_WIDTH_HIGH	0x380c
#define REG_OUT_TOTAL_WIDTH_LOW		0x380d
#define REG_OUT_TOTAL_HEIGHT_HIGH	0x380e
#define REG_OUT_TOTAL_HEIGHT_LOW	0x380f
#define REG_OUTPUT_FORMAT		0x4300
#define REG_ISP_CTRL_01			0x5001
#define REG_AVG_WINDOW_END_X_HIGH	0x5682
#define REG_AVG_WINDOW_END_X_LOW	0x5683
#define REG_AVG_WINDOW_END_Y_HIGH	0x5686
#define REG_AVG_WINDOW_END_Y_LOW	0x5687

/* active pixel array size */
#define OV5642_SENSOR_SIZE_X	2592
#define OV5642_SENSOR_SIZE_Y	1944

/*
 * About OV5642 resolution, cropping and binning:
 * This sensor supports it all, at least in the feature description.
 * Unfortunately, no combination of appropriate registers settings could make
 * the chip work the intended way. As it works with predefined register lists,
 * some undocumented registers are presumably changed there to achieve their
 * goals.
 * This driver currently only works for resolutions up to 720 lines with a
 * 1:1 scale. Hopefully these restrictions will be removed in the future.
 */
#define OV5642_MAX_WIDTH	OV5642_SENSOR_SIZE_X
#define OV5642_MAX_HEIGHT	720

/* default sizes */
#define OV5642_DEFAULT_WIDTH	1280
#define OV5642_DEFAULT_HEIGHT	OV5642_MAX_HEIGHT

/* minimum extra blanking */
#define BLANKING_EXTRA_WIDTH		500
#define BLANKING_EXTRA_HEIGHT		20

/*
 * the sensor's autoexposure is buggy when setting total_height low.
 * It tries to expose longer than 1 frame period without taking care of it
 * and this leads to weird output. So we set 1000 lines as minimum.
 */
#define BLANKING_MIN_HEIGHT		1000

struct regval_list {
	u16 reg_num;
	u8 value;
};

static struct regval_list ov5642_default_regs_init[] = {

	{0x3008, 0x42}, /* SYSTEM CTROL0 */
	{0x3103, 0x03}, /* SCCB SYSTEM CTRL1 */
	{0x3017, 0xff}, /* D[9:0] I/O control */
	{0x3018, 0xff}, /* D[9:0] I/O control */
	{0x3034, 0x1a}, /* SC PLL CONTRL0 */
	{0x3037, 0x13}, /* SC PLL CONTRL3 */
	{0x3108, 0x01}, /* SYSTEM ROOT DIVIDER */
	{0x3630, 0x36}, /* ??????? */
	{0x3631, 0x0e}, /* ??????? */
	{0x3632, 0xe2}, /* ??????? */
	{0x3633, 0x12}, /* ??????? */
	{0x3621, 0xe0}, /* ??????? */
	{0x3704, 0xa0}, /*  */
	{0x3703, 0x5a}, /*  */
	{0x3715, 0x78}, /*  */
	{0x3717, 0x01}, /*  */
	{0x370b, 0x60}, /*  */
	{0x3705, 0x1a}, /*  */
	{0x3905, 0x02}, /*  */
	{0x3906, 0x10}, /*  */
	{0x3901, 0x0a}, /*  */
	{0x3731, 0x12}, /*  */
	{0x3600, 0x08}, /*  */
	{0x3601, 0x33}, /*  */
	{0x302d, 0x60}, /*  */
	{0x3620, 0x52}, /*  */
	{0x371b, 0x20}, /*  */
	{0x471c, 0x50}, /*  */
	{0x3a13, 0x43}, /*  */
	{0x3a18, 0x00}, /*  */
	{0x3a19, 0x7c}, /*  */
	{0x3635, 0x13}, /*  */
	{0x3636, 0x03}, /*  */
	{0x3634, 0x40}, /*  */
	{0x3622, 0x01}, /*  */
	{0x3c01, 0x34}, /*  */
	{0x3c04, 0x28}, /*  */
	{0x3c05, 0x98}, /*  */
	{0x3c06, 0x00}, /*  */
	{0x3c07, 0x07}, /*  */
	{0x3c08, 0x00}, /*  */
	{0x3c09, 0x1c}, /*  */
	{0x3c0a, 0x9c}, /*  */
	{0x3c0b, 0x40}, /*  */
	{0x3810, 0x00}, /*  */
	{0x3811, 0x10}, /*  */
	{0x3812, 0x00}, /*  */
	{0x3708, 0x64}, /*  */
	{0x4001, 0x02}, /*  */
	{0x4005, 0x1a}, /*  */
	{0x3000, 0x00}, /*  */
	{0x3004, 0xff}, /*  */
	{0x300e, 0x58}, /*  */
	{0x302e, 0x00}, /*  */
	{0x4300, 0x30}, /*  */
	{0x501f, 0x00}, /*  */
	{0x440e, 0x00}, /*  */
	{0x5000, 0xa7}, /*  */
	{0x3008, 0x02}, /*  */
	{0xffff, 0xff}, /*  */
};


/*ov5640_init_setting_30fps_VGA*/
static struct regval_list ov5642_default_regs_finalise[] = {
	{0x3008, 0x42},
	{0x3103, 0x03}, {0x3017, 0xff}, {0x3018, 0xff},
	{0x3034, 0x1a}, {0x3035, 0x11}, {0x3036, 0x46},
	{0x3037, 0x13}, {0x3108, 0x01}, {0x3630, 0x36},
	{0x3631, 0x0e}, {0x3632, 0xe2}, {0x3633, 0x12},
	{0x3621, 0xe0}, {0x3704, 0xa0}, {0x3703, 0x5a},
	{0x3715, 0x78}, {0x3717, 0x01}, {0x370b, 0x60},
	{0x3705, 0x1a}, {0x3905, 0x02}, {0x3906, 0x10},
	{0x3901, 0x0a}, {0x3731, 0x12}, {0x3600, 0x08},
	{0x3601, 0x33}, {0x302d, 0x60}, {0x3620, 0x52},
	{0x371b, 0x20}, {0x471c, 0x50}, {0x3a13, 0x43},
	{0x3a18, 0x00}, {0x3a19, 0xf8}, {0x3635, 0x13},
	{0x3636, 0x03}, {0x3634, 0x40}, {0x3622, 0x01},
	{0x3c01, 0x34}, {0x3c04, 0x28}, {0x3c05, 0x98},
	{0x3c06, 0x00}, {0x3c07, 0x08}, {0x3c08, 0x00},
	{0x3c09, 0x1c}, {0x3c0a, 0x9c}, {0x3c0b, 0x40},
	{0x3820, 0x41}, {0x3821, 0x07}, {0x3814, 0x31},
	{0x3815, 0x31}, {0x3800, 0x00}, {0x3801, 0x00},
	{0x3802, 0x00}, {0x3803, 0x04}, {0x3804, 0x0a},
	{0x3805, 0x3f}, {0x3806, 0x07}, {0x3807, 0x9b},
	{0x3808, 0x02}, {0x3809, 0x80}, {0x380a, 0x01},
	{0x380b, 0xe0}, {0x380c, 0x07}, {0x380d, 0x68},
	{0x380e, 0x03}, {0x380f, 0xd8}, {0x3810, 0x00},
	{0x3811, 0x10}, {0x3812, 0x00}, {0x3813, 0x06},
	{0x3618, 0x00}, {0x3612, 0x29}, {0x3708, 0x64},
	{0x3709, 0x52}, {0x370c, 0x03}, {0x3a02, 0x03},
	{0x3a03, 0xd8}, {0x3a08, 0x01}, {0x3a09, 0x27},
	{0x3a0a, 0x00}, {0x3a0b, 0xf6}, {0x3a0e, 0x03},
	{0x3a0d, 0x04}, {0x3a14, 0x03}, {0x3a15, 0xd8},
	{0x4001, 0x02}, {0x4004, 0x02}, {0x3000, 0x00},
	{0x3002, 0x1c}, {0x3004, 0xff}, {0x3006, 0xc3},
	{0x300e, 0x58}, {0x302e, 0x00}, {0x4300, 0x30},
	{0x501f, 0x00}, {0x4713, 0x03}, {0x4407, 0x04},
	{0x440e, 0x00}, {0x460b, 0x35}, {0x460c, 0x22},
	{0x4837, 0x22}, {0x3824, 0x02}, {0x5000, 0xa7},
	{0x5001, 0xa3}, {0x5180, 0xff}, {0x5181, 0xf2},
	{0x5182, 0x00}, {0x5183, 0x14}, {0x5184, 0x25},
	{0x5185, 0x24}, {0x5186, 0x09}, {0x5187, 0x09},
	{0x5188, 0x09}, {0x5189, 0x88}, {0x518a, 0x54},
	{0x518b, 0xee}, {0x518c, 0xb2}, {0x518d, 0x50},
	{0x518e, 0x34}, {0x518f, 0x6b}, {0x5190, 0x46},
	{0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
	{0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03},
	{0x5197, 0x01}, {0x5198, 0x04}, {0x5199, 0x6c},
	{0x519a, 0x04}, {0x519b, 0x00}, {0x519c, 0x09},
	{0x519d, 0x2b}, {0x519e, 0x38}, {0x5381, 0x1e},
	{0x5382, 0x5b}, {0x5383, 0x08}, {0x5384, 0x0a},
	{0x5385, 0x7e}, {0x5386, 0x88}, {0x5387, 0x7c},
	{0x5388, 0x6c}, {0x5389, 0x10}, {0x538a, 0x01},
	{0x538b, 0x98}, {0x5300, 0x08}, {0x5301, 0x30},
	{0x5302, 0x10}, {0x5303, 0x00}, {0x5304, 0x08},
	{0x5305, 0x30}, {0x5306, 0x08}, {0x5307, 0x16},
	{0x5309, 0x08}, {0x530a, 0x30}, {0x530b, 0x04},
	{0x530c, 0x06}, {0x5480, 0x01}, {0x5481, 0x08},
	{0x5482, 0x14}, {0x5483, 0x28}, {0x5484, 0x51},
	{0x5485, 0x65}, {0x5486, 0x71}, {0x5487, 0x7d},
	{0x5488, 0x87}, {0x5489, 0x91}, {0x548a, 0x9a},
	{0x548b, 0xaa}, {0x548c, 0xb8}, {0x548d, 0xcd},
	{0x548e, 0xdd}, {0x548f, 0xea}, {0x5490, 0x1d},
	{0x5580, 0x02}, {0x5583, 0x40}, {0x5584, 0x10},
	{0x5589, 0x10}, {0x558a, 0x00}, {0x558b, 0xf8},
	{0x5800, 0x23}, {0x5801, 0x14}, {0x5802, 0x0f},
	{0x5803, 0x0f}, {0x5804, 0x12}, {0x5805, 0x26},
	{0x5806, 0x0c}, {0x5807, 0x08}, {0x5808, 0x05},
	{0x5809, 0x05}, {0x580a, 0x08}, {0x580b, 0x0d},
	{0x580c, 0x08}, {0x580d, 0x03}, {0x580e, 0x00},
	{0x580f, 0x00}, {0x5810, 0x03}, {0x5811, 0x09},
	{0x5812, 0x07}, {0x5813, 0x03}, {0x5814, 0x00},
	{0x5815, 0x01}, {0x5816, 0x03}, {0x5817, 0x08},
	{0x5818, 0x0d}, {0x5819, 0x08}, {0x581a, 0x05},
	{0x581b, 0x06}, {0x581c, 0x08}, {0x581d, 0x0e},
	{0x581e, 0x29}, {0x581f, 0x17}, {0x5820, 0x11},
	{0x5821, 0x11}, {0x5822, 0x15}, {0x5823, 0x28},
	{0x5824, 0x46}, {0x5825, 0x26}, {0x5826, 0x08},
	{0x5827, 0x26}, {0x5828, 0x64}, {0x5829, 0x26},
	{0x582a, 0x24}, {0x582b, 0x22}, {0x582c, 0x24},
	{0x582d, 0x24}, {0x582e, 0x06}, {0x582f, 0x22},
	{0x5830, 0x40}, {0x5831, 0x42}, {0x5832, 0x24},
	{0x5833, 0x26}, {0x5834, 0x24}, {0x5835, 0x22},
	{0x5836, 0x22}, {0x5837, 0x26}, {0x5838, 0x44},
	{0x5839, 0x24}, {0x583a, 0x26}, {0x583b, 0x28},
	{0x583c, 0x42}, {0x583d, 0xce}, {0x5025, 0x00},
	{0x3a0f, 0x30}, {0x3a10, 0x28}, {0x3a1b, 0x30},
	{0x3a1e, 0x26}, {0x3a11, 0x60}, {0x3a1f, 0x14},
	{0x3008, 0x02}, {0x3034, 0x1a}, {0x3035, 0x11},
	{0x3036, 0x46}, {0x3037, 0x13}, {0xffff, 0xff}, /*  */
};

struct ov5642_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct ov5642 {
	struct v4l2_subdev		subdev;
	struct v4l2_ctrl_handler	hdl;
	const struct ov5642_datafmt	*fmt;
	struct v4l2_rect                crop_rect;
	struct v4l2_clk			*clk;

	/* blanking information */
	int total_width;
	int total_height;

	struct soc_camera_subdev_desc	ssdd_dt;
	struct gpio_desc *resetb_gpio;
	struct gpio_desc *pwdn_gpio;
};

static const struct ov5642_datafmt ov5642_colour_fmts[] = {
	{MEDIA_BUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov5642 *to_ov5642(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov5642, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ov5642_datafmt
			*ov5642_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov5642_colour_fmts); i++)
		if (ov5642_colour_fmts[i].code == code)
			return ov5642_colour_fmts + i;

	return NULL;
}

static int reg_read(struct i2c_client *client, u16 reg, u8 *val)
{
	int ret;
	/* We have 16-bit i2c addresses - care for endianness */
	unsigned char data[2] = { reg >> 8, reg & 0xff };

	ret = i2c_master_send(client, data, 2);
	if (ret < 2) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	ret = i2c_master_recv(client, val, 1);
	if (ret < 1) {
		dev_err(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int reg_write(struct i2c_client *client, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val };

	ret = i2c_master_send(client, data, 3);
	if (ret < 3) {
		dev_err(&client->dev, "%s: i2c write error, reg: %x\n",
			__func__, reg);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

/*
 * convenience function to write 16 bit register values that are split up
 * into two consecutive high and low parts
 */
static int reg_write16(struct i2c_client *client, u16 reg, u16 val16)
{
	int ret;

	ret = reg_write(client, reg, val16 >> 8);
	if (ret)
		return ret;
	return reg_write(client, reg + 1, val16 & 0x00ff);
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5642_get_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = reg_read(client, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ov5642_set_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return reg_write(client, reg->reg, reg->val);
}
#endif

static int ov5642_write_array(struct i2c_client *client,
				struct regval_list *vals)
{
	while (vals->reg_num != 0xffff || vals->value != 0xff) {
		int ret = reg_write(client, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	dev_dbg(&client->dev, "Register list loaded\n");
	return 0;
}

static int ov5642_set_resolution(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *priv = to_ov5642(client);
	int width = priv->crop_rect.width;
	int height = priv->crop_rect.height;
	int total_width = priv->total_width;
	int total_height = priv->total_height;
	int start_x = (OV5642_SENSOR_SIZE_X - width) / 2;
	int start_y = (OV5642_SENSOR_SIZE_Y - height) / 2;
	int ret;

	/*
	 * This should set the starting point for cropping.
	 * Doesn't work so far.
	 */
	ret = reg_write16(client, REG_WINDOW_START_X_HIGH, start_x);
	if (!ret)
		ret = reg_write16(client, REG_WINDOW_START_Y_HIGH, start_y);
	if (!ret) {
		priv->crop_rect.left = start_x;
		priv->crop_rect.top = start_y;
	}

	if (!ret)
		ret = reg_write16(client, REG_WINDOW_WIDTH_HIGH, width);
	if (!ret)
		ret = reg_write16(client, REG_WINDOW_HEIGHT_HIGH, height);
	if (ret)
		return ret;
	priv->crop_rect.width = width;
	priv->crop_rect.height = height;

	/* Set the output window size. Only 1:1 scale is supported so far. */
	ret = reg_write16(client, REG_OUT_WIDTH_HIGH, width);
	if (!ret)
		ret = reg_write16(client, REG_OUT_HEIGHT_HIGH, height);

	/* Total width = output size + blanking */
	if (!ret)
		ret = reg_write16(client, REG_OUT_TOTAL_WIDTH_HIGH, total_width);
	if (!ret)
		ret = reg_write16(client, REG_OUT_TOTAL_HEIGHT_HIGH, total_height);

	/* Sets the window for AWB calculations */
	if (!ret)
		ret = reg_write16(client, REG_AVG_WINDOW_END_X_HIGH, width);
	if (!ret)
		ret = reg_write16(client, REG_AVG_WINDOW_END_Y_HIGH, height);

	return ret;
}

static int ov5642_set_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *priv = to_ov5642(client);
	const struct ov5642_datafmt *fmt = ov5642_find_datafmt(mf->code);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->crop_rect.width;
	mf->height = priv->crop_rect.height;

	if (!fmt) {
		if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
			return -EINVAL;
		mf->code	= ov5642_colour_fmts[0].code;
		mf->colorspace	= ov5642_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		priv->fmt = ov5642_find_datafmt(mf->code);
	else
		cfg->try_fmt = *mf;
	return 0;
}

static int ov5642_get_fmt(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *priv = to_ov5642(client);

	const struct ov5642_datafmt *fmt = priv->fmt;

	if (format->pad)
		return -EINVAL;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->width	= priv->crop_rect.width;
	mf->height	= priv->crop_rect.height;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov5642_enum_mbus_code(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ov5642_colour_fmts))
		return -EINVAL;

	code->code = ov5642_colour_fmts[code->index].code;
	return 0;
}

static int ov5642_s_crop(struct v4l2_subdev *sd, const struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *priv = to_ov5642(client);
	struct v4l2_rect rect = a->c;
	int ret;

	v4l_bound_align_image(&rect.width, 48, OV5642_MAX_WIDTH, 1,
			      &rect.height, 32, OV5642_MAX_HEIGHT, 1, 0);

	priv->crop_rect.width	= rect.width;
	priv->crop_rect.height	= rect.height;
	priv->total_width	= rect.width + BLANKING_EXTRA_WIDTH;
	priv->total_height	= max_t(int, rect.height +
							BLANKING_EXTRA_HEIGHT,
							BLANKING_MIN_HEIGHT);
	priv->crop_rect.width		= rect.width;
	priv->crop_rect.height		= rect.height;

	ret = ov5642_write_array(client, ov5642_default_regs_init);
	/*if (!ret)
		ret = ov5642_set_resolution(sd);*/
	if (!ret)
		ret = ov5642_write_array(client, ov5642_default_regs_finalise);

	return ret;
}

static int ov5642_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *priv = to_ov5642(client);
	struct v4l2_rect *rect = &a->c;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	*rect = priv->crop_rect;

	return 0;
}

static int ov5642_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left			= 0;
	a->bounds.top			= 0;
	a->bounds.width			= OV5642_MAX_WIDTH;
	a->bounds.height		= OV5642_MAX_HEIGHT;
	a->defrect			= a->bounds;
	a->type				= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int ov5642_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->type = V4L2_MBUS_CSI2;
	cfg->flags = V4L2_MBUS_CSI2_2_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
					V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	return 0;
}

static int ov5642_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct ov5642 *priv = to_ov5642(client);
	int ret;
	printk(KERN_DEBUG "OV5642 Power 1 %d",on);

	ret = soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
	
	/*fix how this reyurn on power off*/

	/*printk(KERN_DEBUG "OV5642 Power 1 %d",on);

	if (!on)
		return soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
		return soc_camera_power_off(&client->dev, ssdd, priv->clk);

	printk(KERN_DEBUG "OV5642 Power 2");

	ret = soc_camera_set_power(&client->dev, ssdd, priv->clk, on);
		soc_camera_power_on(&client->dev, ssdd, priv->clk);
	if (ret < 0)
		return ret;

	printk(KERN_DEBUG "OV5642 Power 3");

	return -1;*/

	printk(KERN_DEBUG "OV5642 Power 2");	

	i2c_smbus_write_byte_data(client, 0x30, 0x30);
	i2c_smbus_read_byte_data(client, 0x30);

	ret = ov5642_write_array(client, ov5642_default_regs_init);
	
	printk(KERN_DEBUG "OV5642 Power 3 %d",ret);	
	
	/*if (!ret) {
		ret = ov5642_set_resolution(sd);
		printk(KERN_DEBUG "OV5642 Power 4");
	}*/
	if (!ret) {
		ret = ov5642_write_array(client, ov5642_default_regs_finalise);
		printk(KERN_DEBUG "OV5642 Power 5");
	}
	return ret;
}

static struct v4l2_subdev_video_ops ov5642_subdev_video_ops = {
	.s_crop		= ov5642_s_crop,
	.g_crop		= ov5642_g_crop,
	.cropcap	= ov5642_cropcap,
	.g_mbus_config	= ov5642_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov5642_subdev_pad_ops = {
	.enum_mbus_code = ov5642_enum_mbus_code,
	.get_fmt	= ov5642_get_fmt,
	.set_fmt	= ov5642_set_fmt,
};

static struct v4l2_subdev_core_ops ov5642_subdev_core_ops = {
	.s_power	= ov5642_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov5642_get_register,
	.s_register	= ov5642_set_register,
#endif
};

static struct v4l2_subdev_ops ov5642_subdev_ops = {
	.core	= &ov5642_subdev_core_ops,
	.video	= &ov5642_subdev_video_ops,
	.pad	= &ov5642_subdev_pad_ops,
};

static int ov5642_video_probe(struct i2c_client *client)
{
	/*struct v4l2_subdev *subdev = i2c_get_clientdata(client);*/
	struct ov5642 *priv = to_ov5642(client);
	int ret;
	u8 id_high, id_low;
	u16 id;

	printk(KERN_DEBUG "OV5642 Video Probe 1 %x %s", client->addr, client->name);
	printk(KERN_DEBUG "OV5642 Video Probe 1a %d %s", client->adapter->nr, client->adapter->name);

	ret = ov5642_s_power(&priv->subdev, 1);
	if (ret < 0)
		return ret;

	printk(KERN_DEBUG "OV5642 Video Probe 2");

	/*i2c_smbus_write_byte_data(client, 0x30, 0x30);
	i2c_smbus_read_byte_data(client, 0x30);*/

	/* Read sensor Model ID */
	ret = reg_read(client, REG_CHIP_ID_HIGH, &id_high);
	if (ret < 0)
		goto done;
	
	id = id_high << 8;

	printk(KERN_DEBUG "OV5642 Video Probe 3");

	ret = reg_read(client, REG_CHIP_ID_LOW, &id_low);
	if (ret < 0)
		goto done;

	id |= id_low;

	dev_info(&client->dev, "Chip ID 0x%04x detected\n", id);

	if (id != 0x5642) {
		ret = -ENODEV;
		goto done;
	}

	ret = 0;

done:
	/*(ov5642_s_power(subdev, 0);
	ov5642_s_power(&priv->subdev, 0);*/
	return ret;
}

/* OF probe functions */
static int ov5642_hw_power(struct device *dev, int on)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ov5642 *priv = to_ov5642(client);

	dev_dbg(&client->dev, "%s: %s the camera\n",
			__func__, on ? "ENABLE" : "DISABLE");

	if (priv->pwdn_gpio)
		gpiod_direction_output(priv->pwdn_gpio, !on);

	return 0;
}

static int ov5642_hw_reset(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ov5642 *priv = to_ov5642(client);

	if (priv->resetb_gpio) {
		/* Active the resetb pin to perform a reset pulse */
		gpiod_direction_output(priv->resetb_gpio, 1);
		usleep_range(3000, 5000);
		gpiod_direction_output(priv->resetb_gpio, 0);
	}

	return 0;
}


static int ov5642_probe_dt(struct i2c_client *client,
		struct ov5642 *priv)
{
	printk(KERN_DEBUG "OV5642 DT Probe 1");

	/* Request the reset GPIO deasserted */
	priv->resetb_gpio = devm_gpiod_get_optional(&client->dev, "resetb",
			GPIOD_OUT_LOW);
	if (!priv->resetb_gpio)
		dev_dbg(&client->dev, "resetb gpio is not assigned!\n");
	else if (IS_ERR(priv->resetb_gpio))
		return PTR_ERR(priv->resetb_gpio);

	printk(KERN_DEBUG "OV5642 DT Probe 2");

	/* Request the power down GPIO asserted */
	priv->pwdn_gpio = devm_gpiod_get_optional(&client->dev, "pwdn",
			GPIOD_OUT_HIGH);
	if (!priv->pwdn_gpio)
		dev_dbg(&client->dev, "pwdn gpio is not assigned!\n");
	else if (IS_ERR(priv->pwdn_gpio))
		return PTR_ERR(priv->pwdn_gpio);

	printk(KERN_DEBUG "OV5642 DT Probe 3");

	/* Initialize the soc_camera_subdev_desc */
	priv->ssdd_dt.power = ov5642_hw_power;
	priv->ssdd_dt.reset = ov5642_hw_reset;
	client->dev.platform_data = &priv->ssdd_dt;

	printk(KERN_DEBUG "OV5642 DT Probe 4");

	return 0;
}



static int ov5642_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	struct ov5642 *priv;
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	printk(KERN_DEBUG "OV5642 Probe 1 %x %s", client->addr, client->name);
	printk(KERN_DEBUG "OV5642 Probe 1a %d %s", client->adapter->nr, client->adapter->name);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&adapter->dev,
			"OV5642: I2C-Adapter doesn't support SMBUS\n");
		return -EIO;
	}

	printk(KERN_DEBUG "OV5642 Probe 2");

	priv = devm_kzalloc(&client->dev, sizeof(struct ov5642), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	printk(KERN_DEBUG "OV5642 Probe 3");	

	priv->clk = v4l2_clk_get(&client->dev, "xvclk");
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);

	printk(KERN_DEBUG "OV5642 Probe 4");


	if (!ssdd && !client->dev.of_node) {
		dev_err(&client->dev, "OV5642: missing platform data!\n");
		return -EINVAL;
	}
	
	printk(KERN_DEBUG "OV5642 Probe 5");	

	if (!ssdd) {
		ret = ov5642_probe_dt(client, priv);
		if (ret)
			return -EINVAL;
	}	

	printk(KERN_DEBUG "OV5642 Probe 6");	

	v4l2_i2c_subdev_init(&priv->subdev, client, &ov5642_subdev_ops);
	v4l2_ctrl_handler_init(&priv->hdl, 2);

	priv->subdev.ctrl_handler = &priv->hdl;
	if (priv->hdl.error) {
		return priv->hdl.error;		
	}

	priv->fmt		= &ov5642_colour_fmts[0];

	priv->crop_rect.width	= OV5642_DEFAULT_WIDTH;
	priv->crop_rect.height	= OV5642_DEFAULT_HEIGHT;
	priv->crop_rect.left	= (OV5642_MAX_WIDTH - OV5642_DEFAULT_WIDTH) / 2;
	priv->crop_rect.top	= (OV5642_MAX_HEIGHT - OV5642_DEFAULT_HEIGHT) / 2;
	priv->total_width = OV5642_DEFAULT_WIDTH + BLANKING_EXTRA_WIDTH;
	priv->total_height = BLANKING_MIN_HEIGHT;

	printk(KERN_DEBUG "OV5642 Probe 7");

	
	ret = ov5642_video_probe(client);
	if (ret < 0)
		v4l2_clk_put(priv->clk);

	return ret;
}

static int ov5642_remove(struct i2c_client *client)
{
	struct soc_camera_subdev_desc *ssdd = soc_camera_i2c_to_desc(client);
	struct ov5642 *priv = to_ov5642(client);

	v4l2_clk_put(priv->clk);
	if (ssdd->free_bus)
		ssdd->free_bus(ssdd);

	return 0;
}

static const struct i2c_device_id ov5642_id[] = {
	{ "ov5642", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov5642_id);

static const struct of_device_id ov5642_of_match[] = {
	{.compatible = "ovti,ov5642", },
	{},
};


static struct i2c_driver ov5642_i2c_driver = {
	.driver = {
		.name = "ov5642",
	},
	.probe		= ov5642_probe,
	.remove		= ov5642_remove,
	.id_table	= ov5642_id,
};

module_i2c_driver(ov5642_i2c_driver);

MODULE_DESCRIPTION("Omnivision OV5642 Camera driver");
MODULE_AUTHOR("Bastian Hecht <hechtb@gmail.com>");
MODULE_LICENSE("GPL v2");
