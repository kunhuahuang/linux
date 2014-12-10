/*
 * STM32F4 LTDC framebuffer
 */

#include <linux/fb.h>
#include <linux/module.h>
#include <linux/platform_device.h>

static struct fb_var_screeninfo stm32_fb_default_screeninfo = {
	.xres = 240,
	.yres = 320,
	.xres_virtual = 240,
	.yres_virtual = 320,
	.xoffset = 0,
	.yoffset = 0,
	.bits_per_pixel = 16,
	.grayscale = 0,
	.red = { 11, 5, 0 },
	.green = { 5, 6, 0 },
	.blue = { 0, 5, 0 },
	.transp = { 0, 0, 0 },
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.vmode = FB_VMODE_NONINTERLACED,
	.rotate = 0,
};

static struct fb_fix_screeninfo stm32_fb_fix_screeninfo = {
	.id = "STM32 LTDC",
	.smem_start = 0x90700000,
	.smem_len   = 0x00100000,
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.xpanstep = 0,
	.ypanstep = 0,
	.ywrapstep = 0,
	.line_length = 480,
	.accel = FB_ACCEL_NONE,
};

static unsigned long stm32_fb_calc_line_length(int xres_virtual, int bpp)
{
	unsigned long len;

	len = xres_virtual * bpp;
	len = (len + 31) & ~31;
	len >>= 3;

	return len;
}

static int stm32_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	unsigned long line_length;

	if (var->vmode & FB_VMODE_CONUPDATE) {
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}

	if (var->xres < 1)
		var->xres = 1;
	if (var->yres < 1)
		var->yres = 1;
	if (var->xres > var->xres_virtual)
		var->xres = var->xres_virtual;
	if (var->yres > var->yres_virtual)
		var->yres = var->yres_virtual;
	if (var->bits_per_pixel < 1)
		var->bits_per_pixel = 1;
	else if (var->bits_per_pixel < 8)
		var->bits_per_pixel = 8;
	else if (var->bits_per_pixel < 16)
		var->bits_per_pixel = 16;
	else if (var->bits_per_pixel < 24)
		var->bits_per_pixel = 24;
	else if (var->bits_per_pixel < 32)
		var->bits_per_pixel = 32;
	else if (var->bits_per_pixel > 32)
		return -EINVAL;
	if (var->xres_virtual < var->xoffset + var->xres)
		var->xres_virtual = var->xoffset + var->xres;
	if (var->yres_virtual < var->yoffset + var->yres)
		var->yres_virtual = var->yoffset + var->yres;

	line_length = stm32_fb_calc_line_length(var->xres_virtual, var->bits_per_pixel);
	if (line_length * var->yres_virtual > info->screen_size)
		return -ENOMEM;

	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:
		if (var->transp.length == 0) {
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 0;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		} else {
			var->red.offset = 11;
			var->red.length = 5;
			var->green.offset = 6;
			var->green.length = 5;
			var->blue.offset = 1;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 1;
		}
		break;
	case 24:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:
		var->red.offset = 24;
		var->red.length = 8;
		var->green.offset = 16;
		var->green.length = 8;
		var->blue.offset = 8;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;

	return 0;
}

static int stm32_fb_set_par(struct fb_info *info)
{
	info->fix.line_length = stm32_fb_calc_line_length(info->var.xres_virtual,
		info->var.bits_per_pixel);

	return 0;
}

static inline u32 stm32_fb_chan_to_field(unsigned channel, struct fb_bitfield *bf)
{
	channel &= 0xffff;
	channel >>= 16 - bf->length;
	return channel << bf->offset;
}

static int stm32_fb_setcolreg(unsigned regno, unsigned red, unsigned green,
	unsigned blue, unsigned transparent, struct fb_info *info)
{
	if (info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
			7471 * blue) >> 16;

	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		if (regno >= 16)
			return -EINVAL;
		else {
			u32 *pseudo_palette = info->pseudo_palette;
			u32 val;

			val  = stm32_fb_chan_to_field(red,   &info->var.red);
			val |= stm32_fb_chan_to_field(green, &info->var.green);
			val |= stm32_fb_chan_to_field(blue,  &info->var.blue);
			pseudo_palette[regno] = val;
		}
		break;
	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
	default:
		return -EINVAL;
	}

	return 0;
}

static int stm32_fb_pan_display(struct fb_var_screeninfo *var,
	struct fb_info *info)
{
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset < 0 || var->yoffset >= info->var.yres_virtual || var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + var->xres > info->var.xres_virtual ||
		    var->yoffset + var->yres > info->var.yres_virtual)
			return -EINVAL;
	}

	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;

	return 0;
}

static struct fb_ops stm32_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = stm32_fb_check_var,
	.fb_set_par   = stm32_fb_set_par,
	.fb_setcolreg = stm32_fb_setcolreg,
	.fb_pan_display = stm32_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int stm32_fb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	int ret;

	info = framebuffer_alloc(sizeof(*info), &pdev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto err_fb_alloc;
	}

	info->screen_base = ioremap(0x90700000, 0x00100000);
	if (!info->screen_base) {
		dev_err(&pdev->dev, "failed to map mem region\n");
		ret = -ENOMEM;
		goto err_ioremap;
	}
	info->screen_size = 0x00100000;
	memset(info->screen_base, 0, info->screen_size);
	info->fbops = &stm32_fb_ops;

	ret = fb_find_mode(&info->var, info, NULL, NULL, 0, NULL, 8);
	//if (!ret || (ret == 4))
		memcpy(&info->var, &stm32_fb_default_screeninfo, sizeof(info->var));
	memcpy(&info->fix, &stm32_fb_fix_screeninfo, sizeof(info->fix));

	info->pseudo_palette = kmalloc(sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_palette_alloc;
	}

	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret < 0)
		goto err_alloc_cmap;

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register framebuffer\n");
		goto err_register;
	}

	platform_set_drvdata(pdev, info);

	return 0;

err_register:
	fb_dealloc_cmap(&info->cmap);
err_alloc_cmap:
	kfree(info->pseudo_palette);
err_palette_alloc:
	iounmap(info->screen_base);
err_ioremap:
	framebuffer_release(info);
err_fb_alloc:
	return ret;
}

static int stm32_fb_remove(struct platform_device *pdev)
{
	return 0;
}

static struct of_device_id stm32_fb_match[] = {
	{ .compatible = "st,stm32f429-ltdc" },
	{ }
};
MODULE_DEVICE_TABLE(of, stm32_fb_match);

static struct platform_driver stm32_fb_driver = {
	.driver = {
		.name = "stm32fb",
		.owner = THIS_MODULE,
		.of_match_table = stm32_fb_match,
	},
	.probe = stm32_fb_probe,
	.remove = stm32_fb_remove,
};

static __init int stm32_fb_init(void)
{
	return platform_driver_register(&stm32_fb_driver);
}
module_init(stm32_fb_init);

static __exit void stm32_fb_exit(void)
{
	platform_driver_unregister(&stm32_fb_driver);
}
module_exit(stm32_fb_exit);

MODULE_LICENSE("GPL");
