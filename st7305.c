// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix st7305
 * display controller in SPI mode.
 *
 * Copyright 2025 Wooden Chair <hua.zheng@embeddedboys.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/dma-buf.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

static u8 *g_st7305_dgram = NULL;

static void st7305_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect);

static inline void st7305_reset(struct mipi_dbi *dbi)
{
	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 0);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);
}

static void st7305_pipe_enable(struct drm_simple_display_pipe *pipe,
			       struct drm_crtc_state *crtc_state,
			       struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct drm_framebuffer *fb = plane_state->fb;
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct drm_rect rect = {
		.x1 = 0,
		.x2 = fb->width,
		.y1 = 0,
		.y2 = fb->height,
	};
	int idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	st7305_reset(dbi);

	mipi_dbi_command(dbi, 0xD6, 0x13, 0x02); // NVM Load Control
	mipi_dbi_command(dbi, 0xD1, 0x01); // Booster Enable
	mipi_dbi_command(dbi, 0xC0, 0x08, 0x06); // Gate Voltage Setting

	mipi_dbi_command(dbi, 0xC1, 0x3C, 0x3E, 0x3C, 0x3C);
	mipi_dbi_command(dbi, 0xC2, 0x23, 0x21, 0x23, 0x23);
	mipi_dbi_command(dbi, 0xC4, 0x5A, 0x5C, 0x5A, 0x5A);
	mipi_dbi_command(dbi, 0xC5, 0x37, 0x35, 0x37, 0x37);

	mipi_dbi_command(dbi, 0xD8, 0x80, 0xE9);

	mipi_dbi_command(dbi, 0xB2, 0x02);

	mipi_dbi_command(dbi, 0xB3, 0xE5, 0xF6, 0x17, 0x77, 0x77, 0x77, 0x77,
			 0x77, 0x77, 0x71);
	mipi_dbi_command(dbi, 0xB4, 0x05, 0x46, 0x77, 0x77, 0x77, 0x77, 0x76,
			 0x45);
	mipi_dbi_command(dbi, 0x62, 0x32, 0x03, 0x1F);

	mipi_dbi_command(dbi, 0xB7, 0x13);
	mipi_dbi_command(dbi, 0xB0, 0x60);

	mipi_dbi_command(dbi, 0x11);
	msleep(120);

	mipi_dbi_command(dbi, 0xC9, 0x00);
	mipi_dbi_command(dbi, 0x36, 0x00);
	mipi_dbi_command(dbi, 0x3A, 0x11);
	mipi_dbi_command(dbi, 0xB9, 0x20);
	mipi_dbi_command(dbi, 0xB8, 0x29);
	mipi_dbi_command(dbi, 0x2A, 0x17, 0x24);
	mipi_dbi_command(dbi, 0x2B, 0x00, 0xBF);
	mipi_dbi_command(dbi, 0xD0, 0xFF);
	mipi_dbi_command(dbi, 0x38);
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_ON);

	st7305_fb_dirty(fb, &rect);

	drm_dev_exit(idx);
}

static void st7305_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	DRM_DEBUG_KMS("\n");
	mipi_dbi_command(dbi, MIPI_DCS_SET_DISPLAY_OFF);
}

static void __st7305_convert_buffer(u8 *dst, void *vaddr,
				    struct drm_framebuffer *fb)
{
	u8 b1, b2, mix, y;
	u8 *buf8 = vaddr;
	u16 i, j, k = 0;
	for (i = 0; i < fb->height; i += 2) {
		// Convert 2 columns
		for (j = 0; j < 21; j += 3) {
			for (y = 0; y < 3; y++) {
				b1 = buf8[(j + y) * fb->height + i];
				b2 = buf8[(j + y) * fb->height + i + 1];

				// First 4 bits
				mix = 0;
				mix |= ((b1 & 0x01) << 7);
				mix |= ((b2 & 0x01) << 6);
				mix |= ((b1 & 0x02) << 4);
				mix |= ((b2 & 0x02) << 3);
				mix |= ((b1 & 0x04) << 1);
				mix |= ((b2 & 0x04) << 0);
				mix |= ((b1 & 0x08) >> 2);
				mix |= ((b2 & 0x08) >> 3);
				dst[k++] = mix;

				// Second 4 bits
				b1 >>= 4;
				b2 >>= 4;
				mix = 0;
				mix |= ((b1 & 0x01) << 7);
				mix |= ((b2 & 0x01) << 6);
				mix |= ((b1 & 0x02) << 4);
				mix |= ((b2 & 0x02) << 3);
				mix |= ((b1 & 0x04) << 1);
				mix |= ((b2 & 0x04) << 0);
				mix |= ((b1 & 0x08) >> 2);
				mix |= ((b2 & 0x08) >> 3);
				dst[k++] = mix;
			}
		}
	}
}

static void __st7305_put_pixel(int x, int y, u8 *dgram, u8 *buf,
			       struct drm_framebuffer *fb, u8 rotation)
{
	u8 new_x, new_y;
	u16 byte_idx;
	u8 bit_pos;

	switch (rotation) {
	case 1: // 90 degrees
		new_x = fb->height - y;
		new_y = x;
		break;
	case 2: // 180 degrees
		new_x = fb->width - x;
		new_y = fb->height - y;
		break;
	case 3: // 270 degrees
		new_x = y;
		new_y = fb->width - x;
		break;
	default: // 0 degrees
		new_x = x;
		new_y = y;
		break;
	}

	byte_idx = (new_y >> 3) * fb->height + new_x;
	bit_pos = new_y & 0x07;

	if (*buf > 128)
		dgram[byte_idx] |= (1 << bit_pos);
	else
		dgram[byte_idx] &= ~(1 << bit_pos);
}

static void st7305_xrgb8888_to_monochrome(u8 *dst, void *vaddr,
					  struct drm_framebuffer *fb,
					  struct drm_rect *clip)
{
	size_t len = (clip->x2 - clip->x1) * (clip->y2 - clip->y1);
	unsigned int x, y;
	u8 *buf;

	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	drm_fb_xrgb8888_to_gray8(buf, vaddr, fb, clip);

	for (y = clip->y1; y < clip->y2; y++) {
		for (x = clip->x1; x < clip->x2; x++) {
			__st7305_put_pixel(x, y, g_st7305_dgram, buf++, fb, 3);
		}
	}
	__st7305_convert_buffer(dst, g_st7305_dgram, fb);

	kfree(buf);
}

static int st7305_buf_copy(void *dst, struct drm_framebuffer *fb,
			   struct drm_rect *clip)
{
	struct drm_gem_cma_object *cma_obj = drm_fb_cma_get_gem_obj(fb, 0);
	struct dma_buf_attachment *import_attach = cma_obj->base.import_attach;
	void *src = cma_obj->vaddr;
	int ret = 0;

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
					       DMA_FROM_DEVICE);
		if (ret)
			return ret;
	}

	st7305_xrgb8888_to_monochrome(dst, src, fb, clip);

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);

	return ret;
}

static void st7305_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	struct spi_device *spi = dbi->spi;
	int ret = 0;

	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id,
		      DRM_RECT_ARG(rect));

	ret = st7305_buf_copy(dbidev->tx_buf, fb, rect);
	if (ret)
		goto err_msg;

	mipi_dbi_command(dbi, 0x2A, 0x17, 0x17 + 14 - 1);
	mipi_dbi_command(dbi, 0x2B, 0x00, 0x00 + fb->width + 24 - 1);
	mipi_dbi_command(dbi, 0x2C);

	gpiod_set_value_cansleep(dbi->dc, 1);
	mipi_dbi_spi_transfer(spi, spi->max_speed_hz, 8, dbidev->tx_buf,
			      (fb->width + 24) * 14 * 3);

err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n",
			     ret);
}

static void st7305_pipe_update(struct drm_simple_display_pipe *pipe,
			       struct drm_plane_state *old_state)
{
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_framebuffer *fb = state->fb;
	struct drm_rect rect;
	int idx;

	if (!pipe->crtc.state->active)
		return;

	if (!drm_dev_enter(fb->dev, &idx))
		return;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		st7305_fb_dirty(state->fb, &rect);

	drm_dev_exit(idx);
}

static const u32 st7305_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs st7305_pipe_funcs = {
	.enable = st7305_pipe_enable,
	.disable = st7305_pipe_disable,
	.update = st7305_pipe_update,
	.prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7305_mode = {
	DRM_SIMPLE_MODE(168, 384, 55, 90),
};

DEFINE_DRM_GEM_CMA_FOPS(st7305_fops);

static struct drm_driver st7305_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops = &st7305_fops,
	DRM_GEM_CMA_DRIVER_OPS_VMAP,
	.debugfs_init = mipi_dbi_debugfs_init,
	.name = "st7305",
	.desc = "Sitronix st7305",
	.date = "20250509",
	.major = 1,
	.minor = 0,
};

static const struct of_device_id st7305_of_match[] = {
	{ .compatible = "sitronix,st7567" },
	{},
};
MODULE_DEVICE_TABLE(of, st7305_of_match);

static const struct spi_device_id st7305_id[] = {
	{ "st7567" },
	{},
};
MODULE_DEVICE_TABLE(spi, st7305_id);

static int st7305_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	u32 rotation = 0;
	size_t bufsize;
	int ret;

	printk("%s\n", __func__);

	bufsize = (st7305_mode.hdisplay + 24) * 14 * 3;
	g_st7305_dgram = devm_kzalloc(dev, bufsize, GFP_KERNEL);
	if (!g_st7305_dgram) {
		dev_err(dev, "Failed to allocate memory for g_st7305_dgram\n");
		return -ENOMEM;
	}

	dbidev = devm_drm_dev_alloc(dev, &st7305_driver, struct mipi_dbi_dev,
				    drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	bufsize = (st7305_mode.vdisplay * st7305_mode.hdisplay * sizeof(u16));

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(dbi->reset)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'reset'\n");
		return PTR_ERR(dbi->reset);
	}

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc)) {
		DRM_DEV_ERROR(dev, "Failed to get gpio 'dc'\n");
		return PTR_ERR(dc);
	}

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight)) {
		DRM_DEV_ERROR(dev, "Failed to get backlight device\n");
		return PTR_ERR(dbidev->backlight);
	}

	device_property_read_u32(dev, "rotation", &rotation);
	printk("rotation: %d\n", rotation);
	printk("spi max frequency: %d (Hz)\n", spi->max_speed_hz);

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init_with_formats(dbidev, &st7305_pipe_funcs,
					     st7305_formats,
					     ARRAY_SIZE(st7305_formats),
					     &st7305_mode, rotation, bufsize);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	drm_fbdev_generic_setup(drm, 0);

	return 0;
}

static int st7305_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	printk("%s\n", __func__);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	return 0;
}

static void st7305_shutdown(struct spi_device *spi)
{
	printk("%s\n", __func__);
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7305_spi_driver = {
    .driver =
        {
            .name = "st7305",
            .of_match_table = st7305_of_match,
        },
    .id_table = st7305_id,
    .probe = st7305_probe,
    .remove = st7305_remove,
    .shutdown = st7305_shutdown,
};
module_spi_driver(st7305_spi_driver);

MODULE_DESCRIPTION("Sitronix st7305 DRM driver");
MODULE_AUTHOR("Wooden Chair <hua.zheng@embeddedboys.com>");
MODULE_LICENSE("GPL");
