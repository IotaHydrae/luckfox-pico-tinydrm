// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix st7735r
 * display controller in SPI mode.
 *
 * Copyright 2025 Zheng Hua <hua.zheng@embeddedboys.com>
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
#include <drm/drm_fourcc.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

static void st7735r_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect);

static void st7735r_pipe_enable(struct drm_simple_display_pipe *pipe,
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
	int idx, ret;
	// u8 addr_mode;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	// ret = mipi_dbi_poweron_reset(dbidev);
	// if (ret) {
	//     DRM_DEBUG_KMS("Failed to reset display!\n");
	// 	goto out_exit;
	// }

	ret = 1;

	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 0);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);

	mipi_dbi_command(dbi, 0x11);
    msleep(20);

    mipi_dbi_command(dbi, 0x36, 0x00);
    mipi_dbi_command(dbi, 0x3a, 0x55);

    mipi_dbi_command(dbi, 0xb1, 0x40, 0x00, 0x00);

    mipi_dbi_command(dbi, 0xc6, 0x05);

    st7735r_fb_dirty(fb, &rect);

    /* Set Display ON */
	mipi_dbi_command(dbi, 0x29);
	backlight_enable(dbidev->backlight);

// out_exit:
	drm_dev_exit(idx);
}

static void st7735r_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);

	/*
	 * This callback is not protected by drm_dev_enter/exit since we want to
	 * turn off the display on regular driver unload. It's highly unlikely
	 * that the underlying SPI controller is gone should this be called after
	 * unplug.
	 */

	DRM_DEBUG_KMS("\n");

	mipi_dbi_command(&dbidev->dbi, MIPI_DCS_SET_DISPLAY_OFF);
}

static int st7735r_buf_copy(void *dst, struct drm_framebuffer *fb,
			               struct drm_rect *clip, bool swap)
{
    struct drm_gem_object *gem = drm_gem_fb_get_obj(fb, 0);
	struct drm_gem_cma_object *cma_obj = to_drm_gem_cma_obj(gem);
	struct dma_buf_attachment *import_attach = gem->import_attach;
	struct drm_format_name_buf format_name;
	void *src = cma_obj->vaddr;
	int ret = 0;

	if (import_attach) {
		ret = dma_buf_begin_cpu_access(import_attach->dmabuf,
					       DMA_FROM_DEVICE);
		if (ret)
			return ret;
	}

	switch (fb->format->format) {
	case DRM_FORMAT_RGB565:
		if (swap)
			drm_fb_swab(dst, src, fb, clip, !import_attach);
		else
			drm_fb_memcpy(dst, src, fb, clip);
		break;
	case DRM_FORMAT_XRGB8888:
		drm_fb_xrgb8888_to_rgb565(dst, src, fb, clip, swap);
		break;
	default:
		drm_err_once(fb->dev, "Format is not supported: %s\n",
			     drm_get_format_name(fb->format->format, &format_name));
		return -EINVAL;
	}

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);
	return ret;
}

static void st7735r_set_addr_win(struct mipi_dbi_dev *dbidev,
               					unsigned int xs, unsigned int xe,
               					unsigned int ys, unsigned int ye)
{
    struct mipi_dbi *dbi = &dbidev->dbi;

	xs += dbidev->left_offset;
	xe += dbidev->left_offset;
	ys += dbidev->top_offset;
	ye += dbidev->top_offset;

	mipi_dbi_command(dbi, MIPI_DCS_SET_COLUMN_ADDRESS, (xs >> 8) & 0xff,
			 xs & 0xff, (xe >> 8) & 0xff, xe & 0xff);
	mipi_dbi_command(dbi, MIPI_DCS_SET_PAGE_ADDRESS, (ys >> 8) & 0xff,
			 ys & 0xff, (ye >> 8) & 0xff, ye & 0xff);
}

static void st7735r_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
   	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
   	unsigned int height = rect->y2 - rect->y1;
	unsigned int width = rect->x2 - rect->x1;
	struct mipi_dbi *dbi = &dbidev->dbi;
    int ret = 0;
	void *tr;

   	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

	ret = st7735r_buf_copy(dbidev->tx_buf, fb, rect, false);
	if (ret)
		goto err_msg;

	tr = dbidev->tx_buf;

	st7735r_set_addr_win(dbidev, rect->x1, rect->x2 - 1, rect->y1, rect->y2 - 1);

	ret = mipi_dbi_command_buf(dbi, MIPI_DCS_WRITE_MEMORY_START, tr,
	                width * height * 2);

err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);
}

static void st7735r_pipe_update(struct drm_simple_display_pipe *pipe,
                               struct drm_plane_state *old_state)
{
    struct drm_plane_state *state = pipe->plane.state;
    struct drm_framebuffer *fb = state->fb;
	struct drm_rect rect, full_rect;
	int idx;

	if (!pipe->crtc.state->active)
		return;

	if (!drm_dev_enter(fb->dev, &idx))
	    return;

	full_rect.x1 = 0;
	full_rect.x2 = fb->width;
	full_rect.y1 = 0;
	full_rect.y2 = fb->height;

	if (drm_atomic_helper_damage_merged(old_state, state, &rect))
		st7735r_fb_dirty(state->fb, &full_rect);

	drm_dev_exit(idx);
}

static const u32 st7735r_formats[] = {
    DRM_FORMAT_RGB565,
    DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs st7735r_pipe_funcs = {
    .enable = st7735r_pipe_enable,
    .disable = st7735r_pipe_disable,
    .update = st7735r_pipe_update,
    .prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7735r_mode = {
    DRM_SIMPLE_MODE(128, 160, 41, 21),
};

DEFINE_DRM_GEM_CMA_FOPS(st7735r_fops);

static struct drm_driver st7735r_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &st7735r_fops,
    DRM_GEM_CMA_DRIVER_OPS_VMAP,
    .debugfs_init = mipi_dbi_debugfs_init,
    .name = "st7735r",
    .desc = "Sitronix st7735r",
    .date = "20250509",
    .major = 1,
    .minor = 0,
};

static const struct of_device_id st7735r_of_match[] = {
    { .compatible = "sitronix,st7567" },
    { },
};
MODULE_DEVICE_TABLE(of, st7735r_of_match);

static const struct spi_device_id st7735r_id[] = {
    { "st7567" },
    { },
};
MODULE_DEVICE_TABLE(spi, st7735r_id);

static int st7735r_probe(struct spi_device *spi)
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

    dbidev = devm_drm_dev_alloc(dev, &st7735r_driver,
                    struct mipi_dbi_dev, drm);
    if (IS_ERR(dbidev))
        return PTR_ERR(dbidev);

    dbi = &dbidev->dbi;
    drm = &dbidev->drm;

    bufsize = (st7735r_mode.vdisplay * st7735r_mode.hdisplay * sizeof(u16));

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

    ret = mipi_dbi_spi_init(spi, dbi, dc);
    if (ret)
        return ret;

    dbi->read_commands = NULL;

    ret = mipi_dbi_dev_init_with_formats(dbidev, &st7735r_pipe_funcs,
        st7735r_formats, ARRAY_SIZE(st7735r_formats),
        &st7735r_mode, rotation, bufsize);
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

static int st7735r_remove(struct spi_device *spi)
{
    struct drm_device *drm = spi_get_drvdata(spi);

    printk("%s\n", __func__);

    drm_dev_unplug(drm);
    drm_atomic_helper_shutdown(drm);

    return 0;
}

static void st7735r_shutdown(struct spi_device *spi)
{
    printk("%s\n", __func__);
    drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7735r_spi_driver = {
    .driver = {
        .name = "st7735r",
        .of_match_table = st7735r_of_match,
    },
    .id_table = st7735r_id,
    .probe = st7735r_probe,
    .remove = st7735r_remove,
    .shutdown = st7735r_shutdown,
};
module_spi_driver(st7735r_spi_driver);

MODULE_DESCRIPTION("Sitronix st7735r DRM driver");
MODULE_AUTHOR("Wooden Chair <hua.zheng@embeddedboys.com>");
MODULE_LICENSE("GPL");
