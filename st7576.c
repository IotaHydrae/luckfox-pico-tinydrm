// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix st7576
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
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>
#include <drm/drm_rect.h>

static void st7576_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect);

static void st7576_pipe_enable(struct drm_simple_display_pipe *pipe,
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

	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 0);
	msleep(10);
	gpiod_set_raw_value(dbi->reset, 1);
	msleep(10);

	mipi_dbi_command(dbi, 0xe2);
    msleep(20);

    mipi_dbi_command(dbi, 0xa0);
    mipi_dbi_command(dbi, 0xc8);
    mipi_dbi_command(dbi, 0xa2);
    mipi_dbi_command(dbi, 0x2f);
    mipi_dbi_command(dbi, 0x25);
    mipi_dbi_command(dbi, 0x81);
    mipi_dbi_command(dbi, 0x19);
    mipi_dbi_command(dbi, 0x40);

    st7576_fb_dirty(fb, &rect);

    /* Set Display ON */
	mipi_dbi_command(dbi, 0xaf);
	backlight_enable(dbidev->backlight);

	drm_dev_exit(idx);
}

static void st7576_pipe_disable(struct drm_simple_display_pipe *pipe)
{
    DRM_DEBUG_KMS("\n");
}

static void st7576_xrgb8888_to_monochrome(u8 *dst, void *vaddr,
				       struct drm_framebuffer *fb,
				       struct drm_rect *clip)
{
    size_t len = (clip->x2 - clip->x1) * (clip->y2 - clip->y1);
    unsigned int x, y;
    u8 *src, *buf;

   	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;

	drm_fb_xrgb8888_to_gray8(buf, vaddr, fb, clip);
	src = buf;

	for (y = clip->y1; y < clip->y2; y++) {
		for (x = clip->x1; x < clip->x2; x++) {
		    if (*src++ > 128)
				dst[(y / 8) * fb->width + x] |= (1 << (y % 8));
			else
			    dst[(y / 8) * fb->width + x] &= ~(1 << (y % 8));
		}
	}

	kfree(buf);
}

static int st7576_buf_copy(void *dst, struct drm_framebuffer *fb,
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

	st7576_xrgb8888_to_monochrome(dst, src, fb, clip);

	if (import_attach)
		ret = dma_buf_end_cpu_access(import_attach->dmabuf,
					     DMA_FROM_DEVICE);

	return ret;
}

static void st7576_fb_dirty(struct drm_framebuffer *fb, struct drm_rect *rect)
{
   	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(fb->dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
    struct spi_device *spi = dbi->spi;
    int ret = 0;
    int i, j;
    void *tr;
    u8 *buf8;

   	DRM_DEBUG_KMS("Flushing [FB:%d] " DRM_RECT_FMT "\n", fb->base.id, DRM_RECT_ARG(rect));

	ret = st7576_buf_copy(dbidev->tx_buf, fb, rect);
	if (ret)
		goto err_msg;

	buf8 = (u8 *)dbidev->tx_buf;

   	for (i = 0; i < (fb->height / 8); i++) {
        gpiod_set_value_cansleep(dbi->dc, 0);
        mipi_dbi_command(dbi, 0xb0 + i);
        mipi_dbi_command(dbi, 0x00);
        mipi_dbi_command(dbi, 0x10);

        gpiod_set_value_cansleep(dbi->dc, 1);
        for (j = 0; j < fb->width; j++) {
            tr = buf8 + (i * fb->width + j);

            mipi_dbi_spi_transfer(spi, spi->max_speed_hz, 8, tr, fb->width);
        }
    }

err_msg:
	if (ret)
		dev_err_once(fb->dev->dev, "Failed to update display %d\n", ret);
}

static void st7576_pipe_update(struct drm_simple_display_pipe *pipe,
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
		st7576_fb_dirty(state->fb, &rect);

	drm_dev_exit(idx);
}

static const u32 st7576_formats[] = {
    DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs st7576_pipe_funcs = {
    .enable = st7576_pipe_enable,
    .disable = st7576_pipe_disable,
    .update = st7576_pipe_update,
    .prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7576_mode = {
    DRM_SIMPLE_MODE(132, 64, 41, 21),
};

DEFINE_DRM_GEM_CMA_FOPS(st7576_fops);

static struct drm_driver st7576_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &st7576_fops,
    DRM_GEM_CMA_DRIVER_OPS_VMAP,
    .debugfs_init = mipi_dbi_debugfs_init,
    .name = "st7576",
    .desc = "Sitronix st7576",
    .date = "20250509",
    .major = 1,
    .minor = 0,
};

static const struct of_device_id st7576_of_match[] = {
    { .compatible = "sitronix,st7567" },
    { },
};
MODULE_DEVICE_TABLE(of, st7576_of_match);

static const struct spi_device_id st7576_id[] = {
    { "st7567" },
    { },
};
MODULE_DEVICE_TABLE(spi, st7576_id);

static int st7576_probe(struct spi_device *spi)
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

    dbidev = devm_drm_dev_alloc(dev, &st7576_driver,
                    struct mipi_dbi_dev, drm);
    if (IS_ERR(dbidev))
        return PTR_ERR(dbidev);

    dbi = &dbidev->dbi;
    drm = &dbidev->drm;

    bufsize = (st7576_mode.vdisplay * st7576_mode.hdisplay * sizeof(u16));

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

    ret = mipi_dbi_dev_init_with_formats(dbidev, &st7576_pipe_funcs,
        st7576_formats, ARRAY_SIZE(st7576_formats),
        &st7576_mode, rotation, bufsize);
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

static int st7576_remove(struct spi_device *spi)
{
    struct drm_device *drm = spi_get_drvdata(spi);

    printk("%s\n", __func__);

    drm_dev_unplug(drm);
    drm_atomic_helper_shutdown(drm);

    return 0;
}

static void st7576_shutdown(struct spi_device *spi)
{
    printk("%s\n", __func__);
    drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7576_spi_driver = {
    .driver = {
        .name = "st7576",
        .of_match_table = st7576_of_match,
    },
    .id_table = st7576_id,
    .probe = st7576_probe,
    .remove = st7576_remove,
    .shutdown = st7576_shutdown,
};
module_spi_driver(st7576_spi_driver);

MODULE_DESCRIPTION("Sitronix st7576 DRM driver");
MODULE_AUTHOR("Wooden Chair <hua.zheng@embeddedboys.com>");
MODULE_LICENSE("GPL");
