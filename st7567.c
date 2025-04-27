// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for display panels connected to a Sitronix ST7567
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
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_mipi_dbi.h>

static void st7567_pipe_update(struct drm_simple_display_pipe *pipe,
    struct drm_plane_state *old_state)
{

}

static const u32 st7567_formats[] = {
    DRM_FORMAT_XRGB8888,
};

static const struct drm_simple_display_pipe_funcs st7567_pipe_funcs = {
    .enable = NULL,
    .disable = NULL,
    .update = st7567_pipe_update,
    .prepare_fb = drm_gem_fb_simple_display_pipe_prepare_fb,
};

static const struct drm_display_mode st7567_mode = {
    DRM_SIMPLE_MODE(128, 64, 41, 21),
};

DEFINE_DRM_GEM_CMA_FOPS(st7567_fops);

static struct drm_driver st7567_driver = {
    .driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
    .fops = &st7567_fops,
    DRM_GEM_CMA_DRIVER_OPS_VMAP,
    .debugfs_init = mipi_dbi_debugfs_init,
    .name = "st7567",
    .desc = "Sitronix ST7567",
    .date = "20250427",
    .major = 1,
    .minor = 0,
};

static const struct of_device_id st7567_of_match[] = {
    { .compatible = "sitronix,st7567" },
    { },
};
MODULE_DEVICE_TABLE(of, st7567_of_match);

static const struct spi_device_id st7567_id[] = {
    { "st7567" },
    { },
};
MODULE_DEVICE_TABLE(spi, st7567_id);

static int st7567_probe(struct spi_device *spi)
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

    dbidev = devm_drm_dev_alloc(dev, &st7567_driver,
                    struct mipi_dbi_dev, drm);
    if (IS_ERR(dbidev))
        return PTR_ERR(dbidev);

    dbi = &dbidev->dbi;
    drm = &dbidev->drm;

    bufsize = (st7567_mode.vdisplay * st7567_mode.hdisplay / 8);

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

    ret = mipi_dbi_dev_init_with_formats(dbidev, &st7567_pipe_funcs,
        st7567_formats, ARRAY_SIZE(st7567_formats),
        &st7567_mode, rotation, bufsize);
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

static int st7567_remove(struct spi_device *spi)
{
    struct drm_device *drm = spi_get_drvdata(spi);

    printk("%s\n", __func__);

    drm_dev_unplug(drm);
    drm_atomic_helper_shutdown(drm);

    return 0;
}

static void st7567_shutdown(struct spi_device *spi)
{
    printk("%s\n", __func__);
    drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7567_spi_driver = {
    .driver = {
        .name = "st7567",
        .of_match_table = st7567_of_match,
    },
    .id_table = st7567_id,
    .probe = st7567_probe,
    .remove = st7567_remove,
    .shutdown = st7567_shutdown,
};
module_spi_driver(st7567_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7567 DRM driver");
MODULE_AUTHOR("Zheng Hua <hua.zheng@embeddedboys.com>");
MODULE_LICENSE("GPL");
