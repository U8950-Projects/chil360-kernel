/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>
#include <asm/mach/mmc.h>
#include <mach/gpiomux.h>
#include <mach/board.h>
#include "devices.h"
#include "pm.h"
#include "board-msm7627a.h"
#include <linux/hardware_self_adapt.h>

#ifdef CONFIG_HUAWEI_WIFI_SDCC
#include <linux/wifi_tiwlan.h>
#include <linux/skbuff.h>
#endif

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

#define MAX_SDCC_CONTROLLER 4
static unsigned long vreg_sts, gpio_sts;

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

/**
 * Due to insufficient drive strengths for SDC GPIO lines some old versioned
 * SD/MMC cards may cause data CRC errors. Hence, set optimal values
 * for SDC slots based on timing closure and marginality. SDC1 slot
 * require higher value since it should handle bad signal quality due
 * to size of T-flash adapters.
 */
/*
 * We have the external pull up on data and cmd lines.
 * Qualcomm requests to disable the internal pull up when have external pull up.
 * Change pull up to no pull.
 */
#ifdef CONFIG_HUAWEI_KERNEL
static struct msm_gpio sdc1_cfg_data[] = {
    /* decrease sdcard drive current */
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc1_clk"},
};
#else
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_14MA),
								"sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_14MA),
								"sdc1_clk"},
};
#endif

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
								"sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
								"sdc2_dat_0"},
};

/*
 * We have the external pull up on data and cmd lines.
 * Qualcomm requests to disable the internal pull up when have external pull up.
 * Change pull up to no pull.
 */
#ifdef CONFIG_HUAWEI_KERNEL
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_10MA),
								"sdc3_dat_4"},
#endif
};
#else
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_0"},
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_7"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_6"},
	{GPIO_CFG(21, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_5"},
	{GPIO_CFG(108, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc3_dat_4"},
#endif
};
#endif

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_3"},
	{GPIO_CFG(20, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_2"},
	{GPIO_CFG(21, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_10MA),
								"sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
								"sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
	},
};

static int gpio_sdc1_hw_det = 85;
static void gpio_sdc1_config(void)
{
	if (machine_is_msm7627a_qrd1() || machine_is_msm7627a_evb()
					|| machine_is_msm8625_evb()
					|| machine_is_msm7627a_qrd3()
					|| machine_is_msm8625_qrd7())
		gpio_sdc1_hw_det = 42;
}

static struct regulator *sdcc_vreg_data[MAX_SDCC_CONTROLLER];
static int msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			pr_err("%s: Failed to turn on GPIOs for slot %d\n",
					__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			rc = msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return rc;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
	return rc;
}

static int msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];

	if (test_bit(dev_id, &vreg_sts) == enable)
		return 0;

	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);

	if (enable) {
		set_bit(dev_id, &vreg_sts);

		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
						__func__, rc);
	} else {
		clear_bit(dev_id, &vreg_sts);

		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
						__func__, rc);
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	rc = msm_sdcc_setup_gpio(pdev->id, !!vdd);
	if (rc)
		goto out;

	rc = msm_sdcc_setup_vreg(pdev->id, !!vdd);
out:
	return rc;
}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static unsigned int msm7627a_sdcc_slot_status(struct device *dev)
{
	int status;

	status = gpio_tlmm_config(GPIO_CFG(gpio_sdc1_hw_det, 2, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
				GPIO_CFG_ENABLE);
	if (status)
		pr_err("%s:Failed to configure tlmm for GPIO %d\n", __func__,
				gpio_sdc1_hw_det);

	status = gpio_request(gpio_sdc1_hw_det, "SD_HW_Detect");
	if (status) {
		pr_err("%s:Failed to request GPIO %d\n", __func__,
				gpio_sdc1_hw_det);
	} else {
		status = gpio_direction_input(gpio_sdc1_hw_det);
		if (!status) {
#ifdef CONFIG_HUAWEI_KERNEL
            if (machine_is_msm7x27a_U8185() ||
                    machine_is_msm8x25_U8951D() ||
                    machine_is_msm8x25_U8951() ||
                    machine_is_msm8x25_C8813()
                    )
            {
                //u8185 is different from other products.
                status = gpio_get_value(gpio_sdc1_hw_det);
            }
            else
            {
                status = !gpio_get_value(gpio_sdc1_hw_det);
            }
#else
			if (machine_is_msm7627a_qrd1() ||
					machine_is_msm7627a_evb() ||
					machine_is_msm8625_evb()  ||
					machine_is_msm7627a_qrd3() ||
					machine_is_msm8625_qrd7())
				status = !gpio_get_value(gpio_sdc1_hw_det);
			else
				status = gpio_get_value(gpio_sdc1_hw_det);
#endif
		}
		gpio_free(gpio_sdc1_hw_det);
	}
	return status;
}

static struct mmc_platform_data sdc1_plat_data = {
	.ocr_mask       = MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin   = 144000,
	.msmsdcc_fmid   = 24576000,
	.msmsdcc_fmax   = 49152000,
	.status      = msm7627a_sdcc_slot_status,
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
/*disable pm runtime of sd card*/
#ifdef CONFIG_HUAWEI_KERNEL
    .disable_runtime_pm = true
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data sdc2_plat_data_ATH = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask       = MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
//#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
/* ATH WIFI wake up Msm interrupt GPIO 66 */
	.sdiowakeup_irq = MSM_GPIO_TO_INT(66), 
//#endif
	.msmsdcc_fmin   = 144000,
	.msmsdcc_fmid   = 24576000,
	.msmsdcc_fmax   = 49152000,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
static struct mmc_platform_data sdc2_plat_data = {
	/*
	 * SDC2 supports only 1.8V, claim for 2.85V range is just
	 * for allowing buggy cards who advertise 2.8V even though
	 * they can operate at 1.8V supply.
	 */
	.ocr_mask       = MMC_VDD_28_29 | MMC_VDD_165_195,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	/*.sdiowakeup_irq = MSM_GPIO_TO_INT(66),*/
#endif
	.msmsdcc_fmin   = 144000,
	.msmsdcc_fmid   = 24576000,
	.msmsdcc_fmax   = 49152000,
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data sdc3_plat_data = {
	.ocr_mask       = MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin   = 144000,
	.msmsdcc_fmid   = 24576000,
	.msmsdcc_fmax   = 49152000,
	.nonremovable   = 1,
	/*prevent emmc from stepping into pm runtime sleep*/
#ifdef CONFIG_HUAWEI_KERNEL
	.disable_runtime_pm = true,
#endif
};
#endif

#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
static struct mmc_platform_data sdc4_plat_data = {
	.ocr_mask       = MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin   = 144000,
	.msmsdcc_fmid   = 24576000,
	.msmsdcc_fmax   = 49152000,
};
#endif

#ifdef CONFIG_HUAWEI_WIFI_SDCC
#define TAG_BCM			"BCM_4330"

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_HDRSIZE		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SKB_BUF_NUM	17

/* for wifi awake */
#define WLAN_WAKES_MSM        		48	
/* for wifi power supply */
#define WLAN_REG 					6

#define WLAN_GPIO_FUNC_0         	0
#define WLAN_GPIO_FUNC_1         	1
#define WLAN_STAT_ON             	1
#define WLAN_STAT_OFF            	0
	
static unsigned wlan_wakes_msm[] = {
	GPIO_CFG( WLAN_WAKES_MSM, WLAN_GPIO_FUNC_0 , GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA ) 
};

static unsigned wifi_config_init[] = {
	GPIO_CFG( WLAN_REG, WLAN_GPIO_FUNC_0 , GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL , GPIO_CFG_2MA ) 
};

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];
static void *wlan_static_scan_buf0;
static void *wlan_static_scan_buf1;

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

/*wlan static memory alloc*/
static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;
	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;
	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

/*wlan power control*/
static int bcm_wifi_set_power(int enable)
{
	int ret = 0;

   	if (enable)
	{
			/* turn on wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_ON);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn on wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(150);
            printk(KERN_ERR "%s: wifi power successed to pull up\n" , __func__ );
		
	}
    else { 
        	/* turn off wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_OFF);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn off wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(1);
            printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__ );
	}

	return ret;
}

int __init bcm_wifi_init_gpio_mem(void)
{
	int i ;
	int rc = 0;
	int j ;

	/* config gpio WLAN_WAKES_MSM */
	rc = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if( rc ) 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wlan_wakes_msm) failed rc = %d\n", __func__ , TAG_BCM , rc);
	else 
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wlan_wakes_msm) successfully\n", __func__ , TAG_BCM);
	/* request gpio WLAN_WAKES_MSM */
	rc = gpio_request( WLAN_WAKES_MSM , "WLAN_WAKES_MSM" );
	if( rc ) 
		printk(KERN_ERR "%s: %s Failed to gpio_request(WLAN_WAKES_MSM) rc = %d\n" , __func__ , TAG_BCM , rc );	
	else 
		printk(KERN_ERR "%s: %s Success to gpio_request(WLAN_WAKES_MSM)\n" , __func__ , TAG_BCM );	

	/* config gpio WLAN_REG */
	rc = gpio_tlmm_config(wifi_config_init[0], GPIO_CFG_ENABLE);
	if( rc )
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wifi_config_init) failed rc = %d\n", __func__ , TAG_BCM , rc);
	else
		printk(KERN_ERR "%s: %s gpio_tlmm_config(wifi_config_init) successfully\n", __func__ , TAG_BCM);
	/* request gpio WLAN_REG */
	rc = gpio_request( WLAN_REG , "WLAN_REG" );
	if( rc )
		printk(KERN_ERR "%s: %s Failed to gpio_request(WLAN_REG) rc = %d\n" , __func__ , TAG_BCM , rc);
	else
		printk(KERN_ERR "%s: %s Success to gpio_request(WLAN_REG)\n" , __func__ , TAG_BCM );
	
    mdelay(5);

    /* turn off wifi_vreg */
    rc = gpio_direction_output(WLAN_REG, 0);
    if (rc < 0) {
		printk(KERN_ERR "%s: %s turn off wlan_reg failed (%d)\n" , __func__, TAG_BCM,  rc);
		return -EIO;
    }
    else {
		printk(KERN_ERR "%s: %s turn off wlan_reg successfully (%d)\n" , __func__, TAG_BCM,  rc);
    }

    mdelay(5);
       
	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wlan_mem_array[i].mem_ptr =
			kmalloc(wlan_mem_array[i].size, GFP_KERNEL);

		if (!wlan_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	wlan_static_scan_buf0 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf0)
		goto err_mem_alloc;
	wlan_static_scan_buf1 = kmalloc(65536, GFP_KERNEL);
	if (!wlan_static_scan_buf1)
		goto err_mem_alloc;

	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for wifi\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
	pr_err("Failed to skb_alloc for wifi\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

/* Customized Locale table : OPTIONAL feature */
#define COUNTRY_BUF_SZ	4
struct cntry_locales_custom {
	char iso_abbrev[COUNTRY_BUF_SZ];
	char custom_locale[COUNTRY_BUF_SZ];
	int  custom_locale_rev;
};

static struct cntry_locales_custom bcm_wifi_translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XV", 17},	/* Universal if Country code is unknown or empty */
	{"IR", "XV", 17},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XV", 17},	/* Universal if Country code is SUDAN */
	{"SY", "XV", 17},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XV", 17},	/* Universal if Country code is GREENLAND */
	{"PS", "XV", 17},	/* Universal if Country code is PALESTINE */
	{"TL", "XV", 17},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XV", 17},	/* Universal if Country code is MARSHALL ISLANDS */
	{"PK", "XV", 17},	/* Universal if Country code is PAKISTAN */
	{"CK", "XV", 17},	/* Universal if Country code is Cook Island (13.4.27)*/
	{"CU", "XV", 17},	/* Universal if Country code is Cuba (13.4.27)*/
	{"FK", "XV", 17},	/* Universal if Country code is Falkland Island (13.4.27)*/
	{"FO", "XV", 17},	/* Universal if Country code is Faroe Island (13.4.27)*/
	{"GI", "XV", 17},	/* Universal if Country code is Gibraltar (13.4.27)*/
	{"IM", "XV", 17},	/* Universal if Country code is Isle of Man (13.4.27)*/
	{"CI", "XV", 17},	/* Universal if Country code is Ivory Coast (13.4.27)*/
	{"JE", "XV", 17},	/* Universal if Country code is Jersey (13.4.27)*/
	{"KP", "XV", 17},	/* Universal if Country code is North Korea (13.4.27)*/
	{"FM", "XV", 17},	/* Universal if Country code is Micronesia (13.4.27)*/
	{"MM", "XV", 17},	/* Universal if Country code is Myanmar (13.4.27)*/
	{"NU", "XV", 17},	/* Universal if Country code is Niue (13.4.27)*/
	{"NF", "XV", 17},	/* Universal if Country code is Norfolk Island (13.4.27)*/
	{"PN", "XV", 17},	/* Universal if Country code is Pitcairn Islands (13.4.27)*/
	{"PM", "XV", 17},	/* Universal if Country code is Saint Pierre and Miquelon (13.4.27)*/
	{"SS", "XV", 17},	/* Universal if Country code is South_Sudan (13.4.27)*/
	{"AL", "AL", 2},
	{"DZ", "DZ", 1},
	{"AS", "AS", 12},	/* changed 2 -> 12*/
	{"AI", "AI", 1},
	{"AG", "AG", 2},
	{"AR", "AR", 21},
	{"AW", "AW", 2},
	{"AU", "AU", 6},
	{"AT", "AT", 4},
	{"AZ", "AZ", 2},
	{"BS", "BS", 2},
	{"BH", "BH", 4},	/* changed 24 -> 4*/
	{"BD", "BD", 2},
	{"BY", "BY", 3},
	{"BE", "BE", 4},
	{"BM", "BM", 12},
	{"BA", "BA", 2},
	{"BR", "BR", 4},
	{"VG", "VG", 2},
	{"BN", "BN", 4},
	{"BG", "BG", 4},
	{"KH", "KH", 2},
	{"CA", "CA", 31},
	{"KY", "KY", 3},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CR", "CR", 17},
	{"HR", "HR", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ET", "ET", 2},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GF", "GF", 2},
	{"DE", "DE", 7},
	{"GR", "GR", 4},
	{"GD", "GD", 2},
	{"GP", "GP", 2},
	{"GU", "GU", 12},
	{"HK", "HK", 2},
	{"HU", "HU", 4},
	{"IS", "IS", 4},
	{"IN", "IN", 3},
	{"ID", "KR", 25},	/* ID/1 -> KR/24 */
	{"IE", "IE", 5},
	{"IL", "BO", 0},	/* IL/7 -> BO/0 */
	{"IT", "IT", 4},
	{"JP", "JP", 58},
	{"JO", "JO", 3},
	{"KW", "KW", 5},
	{"LA", "LA", 2},
	{"LV", "LV", 4},
	{"LB", "LB", 5},
	{"LS", "LS", 2},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"MO", "MO", 2},
	{"MK", "MK", 2},
	{"MW", "MW", 1},
	{"MY", "MY", 3},
	{"MV", "MV", 3},
	{"MT", "MT", 4},
	{"MQ", "MQ", 2},
	{"MR", "MR", 2},
	{"MU", "MU", 2},
	{"YT", "YT", 2},
	{"MX", "MX", 20},
	{"MD", "MD", 2},
	{"MC", "MC", 1},
	{"ME", "ME", 2},
	{"MA", "MA", 2},
	{"NP", "NP", 3},
	{"NL", "NL", 4},
	{"AN", "AN", 2},
	{"NZ", "NZ", 4},
	{"NO", "NO", 4},
	{"OM", "OM", 4},
	{"PA", "PA", 17},
	{"PG", "PG", 2},
	{"PY", "PY", 2},
	{"PE", "PE", 20},
	{"PH", "PH", 5},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PR", "PR", 20},
	{"RE", "RE", 2},
	{"RO", "RO", 4},
	{"SN", "SN", 2},
	{"RS", "RS", 2},
	{"SG", "SG", 4},
	{"SK", "SK", 4},
	{"SI", "SI", 4},
	{"ES", "ES", 4},
	{"LK", "LK", 1},
	{"SE", "SE", 4},
	{"CH", "CH", 4},
	{"TW", "TW", 1},
	{"TH", "TH", 5},
	{"TT", "TT", 3},
	{"TR", "TR", 7},
	{"AE", "AE", 6},
	{"UG", "UG", 2},
	{"GB", "GB", 6},
	{"UY", "UY", 1},
	{"VI", "VI", 13},
	{"VA", "VA", 2},
	{"VE", "VE", 3},
	{"VN", "VN", 4},
	{"MA", "MA", 1},
	{"ZM", "ZM", 2},
	{"EC", "EC", 21},
	{"SV", "SV", 19},
	{"KR", "KR", 57},
	{"RU", "RU", 13},
	{"UA", "UA", 8},
	{"GT", "GT", 1},
	{"MN", "MN", 1},
	{"NI", "NI", 2},
	{"US", "Q2", 57},
};

static void *bcm_wifi_get_country_code(char *ccode)
{
	int size, i;
	static struct cntry_locales_custom country_code;

	size = ARRAY_SIZE(bcm_wifi_translate_custom_table);

	if ((size == 0) || (ccode == NULL))
		return NULL;

	for (i = 0; i < size; i++) {
		if (!strcmp(
			ccode, bcm_wifi_translate_custom_table[i].iso_abbrev))
			return &bcm_wifi_translate_custom_table[i];
	}

	memset(&country_code, 0, sizeof(struct cntry_locales_custom));
	strlcpy(country_code.custom_locale, ccode, COUNTRY_BUF_SZ);

	return &country_code;
}

static struct wifi_platform_data bcm_wifi_control = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power	=bcm_wifi_set_power,
	.get_country_code	= bcm_wifi_get_country_code,
};

static struct platform_device bcm_wifi_device = {
        .name           = "bcm4330_wlan",	/*bcm4330 wlan device*/
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
        .dev            = {
                .platform_data = &bcm_wifi_control,
        },
};
#endif

extern int sdcc_wifi_slot;


static int __init mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}

void __init msm7627a_init_mmc(void)
{
	/* eMMC slot */
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT

	/* There is no eMMC on SDC3 for QRD3 based devices */
	if (!(machine_is_msm7627a_qrd3() || machine_is_msm8625_qrd7())) {
/* "S3" is always on for emmc,so don't configure the "emmc" in the linux */
#ifdef CONFIG_HUAWEI_KERNEL
    if (mmc_regulator_init(3, "smps3", 1800000))
        return;       
#else
	if (mmc_regulator_init(3, "emmc", 3000000))
		return;
#endif	
		msm_add_sdcc(3, &sdc3_plat_data);
	}
#endif
	/* Micro-SD slot */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	gpio_sdc1_config();
	if (mmc_regulator_init(1, "mmc", 2850000))
		return;
	/* 8x25 EVT do not use hw detector */
	if (!(machine_is_msm8625_evt()))
		sdc1_plat_data.status_irq = MSM_GPIO_TO_INT(gpio_sdc1_hw_det);
	if (machine_is_msm8625_evt())
		sdc1_plat_data.status = NULL;

	msm_add_sdcc(1, &sdc1_plat_data);
#endif
	/* SDIO WLAN slot */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	/* WLAN use S3 as power supply but not mmc */
#ifdef CONFIG_HUAWEI_KERNEL
	if (mmc_regulator_init(2, "smps3", 1800000))
		return;
#else
	if (mmc_regulator_init(2, "mmc", 2850000))
		return;
#endif	
	sdcc_wifi_slot = 2;
    if (WIFI_BROADCOM == get_hw_wifi_device_type())
        msm_add_sdcc(2, &sdc2_plat_data);
    else
        msm_add_sdcc(2, &sdc2_plat_data_ATH);
      
	if (WIFI_BROADCOM == get_hw_wifi_device_type()){
		#ifdef CONFIG_HUAWEI_WIFI_SDCC		
		bcm_wifi_init_gpio_mem();
		platform_device_register(&bcm_wifi_device);
		#endif
	}
#endif
	/* Not Used */
#if (defined(CONFIG_MMC_MSM_SDC4_SUPPORT)\
		&& !defined(CONFIG_MMC_MSM_SDC3_8_BIT_SUPPORT))
	/* There is no SDC4 for QRD3/7 based devices */
	if (!(machine_is_msm7627a_qrd3() || machine_is_msm8625_qrd7())) {
		if (mmc_regulator_init(4, "smps3", 1800000))
			return;
		msm_add_sdcc(4, &sdc4_plat_data);
	}
#endif
}
#endif
