/*
 * sound\soc\sun4i\hdmiaudio\sun4i-sndhdmi.c
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * chenpailin <chenpailin@allwinnertech.com>
 *
 * some simple description for this code
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/mutex.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <plat/sys_config.h>
#include <linux/io.h>

#include "sun4i-hdmiaudio.h"
#include "sun4i-hdmipcm.h"

#include "sndhdmi.h"

static struct clk *xtal;

static int clk_users;
static DEFINE_MUTEX(clk_lock);

#ifdef ENFORCE_RATES
static struct snd_pcm_hw_constraint_list hw_constraints_rates = {
	.count	= ARRAY_SIZE(rates),
	.list	= rates,
	.mask	= 0,
};
#endif

static int sun4i_sndhdmi_startup(struct snd_pcm_substream *substream)
{
	int ret = 0;
	#ifdef ENFORCE_RATES
		struct snd_pcm_runtime *runtime = substream->runtime;;
	#endif
	mutex_lock(&clk_lock);
	mutex_unlock(&clk_lock);
	if (!ret) {
	#ifdef ENFORCE_RATES
		ret = snd_pcm_hw_constraint_list(runtime, 0,
						 SNDRV_PCM_HW_PARAM_RATE,
						 &hw_constraints_rates);
		if (ret < 0)

	#endif
	}
	return ret;
}

static void sun4i_sndhdmi_shutdown(struct snd_pcm_substream *substream)
{
	mutex_lock(&clk_lock);
	clk_users -= 1;
	if (clk_users == 0) {
		clk_put(xtal);
		xtal = NULL;
	}
	mutex_unlock(&clk_lock);
}

typedef struct __MCLK_SET_INF
{
    __u32       samp_rate;      // sample rate
    __u16       mult_fs;        // multiply of smaple rate

    __u8        clk_div;        // mpll division
    __u8        mpll;           // select mpll, 0 - 24.576 Mhz, 1 - 22.5792 Mhz

} __mclk_set_inf;

typedef struct __BCLK_SET_INF
{
    __u8        bitpersamp;     // bits per sample
    __u8        clk_div;        // clock division
    __u16       mult_fs;        // multiplay of sample rate

} __bclk_set_inf;

//bclk divider table
static __bclk_set_inf BCLK_INF[] =
{
    // 16bits per sample
    {16,  4, 128}, {16,  6, 192}, {16,  8, 256},
    {16, 12, 384}, {16, 16, 512},

    //24 bits per sample
    {24,  4, 192}, {24,  8, 384}, {24, 16, 768},

    //32 bits per sample
    {32,  2, 128}, {32,  4, 256}, {32,  6, 384},
    {32,  8, 512}, {32, 12, 768},

    //end flag
    {0xff, 0, 0},
};

//mclk divider table
static __mclk_set_inf  MCLK_INF[] =
{
    // 8k bitrate
    {  8000, 128, 24, 0}, {  8000, 192, 16, 0}, {  8000, 256, 12, 0},
    {  8000, 384,  8, 0}, {  8000, 512,  6, 0}, {  8000, 768,  4, 0},

    // 16k bitrate
    { 16000, 128, 12, 0}, { 16000, 192,  8, 0}, { 16000, 256,  6, 0},
    { 16000, 384,  4, 0}, { 16000, 768,  2, 0},

    // 32k bitrate
    { 32000, 128,  6, 0}, { 32000, 192,  4, 0}, { 32000, 384,  2, 0},
    { 32000, 768,  1, 0},

    // 64k bitrate
    { 64000, 192,  2, 0}, { 64000, 384,  1, 0},

    //128k bitrate
    {128000, 192,  1, 0},

    // 12k bitrate
    { 12000, 128, 16, 0}, { 12000, 256, 8, 0}, { 12000, 512, 4, 0},

    // 24k bitrate
    { 24000, 128,  8, 0}, { 24000, 256, 4, 0}, { 24000, 512, 2, 0},

    // 48K bitrate
    { 48000, 128,  4, 0}, { 48000, 256,  2, 0}, { 48000, 512, 1, 0},

    // 96k bitrate
    { 96000, 128 , 2, 0}, { 96000, 256,  1, 0},

    //192k bitrate
    {192000, 128,  1, 0},

    //11.025k bitrate
    { 11025, 128, 16, 1}, { 11205, 256,  8, 1}, { 11205, 512,  4, 1},

    //22.05k bitrate
    { 22050, 128,  8, 1}, { 22050, 256,  4, 1},
    { 22050, 512,  2, 1},

    //44.1k bitrate
    { 44100, 128,  4, 1}, { 44100, 256,  2, 1}, { 44100, 512,  1, 1},

    //88.2k bitrate
    { 88200, 128,  2, 1}, { 88200, 256,  1, 1},

    //176.4k bitrate
    {176400, 128, 1, 1},

    //end flag 0xffffffff
    {0xffffffff, 0, 0, 0},
};

static s32 get_clock_divder(u32 sample_rate, u32 sample_width, u32 * mclk_div,
                            u32* mpll, u32* bclk_div, u32* mult_fs)
{
	u32 i, j, ret = -EINVAL;

	for(i=0; i< 100; i++) {
		 if((MCLK_INF[i].samp_rate == sample_rate) &&
		 	((MCLK_INF[i].mult_fs == 256) || (MCLK_INF[i].mult_fs == 128))) {
			  for(j=0; j<ARRAY_SIZE(BCLK_INF); j++) {
					if((BCLK_INF[j].bitpersamp == sample_width) &&
						(BCLK_INF[j].mult_fs == MCLK_INF[i].mult_fs)) {
						 //set mclk and bclk division
						 *mclk_div = MCLK_INF[i].clk_div;
						 *mpll = MCLK_INF[i].mpll;
						 *bclk_div = BCLK_INF[j].clk_div;
						 *mult_fs = MCLK_INF[i].mult_fs;
						 ret = 0;
						 break;
					}
			  }
		 }else if(MCLK_INF[i].samp_rate == 0xffffffff)
		 	break;
	}

	return ret;
}

static int sun4i_sndhdmi_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;
	unsigned long rate = params_rate(params);
	u32 mclk_div=0, mpll=0, bclk_div=0, mult_fs=0;

	get_clock_divder(rate, 32, &mclk_div, &mpll, &bclk_div, &mult_fs);

	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0 , mpll, 0);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, 0 , mpll, 0);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SUN4I_DIV_MCLK, mclk_div);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, SUN4I_DIV_BCLK, bclk_div);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(codec_dai, 0, mult_fs);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops sun4i_sndhdmi_ops = {
	.startup 	= sun4i_sndhdmi_startup,
	.shutdown 	= sun4i_sndhdmi_shutdown,
	.hw_params 	= sun4i_sndhdmi_hw_params,
};

static struct snd_soc_dai_link sun4i_sndhdmi_dai_link = {
	.name 			= "HDMIAUDIO",
	.stream_name 	= "SUN4I-HDMIAUDIO",
	.cpu_dai_name 	= "sun4i-hdmiaudio.0",
	.codec_dai_name = "sndhdmi",
	.platform_name 	= "sun4i-hdmiaudio-pcm-audio.0",
	.codec_name 	= "sun4i-hdmiaudio-codec.0",
	.ops 			= &sun4i_sndhdmi_ops,
};

static struct snd_soc_card snd_soc_sun4i_sndhdmi = {
	.name 		= "sun4i-sndhdmi",
	.dai_link 	= &sun4i_sndhdmi_dai_link,
	.num_links 	= 1,
};

static struct platform_device *sun4i_sndhdmi_device;

static int __init sun4i_sndhdmi_init(void)
{
	int ret;

	sun4i_sndhdmi_device = platform_device_alloc("soc-audio", 0);

	if(!sun4i_sndhdmi_device)
		return -ENOMEM;

	platform_set_drvdata(sun4i_sndhdmi_device, &snd_soc_sun4i_sndhdmi);

	ret = platform_device_add(sun4i_sndhdmi_device);

	if (ret) {
		platform_device_put(sun4i_sndhdmi_device);
	}

	return ret;
}

static void __exit sun4i_sndhdmi_exit(void)
{
	platform_device_unregister(sun4i_sndhdmi_device);
}

module_init(sun4i_sndhdmi_init);
module_exit(sun4i_sndhdmi_exit);

MODULE_AUTHOR("ALL WINNER");
MODULE_DESCRIPTION("SUN4I_SNDHDMI ALSA SoC audio driver");
MODULE_LICENSE("GPL");
