/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 */

#ifndef _SDE_HW_RC_H
#define _SDE_HW_RC_H

#include "sde_hw_mdss.h"
#include "sde_crtc.h"


/**
 * sde_hw_rc_init -  Initialize RC internal state object and ops
 * @hw_dspp: DSPP instance.
 * Return: 0 on success, non-zero otherwise.
 */
int sde_hw_rc_init(struct sde_hw_dspp *hw_dspp);

/**
 * sde_hw_rc_check_mask -  Validate RC mask configuration
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success, non-zero otherwise.
 */
int sde_hw_rc_check_mask(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_setup_mask -  Setup RC mask configuration
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success, non-zero otherwise.
 */
int sde_hw_rc_setup_mask(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_check_pu_roi -  Validate RC partial update region of interest
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success.
 *         > 0 on early return.
 *         < 0 on error.
 */
int sde_hw_rc_check_pu_roi(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_setup_pu_roi -  Setup RC partial update region of interest
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success.
 *         > 0 on early return.
 *         < 0 on error.
 */
int sde_hw_rc_setup_pu_roi(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_setup_data_ahb - Program mask data with AHB
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success, non-zero otherwise.
 */
int sde_hw_rc_setup_data_ahb(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_setup_data_dma - Program mask data with DMA
 * @hw_dspp: DSPP instance.
 * @cfg: Pointer to configuration blob.
 * Return: 0 on success, non-zero otherwise.
 */
int sde_hw_rc_setup_data_dma(struct sde_hw_dspp *hw_dspp, void *cfg);

/**
 * sde_hw_rc_data_programmed - Check if mask data is programmed
 * @hw_dspp: DSPP instance.
 * Return: true if data is programmed, false otherwise.
 */
bool sde_hw_rc_data_programmed(struct sde_hw_dspp *hw_dspp);

/**
 * sde_hw_rc_dma_pending - Check if mask data programming over DMA is pending
 * @sde_crtc: Pointer to sde crtc
 * Return: true if DMA programming is pending, false otherwise.
 */
bool sde_hw_rc_dma_pending(struct sde_crtc *sde_crtc);

/**
 * sde_hw_rc_dma_done - Indicate that DMA programming of mask data is done
 * @sde_crtc: Pointer to sde crtc
 * Return: none.
 */
void sde_hw_rc_dma_done(struct sde_crtc *sde_crtc);

#endif
