// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include <linux/module.h>
#include <cam_sensor_cmn_header.h>
#include "cam_actuator_core.h"
#include "cam_sensor_util.h"
#include "cam_trace.h"
#include "cam_common_util.h"
#include "cam_packet_util.h"

#include "dw9784_fw_data.h"

#define EOK						0
#define ERROR_SECOND_ID 				1
#define ERROR_WHOAMI					2
#define ERROR_FW_VALID					3
#define ERROR_FW_VERIFY 				4
#define ERROR_FW_CHECKSUM				5
#define ERROR_FW_DOWN_FMC				6
#define ERROR_FW_DWON_FAIL				7
#define ERROR_ALL_CHECKUM				8
#define ERROR_WHO_AMI					9
#define ERROR_WAIT_CHECK_REG				10
#define ERROR_GYRO_OFS_CAL				11
#define ERROR_CAL_STORE					12


#define GYRO_TDK_ICM20690				0
#define GYRO_ST_LSM6DSM					2
#define GYRO_ST_LSM6DSOQ				4
#define GYRO_TDK_ICM42631				5
#define GYRO_BOSCH_BMI260				6
#define GYRO_TDK_ICM42692				7

#define GYRO_FRONT_LAYOUT				0
#define GYRO_BACK_LAYOUT				1

#define GYRO_DEGREE_0					0
#define GYRO_DEGREE_90					90
#define GYRO_DEGREE_180					180
#define GYRO_DEGREE_270					270

#define DW9784_CHIP_ID_ADDRESS			0x7000
#define DW9784_CHIP_ID					0x9784

#define MCS_SIZE_W						10240	//20KB
#define DATPKT_SIZE						256
#define PID_SIZE_W						256		//0.5KB

#define IF_START_ADDRESS				0x8000
#define MCS_START_ADDRESS				0x8000

//#define USE_FW_CHECK

#define LOOP_A						200
#define LOOP_B						LOOP_A-1
#define WAIT_TIME					100

/* gyro offset calibration */
#define GYRO_OFS_CAL_DONE_FAIL				0xFF
#define X_AXIS_GYRO_OFS_PASS				0x1
#define X_AXIS_GYRO_OFS_FAIL				0x1
#define Y_AXIS_GYRO_OFS_PASS				0x2
#define Y_AXIS_GYRO_OFS_FAIL				0x2
#define X_AXIS_GYRO_OFS_OVER_MAX_LIMIT			0x10
#define Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT			0x20
#define XY_AXIS_CHECK_GYRO_RAW_DATA			0x800

//#define USE_GYRO_OFS_ZERO

extern bool DW9784_calibration_enable;
extern int DW9784_calibration_status;



typedef struct
{
	unsigned short driverIc;
	unsigned short *fwContentPtr;
	unsigned short version;
}FirmwareContex;



/*Global buffer for flash download*/
FirmwareContex g_firmwareContext;
unsigned short g_downloadByForce;
unsigned short g_updateFw;

unsigned short set_gyro_type = 0;	/* gyro type on the phone set */
int gyro_arrangement = 0;
int gyro_degree = 0;

static int32_t write_reg_value(struct camera_io_master *io_master_info,uint32_t reg,uint32_t data,uint32_t delay_ms)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	struct cam_sensor_i2c_reg_array    reg_setting;
	int32_t rc=0;

	reg_setting.reg_addr = reg;
	reg_setting.reg_data = data;
	reg_setting.delay = 0;
	reg_setting.data_mask = 0;

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = 1;
	i2c_reg_setting.delay = delay_ms;
	i2c_reg_setting.reg_setting = &reg_setting;

	rc = camera_io_dev_write_continuous(io_master_info,
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_SEQ);

	return rc;
}

static int32_t write_block_16bit_value(struct camera_io_master *io_master_info,uint32_t reg,
unsigned short *data,uint32_t size)
{
#if 1
	struct cam_sensor_i2c_reg_setting  i2c_reg_setting;
	void *vaddr = NULL;
	int32_t rc=0,mem_size=0,cnt=0;

	mem_size = (sizeof(struct cam_sensor_i2c_reg_array) * size);
	vaddr = vmalloc(mem_size);
	if (!vaddr) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed in allocating i2c_array: mem_size: %u", mem_size);
		return -ENOMEM;
	}

	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *) (
		vaddr);

	for (cnt = 0; cnt < size; cnt++ ) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = reg + cnt;
		i2c_reg_setting.reg_setting[cnt].reg_data = data[cnt];
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}

	i2c_reg_setting.addr_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_WORD;
	i2c_reg_setting.size = size;
	i2c_reg_setting.delay = 0;

	rc = camera_io_dev_write_continuous(io_master_info,
		&i2c_reg_setting, CAM_SENSOR_I2C_WRITE_SEQ);

	vfree(vaddr);
#else
	uint32_t i=0,addr=0;
	int32_t rc=0;

	for(i=0;i<size;i++)
	{
		addr = reg+i;
		rc=write_reg_value(io_master_info,addr,data[i],0);
	}
#endif
	return rc;

}

static int32_t read_reg_value(struct camera_io_master *io_master_info,uint32_t reg,uint32_t* data)
{
	int32_t rc=0;

	rc = camera_io_dev_read(io_master_info,reg, data, CAMERA_SENSOR_I2C_TYPE_WORD,
		CAMERA_SENSOR_I2C_TYPE_WORD);

	return rc;
}

static int dw9784_wait_check_register(struct camera_io_master *io_master_info,unsigned short reg, unsigned short ref)
{
	uint32_t r_data;
	int i=0;

	for(i = 0; i < LOOP_A; i++) {
		read_reg_value(io_master_info,reg,&r_data); //Read status
		if(r_data == ref) {
			break;
		}
		else {
			if (i >= LOOP_B) {
				CAM_ERR(CAM_ACTUATOR,"[dw9784_wait_check_register]fail: 0x%04X", r_data);
				return ERROR_WAIT_CHECK_REG;
			}
		}
		msleep(WAIT_TIME);
	}
	return EOK;
}


static void dw9784_flash_acess(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_flash_acess] start");
	/* release all protection */
	write_reg_value(io_master_info,0xFAFA, 0x98AC,1);

	write_reg_value(io_master_info,0xF053, 0x70BD,2);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_flash_acess] finish");
}

static void dw9784_ois_reset(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_ois_reset] start ois reset");
	write_reg_value(io_master_info,0xD002, 0x0001,4); /* logic reset */
	write_reg_value(io_master_info,0xD001, 0x0001,25); /* Active mode (DSP ON) */
	write_reg_value(io_master_info,0xEBF1, 0x56FA,0); /* User protection release */
	CAM_INFO(CAM_ACTUATOR,"[dw9784_ois_reset] finish");
}

static int dw9784_whoami_chk(struct camera_io_master *io_master_info)
{
	uint32_t sec_chip_id;
	write_reg_value(io_master_info, 0xD000, 0x0001, 4); /* chip enable */
	write_reg_value(io_master_info, 0xD001, 0x0000, 1); /* dsp off mode */

	dw9784_flash_acess(io_master_info); /* All protection */

	read_reg_value(io_master_info,0xD060,&sec_chip_id);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_ois_ready_check] sec_chip_id : 0x%04x", sec_chip_id);
	if(sec_chip_id != 0x0020)
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784] second_chip_id check fail : 0x%04X", sec_chip_id);
		CAM_ERR(CAM_ACTUATOR,"[dw9784] second_enter shutdown mode");
		CAM_ERR(CAM_ACTUATOR,"[dw9784] dw9784 cannot work");
		write_reg_value(io_master_info,0xD000, 0x0000,0); /* ic */
		return ERROR_SECOND_ID;
	}

	dw9784_ois_reset(io_master_info); /* ois reset */
	return EOK;
}

static uint32_t dw9784_chip_id_chk(struct camera_io_master *io_master_info)
{
	uint32_t chip_id;
	read_reg_value(io_master_info,DW9784_CHIP_ID_ADDRESS, &chip_id);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_chip_id] chip_id : 0x%04X", chip_id);
	return chip_id;
}

static int dw9784_checksum_fw_chk(struct camera_io_master *io_master_info)
{
	/*
	Bit [0]: FW checksum error
	Bit [1]: Module cal. checksum error
	Bit [2]: Set cal. checksum error
	*/
	uint32_t reg_fw_checksum;			// 0x700C
	uint32_t reg_checksum_status;		// 0x700D

	read_reg_value(io_master_info,0x700C, &reg_fw_checksum);
	read_reg_value(io_master_info,0x700D, &reg_checksum_status);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_checksum_fw_chk] reg_checksum_status : 0x%04X", reg_checksum_status);
	CAM_INFO(CAM_ACTUATOR,"[dw9784_checksum_fw_chk] ref_fw_checksum : 0x%04X, reg_fw_checksum : 0x%04X", DW9784_FW_CHECKSUM, reg_fw_checksum);

	if( (reg_checksum_status & 0x0001) == 0)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_checksum_fw_chk] fw checksum pass");
		return EOK;
	}else
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784_checksum_fw_chk] fw checksum error");
		return ERROR_FW_CHECKSUM;
	}
}

static unsigned short dw9784_fw_ver_chk(struct camera_io_master *io_master_info)
{
	uint32_t fw_ver;
	uint32_t fw_date;

	read_reg_value(io_master_info,0x7001, &fw_ver);
	read_reg_value(io_master_info,0x7002, &fw_date);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_fw_ver_chk] fw version : 0x%04X", fw_ver);
	CAM_INFO(CAM_ACTUATOR,"[dw9784_fw_ver_chk] fw date : 0x%04X", fw_date);

	return fw_ver;
}

static void dw9784_shutdown_mode(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[all_prot_off] enter ic shutdown mode");
	write_reg_value(io_master_info,0xD000, 0x0000,1);
}

static void dw9784_code_pt_off(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_code_pt_off] start");
	/* release all protection */
	write_reg_value(io_master_info,0xFD00, 0x5252,1);
	CAM_INFO(CAM_ACTUATOR,"[dw9784_code_pt_off] finish");
}

static void dw9784_fw_eflash_erase(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_fw_eflash_erase] start fw flash erase");
	write_reg_value(io_master_info,0xde03, 0x0000,1);			// 4k Sector_0

	write_reg_value(io_master_info,0xde04, 0x0002,10);			// 4k Sector Erase

	write_reg_value(io_master_info,0xde03, 0x0008,1);			// 4k Sector_1

	write_reg_value(io_master_info,0xde04, 0x0002,10);			// 4k Sector Erase

	write_reg_value(io_master_info,0xde03, 0x0010,1);			// 4k Sector_2

	write_reg_value(io_master_info,0xde04, 0x0002,10);			// 4k Sector Erase

	write_reg_value(io_master_info,0xde03, 0x0018,1);			// 4k Sector_3

	write_reg_value(io_master_info,0xde04, 0x0002,10);			// 4k Sector Erase

	write_reg_value(io_master_info,0xde03, 0x0020,1);			// 4k Sector_4

	write_reg_value(io_master_info,0xde04, 0x0002,10);			// 4k Sector Erase

	CAM_INFO(CAM_ACTUATOR,"[dw9784_fw_eflash_erase] finish");
}

static void dw9784_pid_erase(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_pid_erase] start pid flash(IF) erase");
	write_reg_value(io_master_info,0xde03, 0x0000,1);			// page 0

	write_reg_value(io_master_info,0xde04, 0x0008,10);			// page erase

	CAM_INFO(CAM_ACTUATOR,"[dw9784_pid_erase] finish");
}


#ifdef USE_FW_CHECK
unsigned short buf_temp[10240];
#endif
static int dw9784_download_fw(struct camera_io_master *io_master_info,int module_state)
{
	unsigned short i;
	uint32_t addr;
	uint32_t FMC;
	//unsigned short buf[g_firmwareContext.size];
#ifdef USE_FW_CHECK
	uint32_t data_buf = 0;
	memset(buf_temp, 0, MCS_SIZE_W * sizeof(unsigned short));
#endif

	/* step 1: MTP Erase and DSP Disable for firmware 0x8000 write */
	write_reg_value(io_master_info,0xd001, 0x0000,0);

	/* step 2: MTP setup */
	dw9784_flash_acess(io_master_info);


	/* step 3. FMC register check */
	write_reg_value(io_master_info,0xDE01, 0x0000,1); // FMC block FW select

	read_reg_value(io_master_info,0xDE01, &FMC);
	if (FMC != 0)
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] FMC register value 1st warning : %04x", FMC);
		write_reg_value(io_master_info,0xDE01, 0x0000,1);
		FMC = 0; // initialize FMC value

		read_reg_value(io_master_info,0xDE01, &FMC);
		if (FMC != 0)
		{
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] 2nd FMC register value 2nd warning : %04x", FMC);
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] stop f/w download");
			return ERROR_FW_DOWN_FMC;
		}
	}

	/* step 4. code protection off */
	dw9784_code_pt_off(io_master_info);

	/* step 5. erase flash fw data */
	dw9784_fw_eflash_erase(io_master_info);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] start firmware download");
	/* step 6. firmware sequential write to flash */
	/* updates the module status before firmware download */
	*(g_firmwareContext.fwContentPtr + MCS_SIZE_W -1) = module_state;
	for (i = 0; i < MCS_SIZE_W; i += DATPKT_SIZE)
	{
		addr = MCS_START_ADDRESS + i;
		write_block_16bit_value(io_master_info,addr,g_firmwareContext.fwContentPtr + i,DATPKT_SIZE);
	}

#ifdef USE_FW_CHECK
	/* Check by reading fw flash directly to i2c */
	/* It is disabled by default */
	/* firmware sequential read from flash */
	for (i = 0; i <  MCS_SIZE_W; i++)
	{
		addr = MCS_START_ADDRESS + i;
		read_reg_value(io_master_info,addr, &data_buf);
		buf_temp[i] = data_buf;
	}
	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] read firmware from flash");
	/* firmware verify */
	for (i = 0; i < MCS_SIZE_W; i++)
	{
		if (g_firmwareContext.fwContentPtr[i] != buf_temp[i])
		{
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] firmware verify NG!!! ADDR:%04X -- firmware:%04x -- READ:%04x ", MCS_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i], buf_temp[i]);
			return ERROR_FW_VERIFY;
		}
	}
	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] firmware verification pass");
#endif

	/* step 6. Writes 512Byte FW(PID) data to IF flash.	(FMC register check) */
	write_reg_value(io_master_info,0xDE01, 0x1000, 1);

	read_reg_value(io_master_info,0xDE01, &FMC);

	if (FMC != 0x1000)
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] IF FMC register value 1st warning : %04x", FMC);
		write_reg_value(io_master_info,0xDE01, 0x1000,1);
		FMC = 0; // initialize FMC value

		read_reg_value(io_master_info,0xDE01, &FMC);
		if (FMC != 0x1000)
		{
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] 2nd IF FMC register value 2nd fail : %04x", FMC);
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] stop firmware download");
			return ERROR_FW_DOWN_FMC;
		}
	}

	/* step 7. erease IF(FW/PID) eFLASH  */
	dw9784_pid_erase(io_master_info);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] start firmware/pid download");
	/* step 8. firmware sequential write to flash */
	for (i = 0; i < PID_SIZE_W; i += DATPKT_SIZE)
	{
		addr = IF_START_ADDRESS + i;
		write_block_16bit_value(io_master_info,addr,g_firmwareContext.fwContentPtr + MCS_SIZE_W + i,DATPKT_SIZE);
	}
	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] write firmware/pid to flash");

#ifdef USE_FW_CHECK
	/* Check by reading fw flash directly to i2c */
	/* It is disabled by default */
	memset(buf_temp, 0, MCS_SIZE_W * sizeof(unsigned short));

	/* step 9. firmware sequential read from flash */
	for (i = 0; i <  PID_SIZE_W; i++)
	{
		addr = IF_START_ADDRESS + i;
		read_reg_value(io_master_info,addr,  &data_buf);
		buf_temp[i] = data_buf;
	}
	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] read firmware/pid from flash");
	/* step 10. firmware verify */
	for (i = 0; i < PID_SIZE_W; i++)
	{
		if (g_firmwareContext.fwContentPtr[i + MCS_SIZE_W] != buf_temp[i])
		{
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] firmware/pid verify fail! ADDR:%04X -- firmware:%04x -- READ:%04x ", MCS_START_ADDRESS+i, g_firmwareContext.fwContentPtr[i + MCS_SIZE_W], buf_temp[i]);
			return ERROR_FW_VERIFY;
		}
	}
	CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] firmware/pid verification pass");
#endif
	/* step 14. ic reboot */
	dw9784_ois_reset(io_master_info);

	/* step 15. check fw_checksum */
	if(dw9784_checksum_fw_chk(io_master_info) == 0)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] fw download success.");
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_fw] finish");
		return EOK;
	} else
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] fw download cheksum fail.");
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_fw] finish");
		return ERROR_FW_CHECKSUM;
	}
}



static void GenerateFirmwareContexts(void)
{
	g_firmwareContext.version = DW9784_FW_VERSION;
	g_firmwareContext.driverIc = 0x9784;
	g_firmwareContext.fwContentPtr = DW9784_FW;
	g_downloadByForce = 0;
	g_updateFw = 0;

	/* select phone gyro type & direction here */
	set_gyro_type = GYRO_TDK_ICM42631;
	gyro_arrangement = GYRO_FRONT_LAYOUT;	/* 0:front side, 1:back side */
	gyro_degree = GYRO_DEGREE_0;			/* 0: 0deg, 90: 90deg, 180:180deg, 270:270deg */
}

static int dw9784_download_open_camera(struct camera_io_master *io_master_info)
{
	int err_whoami = 0;
	unsigned short pre_module_state = 0; /* 0x0000: normal, 0xFFFF: abnormal */
	static int updata_flag = 0;

	if(updata_flag)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] no need to update");
		return EOK;
	}

	GenerateFirmwareContexts();

	err_whoami = dw9784_whoami_chk(io_master_info);
	if (err_whoami == ERROR_SECOND_ID)
	{
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] failed to check the second_id(0x0020) inside the ic");
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] stop the dw9784 ic boot operation");
		return ERROR_WHO_AMI;
	}

	if (dw9784_chip_id_chk(io_master_info) == DW9784_CHIP_ID)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] dw9784 chip_id check pass");
	} else
	{
		g_downloadByForce = 1;
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] dw9784 chip_id check failed");
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_checksum_fw_chk(io_master_info) == ERROR_FW_CHECKSUM)
	{
		g_downloadByForce = 1;
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] The firmware checksum check of the flash memory failed.");
		CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] force to recovery firmware");
	}

	if (dw9784_fw_ver_chk(io_master_info) != DW9784_FW_VERSION )
	{
		g_updateFw = 1;
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] current fw is not the latest version");
	}else
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] the firmware version is the latest version, ver: 0x%04X", DW9784_FW_VERSION);
	}

	if (g_downloadByForce == 1)
	{
		pre_module_state = 0xFFFF; /* abnormal state */
	}else if( g_updateFw == 1)
	{
		pre_module_state = 0x0000; /* normal state */
	}

	if (g_downloadByForce || g_updateFw)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] start downloading the latest version firmware, ver: 0x%04X", DW9784_FW_VERSION);
		if(dw9784_download_fw(io_master_info,pre_module_state) == EOK)
		{
			/* fw download success */
			CAM_INFO(CAM_ACTUATOR,"[dw9784_download_open_camera] complete fw download");
		}else
		{
			/* fw download failed */
			dw9784_shutdown_mode(io_master_info);
			CAM_ERR(CAM_ACTUATOR,"[dw9784_download_open_camera] enter ic shutdown(sleep) mode");
			return ERROR_FW_DWON_FAIL;
		}
	}

#if 0
	/* if SET has to change differnt gyro type, the code below applies */
	store_flag += dw9784_set_gyro_select(set_gyro_type);
	store_flag += dw9784_gyro_direction_setting(gyro_arrangement, gyro_degree);

	if (store_flag)
	{
		ret = dw9784_set_cal_store();
	}
#endif
	updata_flag = 1;
	return EOK;
}

static int dw9784_set_cal_store(struct camera_io_master *io_master_info)
{
	CAM_INFO(CAM_ACTUATOR,"[dw9784_set_cal_store] start");
	write_reg_value(io_master_info,0x7012, 0x000A,0); //Set store mode

	//When store is done, status changes to 0xA000
	if(dw9784_wait_check_register(io_master_info,0x7010, 0xA000) == EOK) {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_set_cal_store] successful entry into store mode");
	}
	else {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_set_cal_store] failed to enter store mode");
		return ERROR_CAL_STORE;
	}

	dw9784_code_pt_off(io_master_info); /* code protection off */
	write_reg_value(io_master_info,0x700F, 0x5959,1); //Set protect code
	write_reg_value(io_master_info,0x7011, 0x0001,40); //Execute store

	//When store is done, status changes to 0xA001
	if(dw9784_wait_check_register(io_master_info,0x7010, 0xA001) == EOK) {
		dw9784_ois_reset(io_master_info);
		CAM_INFO(CAM_ACTUATOR,"[dw9784_set_cal_store] finish");
	}
	else {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_set_cal_store] store function fail");
		return ERROR_CAL_STORE;
	}
	return EOK;
}

#ifndef USE_GYRO_OFS_ZERO
static int dw9784_gyro_ofs_calibration(struct camera_io_master *io_master_info)
{
	int msg = 0;
	uint32_t x_ofs, y_ofs, gyro_status;

	if(!DW9784_calibration_enable)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]DW9784 calibration disable");
		return EOK;
	}

	CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] start");


	DW9784_calibration_enable = 0;

	dw9784_ois_reset(io_master_info);

	write_reg_value(io_master_info,0x7012, 0x0006,1); // gyro offset calibration

	if(dw9784_wait_check_register(io_master_info,0x7010, 0x6000) == EOK) {
		write_reg_value(io_master_info,0x7011, 0x0001,100);   // gyro ofs calibration execute command
	}
	else {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] FUNC_FAIL");
		DW9784_calibration_status = STATUS_CALIBRATION_FAIL;//calibration fail
		return ERROR_GYRO_OFS_CAL;
	}
	if(dw9784_wait_check_register(io_master_info,0x7010, 0x6001) == EOK) { // when calibration is done, Status changes to 0x6001
		CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]calibration function finish");
	}
	else {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]calibration function error");
		DW9784_calibration_status = STATUS_CALIBRATION_FAIL;//calibration fail
		return ERROR_GYRO_OFS_CAL;
	}

	read_reg_value(io_master_info,0x7180, &x_ofs); /* x gyro offset */
	read_reg_value(io_master_info,0x7181, &y_ofs); /* y gyro offset */
	read_reg_value(io_master_info,0x7195, &gyro_status); /* gyro offset status */
	CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]x gyro offset: 0x%04X(%d)", x_ofs, (short)x_ofs);
	CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]y gyro offset: 0x%04X(%d)", y_ofs, (short)y_ofs);
	CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration]gyro_status: 0x%04X", gyro_status);

	if( (gyro_status & 0x8000)== 0x8000) {	/* Read Gyro offset cailbration result status */
		if ((gyro_status & 0x1) == X_AXIS_GYRO_OFS_PASS) {
			msg = EOK;
			CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x gyro ofs cal pass");
		}else
		{
			msg += X_AXIS_GYRO_OFS_FAIL;
			CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x gyro ofs cal fail");
		}

		if ( (gyro_status & 0x10) == X_AXIS_GYRO_OFS_OVER_MAX_LIMIT) {
			msg += X_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x gyro ofs over the max. limit");
		}

		if ((gyro_status & 0x2) == Y_AXIS_GYRO_OFS_PASS) {
			msg += EOK;
			CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] y gyro ofs cal pass");
		}else
		{
			msg += Y_AXIS_GYRO_OFS_FAIL;
			CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] y gyro ofs cal fail");
		}

		if ( (gyro_status & 0x20) == Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT) {
			msg += Y_AXIS_GYRO_OFS_OVER_MAX_LIMIT;
			CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] y gyro ofs over the max. limit");
		}

		if ( (gyro_status & 0x800) == XY_AXIS_CHECK_GYRO_RAW_DATA) {
			msg += XY_AXIS_CHECK_GYRO_RAW_DATA;
			CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] check the x/y gyro raw data");
		}
		CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");

		if(msg == EOK)
		{
			msg = dw9784_set_cal_store(io_master_info);
		}
		if(msg == EOK)
		{
			DW9784_calibration_status = STATUS_CALIBRATION_SUCCEED;//calibration succeed
		}
		else
		{
			DW9784_calibration_status = STATUS_CALIBRATION_FAIL;//calibration fail
		}
		return msg;
	}
	else {
		CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration done fail");
		CAM_ERR(CAM_ACTUATOR,"[dw9784_gyro_ofs_calibration] x/y gyro ofs calibration finish");
		DW9784_calibration_status = STATUS_CALIBRATION_FAIL;//calibration fail
		return GYRO_OFS_CAL_DONE_FAIL;
	}
}
#else
static int dw9784_gyro_ofs_set_zero(struct camera_io_master *io_master_info)
{
	static int set_zero_flag = 0;
	uint32_t x_data=0,y_data=0;
	int ret = 0;

	if(set_zero_flag)
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_set_zero] no need to set 0");
		return EOK;
	}

	dw9784_ois_reset(io_master_info);

	read_reg_value(io_master_info,0x7180, &x_data);
	read_reg_value(io_master_info,0x7181, &y_data);

	CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_set_zero]0x7180 = %d, 0x7181 = %d",x_data,y_data);

	if((x_data == 0)&&(y_data == 0))
	{
		CAM_INFO(CAM_ACTUATOR,"[dw9784_gyro_ofs_set_zero]ofs_set had been set to 0");
		set_zero_flag =1;
		return EOK;
	}

	write_reg_value(io_master_info,0x7180, 0x0000,0);
	write_reg_value(io_master_info,0x7181, 0x0000,0);

	ret = dw9784_set_cal_store(io_master_info);
	if(ret == EOK)
	{
		set_zero_flag = 1;
	}
	return ret;
}
#endif


int32_t cam_actuator_construct_default_power_setting(
	struct cam_sensor_power_ctrl_t *power_info)
{
	int rc = 0;

	power_info->power_setting_size = 1;
	power_info->power_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_setting)
		return -ENOMEM;

	power_info->power_setting[0].seq_type = SENSOR_VAF;
	power_info->power_setting[0].seq_val = CAM_VAF;
	power_info->power_setting[0].config_val = 1;
	power_info->power_setting[0].delay = 2;

	power_info->power_down_setting_size = 1;
	power_info->power_down_setting =
		kzalloc(sizeof(struct cam_sensor_power_setting),
			GFP_KERNEL);
	if (!power_info->power_down_setting) {
		rc = -ENOMEM;
		goto free_power_settings;
	}

	power_info->power_down_setting[0].seq_type = SENSOR_VAF;
	power_info->power_down_setting[0].seq_val = CAM_VAF;
	power_info->power_down_setting[0].config_val = 0;

	return rc;

free_power_settings:
	kfree(power_info->power_setting);
	power_info->power_setting = NULL;
	power_info->power_setting_size = 0;
	return rc;
}

static int32_t cam_actuator_power_up(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	struct cam_hw_soc_info  *soc_info =
		&a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;
	struct cam_sensor_power_ctrl_t *power_info;

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	if ((power_info->power_setting == NULL) &&
		(power_info->power_down_setting == NULL)) {
		CAM_INFO(CAM_ACTUATOR,
			"Using default power settings");
		rc = cam_actuator_construct_default_power_setting(power_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Construct default actuator power setting failed.");
			return rc;
		}
	}

	/* Parse and fill vreg params for power up settings */
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_setting,
		power_info->power_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params for power up rc:%d", rc);
		return rc;
	}

	/* Parse and fill vreg params for power down settings*/
	rc = msm_camera_fill_vreg_params(
		&a_ctrl->soc_info,
		power_info->power_down_setting,
		power_info->power_down_setting_size);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed to fill vreg params power down rc:%d", rc);
		return rc;
	}

	power_info->dev = soc_info->dev;

	rc = cam_sensor_core_power_up(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR,
			"failed in actuator power up rc %d", rc);
		return rc;
	}

	rc = camera_io_init(&a_ctrl->io_master_info);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "cci init failed: rc: %d", rc);
		goto cci_failure;
	}

	return rc;
cci_failure:
	if (cam_sensor_util_power_down(power_info, soc_info))
		CAM_ERR(CAM_ACTUATOR, "Power down failure");

	return rc;
}

static int32_t cam_actuator_power_down(struct cam_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	struct cam_sensor_power_ctrl_t *power_info;
	struct cam_hw_soc_info *soc_info = &a_ctrl->soc_info;
	struct cam_actuator_soc_private  *soc_private;

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "failed: a_ctrl %pK", a_ctrl);
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;
	soc_info = &a_ctrl->soc_info;

	if (!power_info) {
		CAM_ERR(CAM_ACTUATOR, "failed: power_info %pK", power_info);
		return -EINVAL;
	}
	rc = cam_sensor_util_power_down(power_info, soc_info);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "power down the core is failed:%d", rc);
		return rc;
	}

	camera_io_release(&a_ctrl->io_master_info);

	return rc;
}

static int32_t cam_actuator_i2c_modes_util(
	struct camera_io_master *io_master_info,
	struct i2c_settings_list *i2c_list)
{
	int32_t rc = 0;
	uint32_t i, size;

	if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_RANDOM) {
		rc = camera_io_dev_write(io_master_info,
			&(i2c_list->i2c_settings));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to random write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_SEQ) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			CAM_SENSOR_I2C_WRITE_SEQ);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to seq write I2C settings: %d",
				rc);
			return rc;
			}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_WRITE_BURST) {
		rc = camera_io_dev_write_continuous(
			io_master_info,
			&(i2c_list->i2c_settings),
			CAM_SENSOR_I2C_WRITE_BURST);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to burst write I2C settings: %d",
				rc);
			return rc;
		}
	} else if (i2c_list->op_code == CAM_SENSOR_I2C_POLL) {
		size = i2c_list->i2c_settings.size;
		for (i = 0; i < size; i++) {
			rc = camera_io_dev_poll(
			io_master_info,
			i2c_list->i2c_settings.reg_setting[i].reg_addr,
			i2c_list->i2c_settings.reg_setting[i].reg_data,
			i2c_list->i2c_settings.reg_setting[i].data_mask,
			i2c_list->i2c_settings.addr_type,
			i2c_list->i2c_settings.data_type,
			i2c_list->i2c_settings.reg_setting[i].delay);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"i2c poll apply setting Fail: %d", rc);
				return rc;
			}
		}
	}

	return rc;
}

int32_t cam_actuator_slaveInfo_pkt_parser(struct cam_actuator_ctrl_t *a_ctrl,
	uint32_t *cmd_buf, size_t len)
{
	int32_t rc = 0;
	struct cam_cmd_i2c_info *i2c_info;

	if (!a_ctrl || !cmd_buf || (len < sizeof(struct cam_cmd_i2c_info))) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	i2c_info = (struct cam_cmd_i2c_info *)cmd_buf;
	if (a_ctrl->io_master_info.master_type == CCI_MASTER) {
		a_ctrl->io_master_info.cci_client->cci_i2c_master =
			a_ctrl->cci_i2c_master;
		a_ctrl->io_master_info.cci_client->i2c_freq_mode =
			i2c_info->i2c_freq_mode;
		a_ctrl->io_master_info.cci_client->sid =
			i2c_info->slave_addr >> 1;
		CAM_DBG(CAM_ACTUATOR, "Slave addr: 0x%x Freq Mode: %d",
			i2c_info->slave_addr, i2c_info->i2c_freq_mode);
	} else if (a_ctrl->io_master_info.master_type == I2C_MASTER) {
		a_ctrl->io_master_info.client->addr = i2c_info->slave_addr;
		CAM_DBG(CAM_ACTUATOR, "Slave addr: 0x%x", i2c_info->slave_addr);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Invalid Master type: %d",
			a_ctrl->io_master_info.master_type);
		 rc = -EINVAL;
	}

	return rc;
}

int32_t cam_actuator_apply_settings(struct cam_actuator_ctrl_t *a_ctrl,
	struct i2c_settings_array *i2c_set)
{
	struct i2c_settings_list *i2c_list;
	int32_t rc = 0;

	if (a_ctrl == NULL || i2c_set == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	if (i2c_set->is_settings_valid != 1) {
		CAM_ERR(CAM_ACTUATOR, " Invalid settings");
		return -EINVAL;
	}

	list_for_each_entry(i2c_list,
		&(i2c_set->list_head), list) {
		rc = cam_actuator_i2c_modes_util(
			&(a_ctrl->io_master_info),
			i2c_list);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to apply settings: %d",
				rc);
		} else {
			CAM_DBG(CAM_ACTUATOR,
				"Success:request ID: %d",
				i2c_set->request_id);
		}
	}

	return rc;
}

int32_t cam_actuator_apply_request(struct cam_req_mgr_apply_request *apply)
{
	int32_t rc = 0, request_id, del_req_id;
	struct cam_actuator_ctrl_t *a_ctrl = NULL;

	if (!apply) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Input Args");
		return -EINVAL;
	}

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(apply->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}
	request_id = apply->request_id % MAX_PER_FRAME_ARRAY;

	trace_cam_apply_req("Actuator", a_ctrl->soc_info.index, apply->request_id, apply->link_hdl);

	CAM_DBG(CAM_ACTUATOR, "Request Id: %lld", apply->request_id);
	mutex_lock(&(a_ctrl->actuator_mutex));
	if ((apply->request_id ==
		a_ctrl->i2c_data.per_frame[request_id].request_id) &&
		(a_ctrl->i2c_data.per_frame[request_id].is_settings_valid)
		== 1) {
		rc = cam_actuator_apply_settings(a_ctrl,
			&a_ctrl->i2c_data.per_frame[request_id]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in applying the request: %lld\n",
				apply->request_id);
			goto release_mutex;
		}
	}
	del_req_id = (request_id +
		MAX_PER_FRAME_ARRAY - MAX_SYSTEM_PIPELINE_DELAY) %
		MAX_PER_FRAME_ARRAY;

	if (apply->request_id >
		a_ctrl->i2c_data.per_frame[del_req_id].request_id) {
		a_ctrl->i2c_data.per_frame[del_req_id].request_id = 0;
		rc = delete_request(&a_ctrl->i2c_data.per_frame[del_req_id]);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Fail deleting the req: %d err: %d\n",
				del_req_id, rc);
			goto release_mutex;
		}
	} else {
		CAM_DBG(CAM_ACTUATOR, "No Valid Req to clean Up");
	}

release_mutex:
	mutex_unlock(&(a_ctrl->actuator_mutex));
	return rc;
}

int32_t cam_actuator_establish_link(
	struct cam_req_mgr_core_dev_link_setup *link)
{
	struct cam_actuator_ctrl_t *a_ctrl = NULL;

	if (!link) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(link->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (link->link_enable) {
		a_ctrl->bridge_intf.link_hdl = link->link_hdl;
		a_ctrl->bridge_intf.crm_cb = link->crm_cb;
	} else {
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.crm_cb = NULL;
	}
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return 0;
}

static int cam_actuator_update_req_mgr(
	struct cam_actuator_ctrl_t *a_ctrl,
	struct cam_packet *csl_packet)
{
	int rc = 0;
	struct cam_req_mgr_add_request add_req;

	memset(&add_req, 0, sizeof(add_req));
	add_req.link_hdl = a_ctrl->bridge_intf.link_hdl;
	add_req.req_id = csl_packet->header.request_id;
	add_req.dev_hdl = a_ctrl->bridge_intf.device_hdl;

	if (a_ctrl->bridge_intf.crm_cb &&
		a_ctrl->bridge_intf.crm_cb->add_req) {
		rc = a_ctrl->bridge_intf.crm_cb->add_req(&add_req);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Adding request: %llu failed: rc: %d",
				csl_packet->header.request_id, rc);
			return rc;
		}
		CAM_DBG(CAM_ACTUATOR, "Request Id: %lld added to CRM",
			add_req.req_id);
	} else {
		CAM_ERR(CAM_ACTUATOR, "Can't add Request ID: %lld to CRM",
			csl_packet->header.request_id);
		rc = -EINVAL;
	}

	return rc;
}

int32_t cam_actuator_publish_dev_info(struct cam_req_mgr_device_info *info)
{
	if (!info) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	info->dev_id = CAM_REQ_MGR_DEVICE_ACTUATOR;
	strlcpy(info->name, CAM_ACTUATOR_NAME, sizeof(info->name));
	info->p_delay = 1;
	info->trigger = CAM_TRIGGER_POINT_SOF;

	return 0;
}

int32_t cam_actuator_i2c_pkt_parse(struct cam_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	int32_t  rc = 0;
	int32_t  i = 0;
	uint32_t total_cmd_buf_in_bytes = 0;
	size_t   len_of_buff = 0;
	size_t   remain_len = 0;
	uint32_t *offset = NULL;
	uint32_t *cmd_buf = NULL;
	uintptr_t generic_ptr;
	uintptr_t generic_pkt_ptr;
	struct common_header      *cmm_hdr = NULL;
	struct cam_control        *ioctl_ctrl = NULL;
	struct cam_packet         *csl_packet = NULL;
	struct cam_config_dev_cmd config;
	struct i2c_data_settings  *i2c_data = NULL;
	struct i2c_settings_array *i2c_reg_settings = NULL;
	struct cam_cmd_buf_desc   *cmd_desc = NULL;
	struct cam_actuator_soc_private *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!a_ctrl || !arg) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;

	power_info = &soc_private->power_info;

	ioctl_ctrl = (struct cam_control *)arg;
	if (copy_from_user(&config,
		u64_to_user_ptr(ioctl_ctrl->handle),
		sizeof(config)))
		return -EFAULT;
	rc = cam_mem_get_cpu_buf(config.packet_handle,
		&generic_pkt_ptr, &len_of_buff);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Error in converting command Handle %d",
			rc);
		return rc;
	}

	remain_len = len_of_buff;
	if ((sizeof(struct cam_packet) > len_of_buff) ||
		((size_t)config.offset >= len_of_buff -
		sizeof(struct cam_packet))) {
		CAM_ERR(CAM_ACTUATOR,
			"Inval cam_packet strut size: %zu, len_of_buff: %zu",
			 sizeof(struct cam_packet), len_of_buff);
		rc = -EINVAL;
		goto end;
	}

	remain_len -= (size_t)config.offset;
	csl_packet = (struct cam_packet *)
			(generic_pkt_ptr + (uint32_t)config.offset);

	if (cam_packet_util_validate_packet(csl_packet,
		remain_len)) {
		CAM_ERR(CAM_ACTUATOR, "Invalid packet params");
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_ACTUATOR, "Pkt opcode: %d",	csl_packet->header.op_code);

	if ((csl_packet->header.op_code & 0xFFFFFF) !=
		CAM_ACTUATOR_PACKET_OPCODE_INIT &&
		csl_packet->header.request_id <= a_ctrl->last_flush_req
		&& a_ctrl->last_flush_req != 0) {
		CAM_DBG(CAM_ACTUATOR,
			"reject request %lld, last request to flush %lld",
			csl_packet->header.request_id, a_ctrl->last_flush_req);
		rc = -EINVAL;
		goto end;
	}

	if (csl_packet->header.request_id > a_ctrl->last_flush_req)
		a_ctrl->last_flush_req = 0;

	offset = (uint32_t *)&csl_packet->payload;
	offset += (csl_packet->cmd_buf_offset / sizeof(uint32_t));
	cmd_desc = (struct cam_cmd_buf_desc *)(offset);
	rc = cam_packet_util_validate_cmd_desc(cmd_desc);
	if (rc) {
		CAM_ERR(CAM_ACTUATOR, "Invalid cmd desc ret: %d", rc);
		return rc;
	}

	switch (csl_packet->header.op_code & 0xFFFFFF) {
	case CAM_ACTUATOR_PACKET_OPCODE_INIT:
		if (!csl_packet->num_cmd_buf) {
			CAM_ERR(CAM_ACTUATOR, "Invalid num_cmd_buffer = %d",
				csl_packet->num_cmd_buf);
			return -EINVAL;
		}
		/* Loop through multiple command buffers */
		for (i = 0; i < csl_packet->num_cmd_buf; i++) {
			total_cmd_buf_in_bytes = cmd_desc[i].length;
			if (!total_cmd_buf_in_bytes)
				continue;
			rc = cam_mem_get_cpu_buf(cmd_desc[i].mem_handle,
					&generic_ptr, &len_of_buff);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR, "Failed to get cpu buf");
				goto end;
			}
			cmd_buf = (uint32_t *)generic_ptr;
			if (!cmd_buf) {
				CAM_ERR(CAM_ACTUATOR, "invalid cmd buf");
				rc = -EINVAL;
				goto end;
			}
			if ((len_of_buff < sizeof(struct common_header)) ||
				(cmd_desc[i].offset > (len_of_buff -
				sizeof(struct common_header)))) {
				CAM_ERR(CAM_ACTUATOR,
					"Invalid length for sensor cmd");
				rc = -EINVAL;
				goto end;
			}
			remain_len = len_of_buff - cmd_desc[i].offset;
			cmd_buf += cmd_desc[i].offset / sizeof(uint32_t);
			cmm_hdr = (struct common_header *)cmd_buf;

			switch (cmm_hdr->cmd_type) {
			case CAMERA_SENSOR_CMD_TYPE_I2C_INFO:
				CAM_DBG(CAM_ACTUATOR,
					"Received slave info buffer");
				rc = cam_actuator_slaveInfo_pkt_parser(
					a_ctrl, cmd_buf, remain_len);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed to parse slave info: %d", rc);
					goto end;
				}
				break;
			case CAMERA_SENSOR_CMD_TYPE_PWR_UP:
			case CAMERA_SENSOR_CMD_TYPE_PWR_DOWN:
				CAM_DBG(CAM_ACTUATOR,
					"Received power settings buffer");
				rc = cam_sensor_update_power_settings(
					cmd_buf,
					total_cmd_buf_in_bytes,
					power_info, remain_len);
				if (rc) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed:parse power settings: %d",
					rc);
					goto end;
				}
				break;
			default:
				CAM_DBG(CAM_ACTUATOR,
					"Received initSettings buffer");
				i2c_data = &(a_ctrl->i2c_data);
				i2c_reg_settings =
					&i2c_data->init_settings;

				i2c_reg_settings->request_id = 0;
				i2c_reg_settings->is_settings_valid = 1;
				rc = cam_sensor_i2c_command_parser(
					&a_ctrl->io_master_info,
					i2c_reg_settings,
					&cmd_desc[i], 1, NULL);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
					"Failed:parse init settings: %d",
					rc);
					goto end;
				}
				break;
			}
		}

		if (a_ctrl->cam_act_state == CAM_ACTUATOR_ACQUIRE) {
			rc = cam_actuator_power_up(a_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					" Actuator Power up failed");
				goto end;
			}
			a_ctrl->cam_act_state = CAM_ACTUATOR_CONFIG;
		}

		rc = dw9784_download_open_camera(&(a_ctrl->io_master_info));
		if(rc !=EOK)
		{
			CAM_ERR(CAM_ACTUATOR, "dw9784_download_open_camera failed");
			rc = cam_actuator_power_down(a_ctrl);
			if (rc < 0)
			{
				CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
			}
			else
			{
				rc = cam_actuator_power_up(a_ctrl);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
						" Actuator Power up failed");
					goto end;
				}
			}
		}
#ifndef USE_GYRO_OFS_ZERO
		rc = dw9784_gyro_ofs_calibration(&(a_ctrl->io_master_info));
#else
		rc = dw9784_gyro_ofs_set_zero(&(a_ctrl->io_master_info));
#endif
		if(rc !=EOK)
		{
			CAM_ERR(CAM_ACTUATOR, "dw9784_gyro_ofs_set_zero failed");
			rc = cam_actuator_power_down(a_ctrl);
			if (rc < 0)
			{
				CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
			}
			else
			{
				rc = cam_actuator_power_up(a_ctrl);
				if (rc < 0) {
					CAM_ERR(CAM_ACTUATOR,
						" Actuator Power up failed");
					goto end;
				}
			}
		}

		rc = cam_actuator_apply_settings(a_ctrl,
			&a_ctrl->i2c_data.init_settings);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Cannot apply Init settings");
			goto end;
		}

		/* Delete the request even if the apply is failed */
		rc = delete_request(&a_ctrl->i2c_data.init_settings);
		if (rc < 0) {
			CAM_WARN(CAM_ACTUATOR,
				"Fail in deleting the Init settings");
			rc = 0;
		}
		break;
	case CAM_ACTUATOR_PACKET_AUTO_MOVE_LENS:
		if (!csl_packet->num_cmd_buf) {
			CAM_ERR(CAM_ACTUATOR, "Invalid num_cmd_buffer = %d",
				csl_packet->num_cmd_buf);
			return -EINVAL;
		}
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Not in right state to move lens: %d",
				a_ctrl->cam_act_state);
			goto end;
		}
		a_ctrl->setting_apply_state = ACT_APPLY_SETTINGS_NOW;

		i2c_data = &(a_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->init_settings;

		i2c_data->init_settings.request_id =
			csl_packet->header.request_id;
		i2c_reg_settings->is_settings_valid = 1;
		rc = cam_sensor_i2c_command_parser(
			&a_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Auto move lens parsing failed: %d", rc);
			goto end;
		}
		rc = cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in adding request to request manager");
			goto end;
		}
		break;
	case CAM_ACTUATOR_PACKET_MANUAL_MOVE_LENS:
		if (!csl_packet->num_cmd_buf) {
			CAM_ERR(CAM_ACTUATOR, "Invalid num_cmd_buffer = %d",
				csl_packet->num_cmd_buf);
			return -EINVAL;
		}
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Not in right state to move lens: %d",
				a_ctrl->cam_act_state);
			goto end;
		}

		a_ctrl->setting_apply_state = ACT_APPLY_SETTINGS_LATER;
		i2c_data = &(a_ctrl->i2c_data);
		i2c_reg_settings = &i2c_data->per_frame[
			csl_packet->header.request_id % MAX_PER_FRAME_ARRAY];

		 i2c_reg_settings->request_id =
			csl_packet->header.request_id;
		i2c_reg_settings->is_settings_valid = 1;
		rc = cam_sensor_i2c_command_parser(
			&a_ctrl->io_master_info,
			i2c_reg_settings,
			cmd_desc, 1, NULL);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Manual move lens parsing failed: %d", rc);
			goto end;
		}

		rc = cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in adding request to request manager");
			goto end;
		}
		break;
	case CAM_PKT_NOP_OPCODE:
		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			CAM_WARN(CAM_ACTUATOR,
				"Received NOP packets in invalid state: %d",
				a_ctrl->cam_act_state);
			rc = -EINVAL;
			goto end;
		}
		rc = cam_actuator_update_req_mgr(a_ctrl, csl_packet);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in adding request to request manager");
			goto end;
		}
		break;
	case CAM_ACTUATOR_PACKET_OPCODE_READ: {
		struct cam_buf_io_cfg *io_cfg;
		struct i2c_settings_array i2c_read_settings;

		if (a_ctrl->cam_act_state < CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Not in right state to read actuator: %d",
				a_ctrl->cam_act_state);
			goto end;
		}
		CAM_DBG(CAM_ACTUATOR, "number of I/O configs: %d:",
			csl_packet->num_io_configs);
		if (csl_packet->num_io_configs == 0) {
			CAM_ERR(CAM_ACTUATOR, "No I/O configs to process");
			rc = -EINVAL;
			goto end;
		}

		INIT_LIST_HEAD(&(i2c_read_settings.list_head));

		io_cfg = (struct cam_buf_io_cfg *) ((uint8_t *)
			&csl_packet->payload +
			csl_packet->io_configs_offset);

		if (io_cfg == NULL) {
			CAM_ERR(CAM_ACTUATOR, "I/O config is invalid(NULL)");
			rc = -EINVAL;
			goto end;
		}

		i2c_read_settings.is_settings_valid = 1;
		i2c_read_settings.request_id = 0;
		rc = cam_sensor_i2c_command_parser(&a_ctrl->io_master_info,
			&i2c_read_settings,
			cmd_desc, 1, io_cfg);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"actuator read pkt parsing failed: %d", rc);
			goto end;
		}

		rc = cam_sensor_i2c_read_data(
			&i2c_read_settings,
			&a_ctrl->io_master_info);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "cannot read data, rc:%d", rc);
			delete_request(&i2c_read_settings);
			goto end;
		}

		rc = delete_request(&i2c_read_settings);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in deleting the read settings");
			goto end;
		}
		break;
		}
	default:
		CAM_ERR(CAM_ACTUATOR, "Wrong Opcode: %d",
			csl_packet->header.op_code & 0xFFFFFF);
		rc = -EINVAL;
		goto end;
	}

end:
	return rc;
}

void cam_actuator_shutdown(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	struct cam_actuator_soc_private  *soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	struct cam_sensor_power_ctrl_t *power_info =
		&soc_private->power_info;

	if (a_ctrl->cam_act_state == CAM_ACTUATOR_INIT)
		return;

	if (a_ctrl->cam_act_state >= CAM_ACTUATOR_CONFIG) {
		rc = cam_actuator_power_down(a_ctrl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "Actuator Power down failed");
		a_ctrl->cam_act_state = CAM_ACTUATOR_ACQUIRE;
	}

	if (a_ctrl->cam_act_state >= CAM_ACTUATOR_ACQUIRE) {
		rc = cam_destroy_device_hdl(a_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "destroying  dhdl failed");
		a_ctrl->bridge_intf.device_hdl = -1;
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.session_hdl = -1;
	}

	kfree(power_info->power_setting);
	kfree(power_info->power_down_setting);
	power_info->power_setting = NULL;
	power_info->power_down_setting = NULL;
	power_info->power_setting_size = 0;
	power_info->power_down_setting_size = 0;
	a_ctrl->last_flush_req = 0;

	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
}

int32_t cam_actuator_driver_cmd(struct cam_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	int rc = 0;
	struct cam_control *cmd = (struct cam_control *)arg;
	struct cam_actuator_soc_private *soc_private = NULL;
	struct cam_sensor_power_ctrl_t  *power_info = NULL;

	if (!a_ctrl || !cmd) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args");
		return -EINVAL;
	}

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;

	power_info = &soc_private->power_info;

	if (cmd->handle_type != CAM_HANDLE_USER_POINTER) {
		CAM_ERR(CAM_ACTUATOR, "Invalid handle type: %d",
			cmd->handle_type);
		return -EINVAL;
	}

	CAM_DBG(CAM_ACTUATOR, "Opcode to Actuator: %d", cmd->op_code);

	mutex_lock(&(a_ctrl->actuator_mutex));
	switch (cmd->op_code) {
	case CAM_ACQUIRE_DEV: {
		struct cam_sensor_acquire_dev actuator_acq_dev;
		struct cam_create_dev_hdl bridge_params;

		if (a_ctrl->bridge_intf.device_hdl != -1) {
			CAM_ERR(CAM_ACTUATOR, "Device is already acquired");
			rc = -EINVAL;
			goto release_mutex;
		}
		rc = copy_from_user(&actuator_acq_dev,
			u64_to_user_ptr(cmd->handle),
			sizeof(actuator_acq_dev));
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copying from user\n");
			goto release_mutex;
		}

		bridge_params.session_hdl = actuator_acq_dev.session_handle;
		bridge_params.ops = &a_ctrl->bridge_intf.ops;
		bridge_params.v4l2_sub_dev_flag = 0;
		bridge_params.media_entity_flag = 0;
		bridge_params.priv = a_ctrl;
		bridge_params.dev_id = CAM_ACTUATOR;

		actuator_acq_dev.device_handle =
			cam_create_device_hdl(&bridge_params);
		if (actuator_acq_dev.device_handle <= 0) {
			rc = -EFAULT;
			CAM_ERR(CAM_ACTUATOR, "Can not create device handle");
			goto release_mutex;
		}
		a_ctrl->bridge_intf.device_hdl = actuator_acq_dev.device_handle;
		a_ctrl->bridge_intf.session_hdl =
			actuator_acq_dev.session_handle;

		CAM_DBG(CAM_ACTUATOR, "Device Handle: %d",
			actuator_acq_dev.device_handle);
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&actuator_acq_dev,
			sizeof(struct cam_sensor_acquire_dev))) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}

		a_ctrl->cam_act_state = CAM_ACTUATOR_ACQUIRE;
	}
		break;
	case CAM_RELEASE_DEV: {
		if (a_ctrl->cam_act_state == CAM_ACTUATOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
				"Cant release actuator: in start state");
			goto release_mutex;
		}

		if (a_ctrl->bridge_intf.device_hdl == -1) {
			CAM_ERR(CAM_ACTUATOR, "link hdl: %d device hdl: %d",
				a_ctrl->bridge_intf.device_hdl,
				a_ctrl->bridge_intf.link_hdl);
			rc = -EINVAL;
			goto release_mutex;
		}

		if (a_ctrl->cam_act_state == CAM_ACTUATOR_CONFIG) {
			rc = cam_actuator_power_down(a_ctrl);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"Actuator Power Down Failed");
				goto release_mutex;
			}
		}

		if (a_ctrl->bridge_intf.link_hdl != -1) {
			CAM_ERR(CAM_ACTUATOR,
				"Device [%d] still active on link 0x%x",
				a_ctrl->cam_act_state,
				a_ctrl->bridge_intf.link_hdl);
			rc = -EAGAIN;
			goto release_mutex;
		}

		rc = cam_destroy_device_hdl(a_ctrl->bridge_intf.device_hdl);
		if (rc < 0)
			CAM_ERR(CAM_ACTUATOR, "destroying the device hdl");
		a_ctrl->bridge_intf.device_hdl = -1;
		a_ctrl->bridge_intf.link_hdl = -1;
		a_ctrl->bridge_intf.session_hdl = -1;
		a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
		a_ctrl->last_flush_req = 0;
		kfree(power_info->power_setting);
		kfree(power_info->power_down_setting);
		power_info->power_setting = NULL;
		power_info->power_down_setting = NULL;
		power_info->power_down_setting_size = 0;
		power_info->power_setting_size = 0;
	}
		break;
	case CAM_QUERY_CAP: {
		struct cam_actuator_query_cap actuator_cap = {0};

		actuator_cap.slot_info = a_ctrl->soc_info.index;
		if (copy_to_user(u64_to_user_ptr(cmd->handle),
			&actuator_cap,
			sizeof(struct cam_actuator_query_cap))) {
			CAM_ERR(CAM_ACTUATOR, "Failed Copy to User");
			rc = -EFAULT;
			goto release_mutex;
		}
	}
		break;
	case CAM_START_DEV: {
		if (a_ctrl->cam_act_state != CAM_ACTUATOR_CONFIG) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
			"Not in right state to start : %d",
			a_ctrl->cam_act_state);
			goto release_mutex;
		}
		a_ctrl->cam_act_state = CAM_ACTUATOR_START;
		a_ctrl->last_flush_req = 0;
	}
		break;
	case CAM_STOP_DEV: {
		struct i2c_settings_array *i2c_set = NULL;
		int i;

		if (a_ctrl->cam_act_state != CAM_ACTUATOR_START) {
			rc = -EINVAL;
			CAM_WARN(CAM_ACTUATOR,
			"Not in right state to stop : %d",
			a_ctrl->cam_act_state);
			goto release_mutex;
		}

		for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
			i2c_set = &(a_ctrl->i2c_data.per_frame[i]);

			if (i2c_set->is_settings_valid == 1) {
				rc = delete_request(i2c_set);
				if (rc < 0)
					CAM_ERR(CAM_SENSOR,
						"delete request: %lld rc: %d",
						i2c_set->request_id, rc);
			}
		}
		a_ctrl->last_flush_req = 0;
		a_ctrl->cam_act_state = CAM_ACTUATOR_CONFIG;
	}
		break;
	case CAM_CONFIG_DEV: {
		a_ctrl->setting_apply_state =
			ACT_APPLY_SETTINGS_LATER;
		rc = cam_actuator_i2c_pkt_parse(a_ctrl, arg);
		if (rc < 0) {
			CAM_ERR(CAM_ACTUATOR, "Failed in actuator Parsing");
			goto release_mutex;
		}

		if (a_ctrl->setting_apply_state ==
			ACT_APPLY_SETTINGS_NOW) {
			rc = cam_actuator_apply_settings(a_ctrl,
				&a_ctrl->i2c_data.init_settings);
			if ((rc == -EAGAIN) &&
			(a_ctrl->io_master_info.master_type == CCI_MASTER)) {
				CAM_WARN(CAM_ACTUATOR,
					"CCI HW is in resetting mode:: Reapplying Init settings");
				usleep_range(1000, 1010);
				rc = cam_actuator_apply_settings(a_ctrl,
					&a_ctrl->i2c_data.init_settings);
			}

			if (rc < 0)
				CAM_ERR(CAM_ACTUATOR,
					"Failed to apply Init settings: rc = %d",
					rc);
			/* Delete the request even if the apply is failed */
			rc = delete_request(&a_ctrl->i2c_data.init_settings);
			if (rc < 0) {
				CAM_ERR(CAM_ACTUATOR,
					"Failed in Deleting the Init Pkt: %d",
					rc);
				goto release_mutex;
			}
		}
	}
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid Opcode %d", cmd->op_code);
	}

release_mutex:
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return rc;
}

int32_t cam_actuator_flush_request(struct cam_req_mgr_flush_request *flush_req)
{
	int32_t rc = 0, i;
	uint32_t cancel_req_id_found = 0;
	struct cam_actuator_ctrl_t *a_ctrl = NULL;
	struct i2c_settings_array *i2c_set = NULL;

	if (!flush_req)
		return -EINVAL;

	a_ctrl = (struct cam_actuator_ctrl_t *)
		cam_get_device_priv(flush_req->dev_hdl);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Device data is NULL");
		return -EINVAL;
	}

	if (a_ctrl->i2c_data.per_frame == NULL) {
		CAM_ERR(CAM_ACTUATOR, "i2c frame data is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_ALL) {
		a_ctrl->last_flush_req = flush_req->req_id;
		CAM_DBG(CAM_ACTUATOR, "last reqest to flush is %lld",
			flush_req->req_id);
	}

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++) {
		i2c_set = &(a_ctrl->i2c_data.per_frame[i]);

		if ((flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ)
				&& (i2c_set->request_id != flush_req->req_id))
			continue;

		if (i2c_set->is_settings_valid == 1) {
			rc = delete_request(i2c_set);
			if (rc < 0)
				CAM_ERR(CAM_ACTUATOR,
					"delete request: %lld rc: %d",
					i2c_set->request_id, rc);

			if (flush_req->type ==
				CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ) {
				cancel_req_id_found = 1;
				break;
			}
		}
	}

	if (flush_req->type == CAM_REQ_MGR_FLUSH_TYPE_CANCEL_REQ &&
		!cancel_req_id_found)
		CAM_DBG(CAM_ACTUATOR,
			"Flush request id:%lld not found in the pending list",
			flush_req->req_id);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	return rc;
}
