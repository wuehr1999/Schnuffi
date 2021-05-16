/*
 * vl53l0x.c
 *
 *  Created on: May 10, 2021
 *      Author: jonas
 */
#include "vl53l0x.h"

uint32_t timeout_start_ms;

void startTimeout(VL53_t* tof)
{
	tof->timeout_start_ms = HAL_GetTick();
}

bool checkTimeoutExpired(VL53_t* tof)
{
	return tof->io_timeout > 0 && ((uint16_t)(HAL_GetTick() - tof->timeout_start_ms) > tof->io_timeout);
}

enum regAddr
{
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

typedef struct SequenceStepEnables
{
	bool tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct SequenceStepTimeouts
{
   uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

   uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
   uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

uint8_t readReg(VL53_t* tof, uint8_t reg)
{
	WIRE_BeginTransmission(tof->wire, tof->address);
	WIRE_WriteByte(tof->wire, reg);
	tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
	WIRE_RequestFrom(tof->wire, tof->address, 1, true);
	return WIRE_Read(tof->wire);
}

void writeReg(VL53_t* tof, uint8_t reg, uint8_t value)
{
	WIRE_BeginTransmission(tof->wire, tof->address);
	WIRE_WriteByte(tof->wire, reg);
	WIRE_WriteByte(tof->wire, value);
	tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
}

uint16_t readReg16Bit(VL53_t* tof, uint8_t reg)
{
	 uint16_t value;

	 WIRE_BeginTransmission(tof->wire, tof->address);
	 WIRE_WriteByte(tof->wire, reg);
	 tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
	 WIRE_RequestFrom(tof->wire, tof->address, (uint8_t)2, true);
	 value  = (uint16_t)WIRE_Read(tof->wire) << 8; // value high byte
	 value |=           WIRE_Read(tof->wire);      // value low byte
	 return value;
}
void writeReg16Bit(VL53_t* tof, uint8_t reg, uint16_t value)
{
	WIRE_BeginTransmission(tof->wire, tof->address);
	WIRE_WriteByte(tof->wire, reg);
	WIRE_WriteByte(tof->wire, (value >> 8) & 0xff);
	WIRE_WriteByte(tof->wire, value & 0xff);
	tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
}

void readMulti(VL53_t* tof, uint8_t reg, uint8_t* dst, uint8_t count)
{
	WIRE_BeginTransmission(tof->wire, tof->address);
	WIRE_WriteByte(tof->wire, reg);
	tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
	HAL_Delay(2);
	WIRE_RequestFrom(tof->wire, tof->address, count, true);
	while(count-- > 0)
	{
		*(dst++) = WIRE_Read(tof->wire);
	}
}

void writeMulti(VL53_t* tof, uint8_t reg, uint8_t const * src, uint8_t count)
{
  WIRE_BeginTransmission(tof->wire, tof->address);
  WIRE_WriteByte(tof->wire, reg);

  while (count-- > 0)
  {
    WIRE_WriteByte(tof->wire, *(src++));
  }

  tof->lastStatus = WIRE_EndTransmission(tof->wire, true);
}
bool getSpadInfo(VL53_t* tof, uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(tof, 0x80, 0x01);
  writeReg(tof, 0xFF, 0x01);
  writeReg(tof, 0x00, 0x00);

  writeReg(tof, 0xFF, 0x06);
  writeReg(tof, 0x83, readReg(tof, 0x83) | 0x04);
  writeReg(tof, 0xFF, 0x07);
  writeReg(tof, 0x81, 0x01);

  writeReg(tof, 0x80, 0x01);

  writeReg(tof, 0x94, 0x6b);
  writeReg(tof, 0x83, 0x00);
  startTimeout(tof);
  while (readReg(tof, 0x83) == 0x00)
  {
    if (checkTimeoutExpired(tof)) { return false; }
  }
  writeReg(tof, 0x83, 0x01);
  tmp = readReg(tof, 0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(tof, 0x81, 0x00);
  writeReg(tof, 0xFF, 0x06);
  writeReg(tof, 0x83, readReg(tof, 0x83)  & ~0x04);
  writeReg(tof, 0xFF, 0x01);
  writeReg(tof, 0x00, 0x01);

  writeReg(tof, 0xFF, 0x00);
  writeReg(tof, 0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(VL53_t* tof, SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(tof, SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}


uint8_t decodeVcselPeriod(uint8_t reg_val)
{
	return ((reg_val + 1) << 1);
}

uint32_t calcMacroPeriod(uint8_t vcsel_period_pclks)
{
	return ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
}
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

uint8_t getVcselPulsePeriod(VL53_t* tof, vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(tof, PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(tof, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(VL53_t* tof, SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(tof, VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(tof, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(tof, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(tof, VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(tof, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint16_t encodeTimeout(uint32_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t getMeasurementTimingBudget(VL53_t* tof)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910;
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(tof, &enables);
  getSequenceStepTimeouts(tof, &enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  tof->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

uint8_t encodeVcselPeriod(uint8_t period_pclks)
{
	return (period_pclks >> 1) -1;
}

bool performSingleRefCalibration(VL53_t* tof, uint8_t vhv_init_byte)
{
  writeReg(tof, SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout(tof);
  while ((readReg(tof, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
//    if (checkTimeoutExpired(tof)) { return false; }
  }

  writeReg(tof, SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(tof, SYSRANGE_START, 0x00);

  return true;
}

uint16_t readRangeContinuousMillimeters(VL53_t* tof)
{
  startTimeout(tof);
  while ((readReg(tof, RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired(tof))
    {
      tof->did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = readReg16Bit(tof, RESULT_RANGE_STATUS + 10);

  writeReg(tof, SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

bool VL53_Init(VL53_t* tof, WIRE_t* wire, uint8_t address, GPIO_TypeDef* xsGPIO, uint16_t xsPIN)
{
	tof->wire = wire;
	tof->address = address;
	tof->xsGPIO = xsGPIO;
	tof->xsPIN = xsPIN;
	tof->io_timeout = 0;
	tof->did_timeout = false;

	if(readReg(tof, IDENTIFICATION_MODEL_ID) != 0xEE)
	{
		return false;
	}

//	writeReg(tof, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, readReg(tof, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

	  // "Set I2C standard mode"
	  writeReg(tof, 0x88, 0x00);

	  writeReg(tof, 0x80, 0x01);
	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x00, 0x00);
	  tof->stopVariable = readReg(tof, 0x91);
	  writeReg(tof, 0x00, 0x01);
	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x80, 0x00);

	  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	  writeReg(tof, MSRC_CONFIG_CONTROL, readReg(tof, MSRC_CONFIG_CONTROL) | 0x12);

	  VL53_SetSignalRateLimit(tof, 0.25);

	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0xFF);

	  uint8_t spad_count;
	  bool spad_type_is_aperture;
	  if (!getSpadInfo(tof, &spad_count, &spad_type_is_aperture)) { return false; }

	  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	  // the API, but the same data seems to be more easily readable from
	  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	  uint8_t ref_spad_map[6];
	  readMulti(tof, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	  writeReg(tof, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	  uint8_t spads_enabled = 0;

	  for (uint8_t i = 0; i < 48; i++)
	  {
	    if (i < first_spad_to_enable || spads_enabled == spad_count)
	    {
	      // This bit is lower than the first one that should be enabled, or
	      // (reference_spad_count) bits have already been enabled, so zero this bit
	      ref_spad_map[i / 8] &= ~(1 << (i % 8));
	    }
	    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
	    {
	      spads_enabled++;
	    }
	  }

	  writeMulti(tof, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x00, 0x00);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x09, 0x00);
	  writeReg(tof, 0x10, 0x00);
	  writeReg(tof, 0x11, 0x00);

	  writeReg(tof, 0x24, 0x01);
	  writeReg(tof, 0x25, 0xFF);
	  writeReg(tof, 0x75, 0x00);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x4E, 0x2C);
	  writeReg(tof, 0x48, 0x00);
	  writeReg(tof, 0x30, 0x20);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x30, 0x09);
	  writeReg(tof, 0x54, 0x00);
	  writeReg(tof, 0x31, 0x04);
	  writeReg(tof, 0x32, 0x03);
	  writeReg(tof, 0x40, 0x83);
	  writeReg(tof, 0x46, 0x25);
	  writeReg(tof, 0x60, 0x00);
	  writeReg(tof, 0x27, 0x00);
	  writeReg(tof, 0x50, 0x06);
	  writeReg(tof, 0x51, 0x00);
	  writeReg(tof, 0x52, 0x96);
	  writeReg(tof, 0x56, 0x08);
	  writeReg(tof, 0x57, 0x30);
	  writeReg(tof, 0x61, 0x00);
	  writeReg(tof, 0x62, 0x00);
	  writeReg(tof, 0x64, 0x00);
	  writeReg(tof, 0x65, 0x00);
	  writeReg(tof, 0x66, 0xA0);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x22, 0x32);
	  writeReg(tof, 0x47, 0x14);
	  writeReg(tof, 0x49, 0xFF);
	  writeReg(tof, 0x4A, 0x00);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x7A, 0x0A);
	  writeReg(tof, 0x7B, 0x00);
	  writeReg(tof, 0x78, 0x21);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x23, 0x34);
	  writeReg(tof, 0x42, 0x00);
	  writeReg(tof, 0x44, 0xFF);
	  writeReg(tof, 0x45, 0x26);
	  writeReg(tof, 0x46, 0x05);
	  writeReg(tof, 0x40, 0x40);
	  writeReg(tof, 0x0E, 0x06);
	  writeReg(tof, 0x20, 0x1A);
	  writeReg(tof, 0x43, 0x40);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x34, 0x03);
	  writeReg(tof, 0x35, 0x44);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x31, 0x04);
	  writeReg(tof, 0x4B, 0x09);
	  writeReg(tof, 0x4C, 0x05);
	  writeReg(tof, 0x4D, 0x04);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x44, 0x00);
	  writeReg(tof, 0x45, 0x20);
	  writeReg(tof, 0x47, 0x08);
	  writeReg(tof, 0x48, 0x28);
	  writeReg(tof, 0x67, 0x00);
	  writeReg(tof, 0x70, 0x04);
	  writeReg(tof, 0x71, 0x01);
	  writeReg(tof, 0x72, 0xFE);
	  writeReg(tof, 0x76, 0x00);
	  writeReg(tof, 0x77, 0x00);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x0D, 0x01);

	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x80, 0x01);
	  writeReg(tof, 0x01, 0xF8);

	  writeReg(tof, 0xFF, 0x01);
	  writeReg(tof, 0x8E, 0x01);
	  writeReg(tof, 0x00, 0x01);
	  writeReg(tof, 0xFF, 0x00);
	  writeReg(tof, 0x80, 0x00);

	  // -- VL53L0X_load_tuning_settings() end

	  // "Set interrupt config to new sample ready"
	  // -- VL53L0X_SetGpioConfig() begin

	  writeReg(tof, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	  writeReg(tof, GPIO_HV_MUX_ACTIVE_HIGH, readReg(tof, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	  writeReg(tof, SYSTEM_INTERRUPT_CLEAR, 0x01);

	  tof->measurement_timing_budget_us = getMeasurementTimingBudget(tof);

	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	  // "Recalculate timing budget"
	  VL53_SetMeasurementTimingBudget(tof, tof->measurement_timing_budget_us);

	  // VL53L0X_StaticInit() end

	  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	  // -- VL53L0X_perform_vhv_calibration() begin

	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0x01);
	  if (!performSingleRefCalibration(tof, 0x40)) { return false; }

	  // -- VL53L0X_perform_vhv_calibration() end

	  // -- VL53L0X_perform_phase_calibration() begin

	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0x02);
	  if (!performSingleRefCalibration(tof, 0x00)) { return false; }

	  // -- VL53L0X_perform_phase_calibration() end

	  // "restore the previous Sequence Config"
	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	  // VL53L0X_PerformRefCalibration() end
	return true;
}

void VL53_SetTimeout(VL53_t* tof, int timeout)
{
	tof->io_timeout = timeout;
}

bool VL53_SetSignalRateLimit(VL53_t* tof, float limit)
{
	  if (limit < 0 || limit > 511.99) { return false; }

	  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	  writeReg16Bit(tof, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit * (1 << 7));
	  return true;
}

bool VL53_SetVcselPulsePeriod(VL53_t* tof, vcselPeriodType type, uint8_t period_pclks)
{
	 uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

	  SequenceStepEnables enables;
	  SequenceStepTimeouts timeouts;

	  getSequenceStepEnables(tof, &enables);
	  getSequenceStepTimeouts(tof, &enables, &timeouts);

	  // "Apply specific settings for the requested clock period"
	  // "Re-calculate and apply timeouts, in macro periods"

	  // "When the VCSEL period for the pre or final range is changed,
	  // the corresponding timeout must be read from the device using
	  // the current VCSEL period, then the new VCSEL period can be
	  // applied. The timeout then must be written back to the device
	  // using the new VCSEL period.
	  //
	  // For the MSRC timeout, the same applies - this timeout being
	  // dependant on the pre-range vcsel period."


	  if (type == VcselPeriodPreRange)
	  {
	    // "Set phase check limits"
	    switch (period_pclks)
	    {
	      case 12:
	        writeReg(tof, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
	        break;

	      case 14:
	        writeReg(tof, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
	        break;

	      case 16:
	        writeReg(tof, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
	        break;

	      case 18:
	        writeReg(tof, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
	        break;

	      default:
	        // invalid period
	        return false;
	    }
	    writeReg(tof, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

	    // apply new VCSEL period
	    writeReg(tof, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

	    uint16_t new_pre_range_timeout_mclks =
	      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

	    writeReg16Bit(tof, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      encodeTimeout(new_pre_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

	    uint16_t new_msrc_timeout_mclks =
	      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

	    writeReg(tof, MSRC_CONFIG_TIMEOUT_MACROP,
	      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

	    // set_sequence_step_timeout() end
	  }
	  else if (type == VcselPeriodFinalRange)
	  {
	    switch (period_pclks)
	    {
	      case 8:
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(tof, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
	        writeReg(tof, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
	        writeReg(tof, 0xFF, 0x01);
	        writeReg(tof, ALGO_PHASECAL_LIM, 0x30);
	        writeReg(tof, 0xFF, 0x00);
	        break;

	      case 10:
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(tof, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(tof, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
	        writeReg(tof, 0xFF, 0x01);
	        writeReg(tof, ALGO_PHASECAL_LIM, 0x20);
	        writeReg(tof, 0xFF, 0x00);
	        break;

	      case 12:
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(tof, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(tof, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
	        writeReg(tof, 0xFF, 0x01);
	        writeReg(tof, ALGO_PHASECAL_LIM, 0x20);
	        writeReg(tof, 0xFF, 0x00);
	        break;

	      case 14:
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
	        writeReg(tof, FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
	        writeReg(tof, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
	        writeReg(tof, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
	        writeReg(tof, 0xFF, 0x01);
	        writeReg(tof, ALGO_PHASECAL_LIM, 0x20);
	        writeReg(tof, 0xFF, 0x00);
	        break;

	      default:
	        // invalid period
	        return false;
	    }

	    // apply new VCSEL period
	    writeReg(tof, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

	    // update timeouts

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint16_t new_final_range_timeout_mclks =
	      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

	    if (enables.pre_range)
	    {
	      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
	    }

	    writeReg16Bit(tof, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      encodeTimeout(new_final_range_timeout_mclks));

	    // set_sequence_step_timeout end
	  }
	  else
	  {
	    // invalid type
	    return false;
	  }

	  // "Finally, the timing budget must be re-applied"

	  VL53_SetMeasurementTimingBudget(tof, tof->measurement_timing_budget_us);

	  // "Perform the phase calibration. This is needed after changing on vcsel period."
	  // VL53L0X_perform_phase_calibration() begin

	  uint8_t sequence_config = readReg(tof, SYSTEM_SEQUENCE_CONFIG);
	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, 0x02);
	  performSingleRefCalibration(tof, 0x0);
	  writeReg(tof, SYSTEM_SEQUENCE_CONFIG, sequence_config);

	  // VL53L0X_perform_phase_calibration() end

	  return true;
}

bool VL53_SetMeasurementTimingBudget(VL53_t* tof, uint32_t budget_us)
{
	  SequenceStepEnables enables;
	  SequenceStepTimeouts timeouts;

	  uint16_t const StartOverhead     = 1910;
	  uint16_t const EndOverhead        = 960;
	  uint16_t const MsrcOverhead       = 660;
	  uint16_t const TccOverhead        = 590;
	  uint16_t const DssOverhead        = 690;
	  uint16_t const PreRangeOverhead   = 660;
	  uint16_t const FinalRangeOverhead = 550;

	  uint32_t const MinTimingBudget = 20000;

	  if (budget_us < MinTimingBudget) { return false; }

	  uint32_t used_budget_us = StartOverhead + EndOverhead;

	  getSequenceStepEnables(tof, &enables);
	  getSequenceStepTimeouts(tof, &enables, &timeouts);

	  if (enables.tcc)
	  {
	    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
	  }

	  if (enables.dss)
	  {
	    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
	  }
	  else if (enables.msrc)
	  {
	    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
	  }

	  if (enables.pre_range)
	  {
	    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
	  }

	  if (enables.final_range)
	  {
	    used_budget_us += FinalRangeOverhead;

	    // "Note that the final range timeout is determined by the timing
	    // budget and the sum of all other timeouts within the sequence.
	    // If there is no room for the final range timeout, then an error
	    // will be set. Otherwise the remaining time will be applied to
	    // the final range."

	    if (used_budget_us > budget_us)
	    {
	      // "Requested timeout too big."
	      return false;
	    }

	    uint32_t final_range_timeout_us = budget_us - used_budget_us;

	    // set_sequence_step_timeout() begin
	    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

	    // "For the final range timeout, the pre-range timeout
	    //  must be added. To do this both final and pre-range
	    //  timeouts must be expressed in macro periods MClks
	    //  because they have different vcsel periods."

	    uint32_t final_range_timeout_mclks =
	      timeoutMicrosecondsToMclks(final_range_timeout_us,
	                                 timeouts.final_range_vcsel_period_pclks);

	    if (enables.pre_range)
	    {
	      final_range_timeout_mclks += timeouts.pre_range_mclks;
	    }

	    writeReg16Bit(tof, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
	      encodeTimeout(final_range_timeout_mclks));

	    // set_sequence_step_timeout() end

	    tof->measurement_timing_budget_us = budget_us; // store for internal reuse
	  }
	  return true;
}

bool VL53_TimeoutOccured(VL53_t* tof)
{
	  bool tmp = tof->did_timeout;
	  tof->did_timeout = false;
	  return tmp;
}

uint16_t VL53_ReadRangeSingle_mm(VL53_t* tof)
{
  writeReg(tof, 0x80, 0x01);
  writeReg(tof, 0xFF, 0x01);
  writeReg(tof, 0x00, 0x00);
  writeReg(tof, 0x91, tof->stopVariable);
  writeReg(tof, 0x00, 0x01);
  writeReg(tof, 0xFF, 0x00);
  writeReg(tof, 0x80, 0x00);

  writeReg(tof, SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout(tof);
  while (readReg(tof, SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired(tof))
    {
      tof->did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters(tof);
}
