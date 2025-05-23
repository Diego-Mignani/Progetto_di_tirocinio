#include "pdb1_params.h"

trgmux_inout_mapping_config_t pdb1MappingConfig = {
  .triggerSource = TRGMUX_TRIG_SOURCE_FTM3_INIT_TRIG,
  .targetModule = TRGMUX_TARGET_MODULE_PDB1_TRG_IN,
  .lockTargetModuleReg = false
};

pdb_timer_config_t pdb1TimerConfig = {
  .loadValueMode = PDB_LOAD_VAL_IMMEDIATELY,
  .seqErrIntEnable = false,
  .clkPreDiv = PDB_CLK_PREDIV_BY_1,
  .clkPreMultFactor = PDB_CLK_PREMULT_FACT_AS_1,
  .triggerInput = PDB_TRIGGER_IN0,
  .continuousModeEnable = false,
  .dmaEnable = false,
  .intEnable = false
};

pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig0U = {
  .adcPreTriggerIdx = 0U,
  .preTriggerEnable = true,
  .preTriggerOutputEnable = true,
  .preTriggerBackToBackEnable = false,
};

pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig1U = {
  .adcPreTriggerIdx = 1U,
  .preTriggerEnable = true,
  .preTriggerOutputEnable = true,
  .preTriggerBackToBackEnable = false,
};

pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig2U = {
  .adcPreTriggerIdx = 2U,
  .preTriggerEnable = true,
  .preTriggerOutputEnable = true,
  .preTriggerBackToBackEnable = false,
};
