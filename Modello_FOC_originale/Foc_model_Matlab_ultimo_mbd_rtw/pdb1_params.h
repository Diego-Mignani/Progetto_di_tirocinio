#ifndef PDB1_PARAMS_H
#define PDB1_PARAMS_H
#include "pdb_driver.h"
#include "trgmux_driver.h"

extern trgmux_inout_mapping_config_t trgmuxAllMappingConfig[];
extern trgmux_inout_mapping_config_t pdb1MappingConfig;
extern pdb_timer_config_t pdb1TimerConfig;
extern pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig0U;
extern pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig1U;
extern pdb_adc_pretrigger_config_t pdb1Ch0UPreTrigConfig2U;

#endif
