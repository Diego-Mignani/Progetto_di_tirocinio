#include "ftm2_input_params_config.h"

ftm_input_ch_param_t ftm2input_ch_param[1U] = {
  {
    1U,                                /* Channel id */
    FTM_EDGE_DETECT,                   /* Input capture operation mode */
    FTM_BOTH_EDGES,                    /* Edge alignment mode */
    FTM_NO_MEASUREMENT,                /* Signal measurement operation type */
    0,                                 /* Filter value */
    false,                             /* Filter state (enabled/disabled) */
    false,                             /* Continuous measurement state */
    NULL,              /* Vector of callbacks  parameters for channels events */
    NULL                           /* Vector of callbacks for channels events */
  },
};

/* Input capture configuration for FTM2 */
ftm_input_param_t flexTimer_ic2_InputCaptureConfig = {
  1U,                                  /* Number of channels */
  65535U,                              /* Max count value */
  ftm2input_ch_param                   /* Channels configuration */
};

/* Global configuration of flexTimer_ic1 */
ftm_user_config_t flexTimer_ic2_InitConfig = {
  {
    true,                              /* Software trigger state */
    false,                             /* Hardware trigger 1 state */
    false,                             /* Hardware trigger 2 state */
    false,                             /* Hardware trigger 3 state */
    false,                             /* Max loading point state */
    false,                             /* Min loading point state */
    FTM_SYSTEM_CLOCK,                  /* Update mode for INVCTRL register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for SWOCTRL register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for OUTMASK register */
    FTM_SYSTEM_CLOCK,                  /* Update mode for CNTIN register */
    false,                             /* Automatic clear of the trigger*/
    FTM_UPDATE_NOW,                    /* Synchronization point */
  },
  FTM_MODE_INPUT_CAPTURE,              /* Mode of operation for FTM */
  FTM_CLOCK_DIVID_BY_128,              /* FTM clock prescaler */
  FTM_CLOCK_SOURCE_SYSTEMCLK,          /* FTM clock source */
  FTM_BDM_MODE_00,                     /* FTM debug mode */
  false,                               /* Interrupt state */
  false                                /* Initialization trigger */
};
