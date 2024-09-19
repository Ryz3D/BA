str_file = "FATFS/Target/bsp_driver_sd.c"

str_find = """
  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd1);
  /* Configure SD Bus width (4 bits mode selected) */
  if (sd_state == MSD_OK)
  {
    /* Enable wide operation */
    if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
    {
      sd_state = MSD_ERROR;
    }
  }
"""

str_replace = """
  /* HAL SD initialization */
  sd_state = HAL_SD_Init(&hsd1);
  /* Configure SD Bus width (4 bits mode selected) */
  if (sd_state == MSD_OK)
  {
    /* Enable wide operation */
    HAL_StatusTypeDef status;
    SD_InitTypeDef Init;

    /* Default SDMMC peripheral configuration for SD card initialization */
    Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
    Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    Init.BusWide = SDMMC_BUS_WIDE_1B;
    Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    Init.ClockDiv = SDMMC_INIT_CLK_DIV;

    /* Initialize SDMMC peripheral interface with default configuration */
    status = SDMMC_Init(hsd1.Instance, Init);
    if (status != HAL_OK)
    {
      sd_state = MSD_ERROR;
    }
    else
    {
      if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
      {
        sd_state = MSD_ERROR;
      }
    }
  }
"""

with open(str_file) as f:
  data = f.read()

if data.find(str_find) != -1:
  print("fixed")
else:
  print("nothing to fix")

data = data.replace(str_find, str_replace)

with open(str_file, 'w') as f:
  f.write(data)
