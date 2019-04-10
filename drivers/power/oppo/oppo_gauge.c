/**********************************************************************************
* Copyright (c)  2008-2015  Guangdong OPPO Mobile Comm Corp., Ltd
* VENDOR_EDIT
* Description: Charger IC management module for charger system framework.
*              Manage all charger IC and define abstarct function flow.
* Version    : 1.0
* Date       : 2015-06-22
* Author     : fanhui@PhoneSW.BSP
*            : Fanhong.Kong@ProDrv.CHG
* ------------------------------ Revision History: --------------------------------
* <version>           <date>                <author>                          <desc>
* Revision 1.0        2015-06-22       fanhui@PhoneSW.BSP            Created for new architecture
* Revision 1.0        2015-06-22       Fanhong.Kong@ProDrv.CHG       Created for new architecture
***********************************************************************************/

#include "oppo_gauge.h"

static struct oppo_gauge_chip *g_gauge_chip = NULL;

int oppo_gauge_get_batt_mvolts(void)
{
        if (!g_gauge_chip) {
                return 3800;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_mvolts();
				if (g_gauge_chip->gauge_ops->get_battery_mvolts && g_gauge_chip->gauge_ops != NULL) {
					return g_gauge_chip->gauge_ops->get_battery_mvolts();
				} else {
					return 3800;
				}
        }
}

int oppo_gauge_get_batt_mvolts_2cell_max(void)
{
	if(!g_gauge_chip) {
		return 3800;
	} else {
		//return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_max();
		if (g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_max && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_max();
		} else {
			return 3800;
		}
	}
}

int oppo_gauge_get_batt_mvolts_2cell_min(void)
{
	if(!g_gauge_chip) {
		return 3800;
	} else {
		//return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_min();
		if (g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_min && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_mvolts_2cell_min();
		} else {
			return 3800;
		}
	}
}

int oppo_gauge_get_batt_temperature(void)
{
        if (!g_gauge_chip) {
                return 250;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_temperature();
				if (g_gauge_chip->gauge_ops->get_battery_temperature && g_gauge_chip->gauge_ops != NULL) {
					return g_gauge_chip->gauge_ops->get_battery_temperature();
				} else {
					return 250;
				}
        }
}

int oppo_gauge_get_batt_soc(void)
{
        if (!g_gauge_chip) {
                return 50;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_soc();
		if (g_gauge_chip->gauge_ops->get_battery_soc && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_soc();
		} else {
			return 50;
		}
        }
}

int oppo_gauge_get_batt_current(void)
{
        if (!g_gauge_chip) {
                return 100;
        } else {
                //return g_gauge_chip->gauge_ops->get_average_current();
			if (g_gauge_chip->gauge_ops->get_average_current && g_gauge_chip->gauge_ops != NULL) {
				return g_gauge_chip->gauge_ops->get_average_current();
			} else {
				return 100;
			}
        }
}

int oppo_gauge_get_remaining_capacity(void)
{
        if (!g_gauge_chip) {
                return 0;
        } else {
                //return g_gauge_chip->gauge_ops->get_batt_remaining_capacity();
		if (g_gauge_chip->gauge_ops->get_batt_remaining_capacity && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_batt_remaining_capacity();
		} else {
			return 0;
		}
        }
}

int oppo_gauge_get_device_type(void)
{
        if (!g_gauge_chip) {
                return 0;
        } else {
                return g_gauge_chip->device_type;
        }
}

int oppo_gauge_get_batt_fcc(void)
{
        if (!g_gauge_chip) {
                return 0;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_fcc();
		if (g_gauge_chip->gauge_ops->get_battery_fcc && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_fcc();
		} else {
			return 0;
		}
        }
}

int oppo_gauge_get_batt_cc(void)
{
        if (!g_gauge_chip) {
                return 0;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_cc();
		if (g_gauge_chip->gauge_ops->get_battery_cc && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_cc();
		} else {
			return 0;
		}
        }
}

int oppo_gauge_get_batt_soh(void)
{
        if (!g_gauge_chip) {
                return 0;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_soh();
		if (g_gauge_chip->gauge_ops->get_battery_soh && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_battery_soh();
		} else {
			return 0;
		}
        }
}

bool oppo_gauge_get_batt_authenticate(void)
{
        if (!g_gauge_chip) {
                return false;
        } else {
                //return g_gauge_chip->gauge_ops->get_battery_authenticate();
				if (g_gauge_chip->gauge_ops->get_battery_authenticate && g_gauge_chip->gauge_ops != NULL) {
					return g_gauge_chip->gauge_ops->get_battery_authenticate();
				} else {
					return false;
				}
        }
}

void oppo_gauge_set_batt_full(bool full)
{
        if (g_gauge_chip) {
                //g_gauge_chip->gauge_ops->set_battery_full(full);
				if (g_gauge_chip->gauge_ops->set_battery_full && g_gauge_chip->gauge_ops != NULL) {
		                	g_gauge_chip->gauge_ops->set_battery_full(full);
				}
        }
}

bool oppo_gauge_check_chip_is_null(void)
{
        if (!g_gauge_chip) {
                return true;
        } else {
                return false;
        }
}

void oppo_gauge_init(struct oppo_gauge_chip *chip)
{
        g_gauge_chip = chip;
}

int oppo_gauge_get_prev_batt_mvolts(void)
{
	if (!g_gauge_chip)
		return 3800;
	else {
		//return g_gauge_chip->gauge_ops->get_prev_battery_mvolts();
		if (g_gauge_chip->gauge_ops->get_prev_battery_mvolts && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_battery_mvolts();
		} else {
			return 3800;
		}
	}
}

int oppo_gauge_get_prev_batt_mvolts_2cell_max(void)
{
	if(!g_gauge_chip) {
		return 3800;
	} else {
		//return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_max();
		if (g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_max && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_max();
		} else {
			return 3800;
		}
	}
}

int oppo_gauge_get_prev_batt_mvolts_2cell_min(void)
{
	if(!g_gauge_chip) {
		return 3800;
	} else {
		//return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_min();
		if (g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_min && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_battery_mvolts_2cell_min();
		} else {
			return 3800;
		}
	}
}

int oppo_gauge_get_prev_batt_temperature(void)
{
	if (!g_gauge_chip) {
		return 250;
	} else {
		//return g_gauge_chip->gauge_ops->get_prev_battery_temperature();
		if (g_gauge_chip->gauge_ops->get_prev_battery_temperature && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_battery_temperature();
		} else {
			return 250;
		}
	}
}

int oppo_gauge_get_prev_batt_soc(void)
{
	if (!g_gauge_chip) {
		return 50;
	} else {
		//return g_gauge_chip->gauge_ops->get_prev_battery_soc();
		if (g_gauge_chip->gauge_ops->get_prev_battery_soc && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_battery_soc();
		} else {
			return 50;
		}
	}
}

int oppo_gauge_get_prev_batt_current(void)
{
	if (!g_gauge_chip) {
		return 100;
	} else {
		//return g_gauge_chip->gauge_ops->get_prev_average_current();
		if (g_gauge_chip->gauge_ops->get_prev_average_current && g_gauge_chip->gauge_ops != NULL) {
			return g_gauge_chip->gauge_ops->get_prev_average_current();
		} else {
			return 100;
		}
	}
}

int oppo_gauge_update_battery_dod0(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		//return g_gauge_chip->gauge_ops->update_battery_dod0();
		if (g_gauge_chip->gauge_ops->update_battery_dod0 && g_gauge_chip->gauge_ops != NULL) {
            return g_gauge_chip->gauge_ops->update_battery_dod0();
		} else {
			return 100;
		}
}


int oppo_gauge_update_soc_smooth_parameter(void)
{
	if (!g_gauge_chip)
		return 0;
	else
		//return g_gauge_chip->gauge_ops->update_soc_smooth_parameter();
		if (g_gauge_chip->gauge_ops->update_soc_smooth_parameter && g_gauge_chip->gauge_ops != NULL) {
            return g_gauge_chip->gauge_ops->update_soc_smooth_parameter();
		} else {
			return 0;
		}
}
