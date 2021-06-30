/*
 * Copyright (c) 2015-16, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __TEGRA_MC_SID_H
#define __TEGRA_MC_SID_H

#define SCEW_STREAMID_WRITE_ACCESS_DISABLED	BIT(16)
#define SCEW_STREAMID_OVERRIDE			BIT(8)
#define SCEW_NS					BIT(0)

#define MC_SMMU_BYPASS_CONFIG_0			0x1820
#define TBU_BYPASS_SID				2

#define DEFREG(__name, __offset)		\
	[__name] = {				\
		.name = __stringify(__name),	\
		.offs = __offset,		\
	}

enum mc_overrides {
	DONTCARE,
	OVERRIDE,
	NO_OVERRIDE,

	/* Enable override in linsim; Keep override disabled elsewhere. */
	SIM_OVERRIDE,
};

struct sid_override_reg {
	const char *name;
	int offs;
};

#define MAX_OIDS_IN_SID 8

struct sid_to_oids {
	int sid;			/* StreamID */
	int noids;			/* # of override IDs */
	int oid[MAX_OIDS_IN_SID];	/* Override IDs */
	enum mc_overrides ord;		/* MC or Device overrides SID? */
	const char *name;		/* Name associated with the SID. */
};

struct tegra_mc_sid_soc_data {
	struct sid_override_reg *sid_override_reg;
	unsigned int nsid_override_reg;
	struct sid_to_oids *sid_to_oids;
	unsigned int nsid_to_oids;
	unsigned int max_oids;
};

int tegra_mc_sid_probe(struct platform_device *pdev,
			const struct tegra_mc_sid_soc_data *soc_data);
int tegra_mc_sid_remove(struct platform_device *pdev);

u32 tegra_mc_get_smmu_bypass_sid(void);
const char *tegra_mc_get_sid_name(int sid);

#endif
