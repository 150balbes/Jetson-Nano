/*
 * Copyright (c) 2011,2013-2014 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/*
 * Notifications and licenses are retained for attribution purposes only.
 */
/*
 * Copyright (c) 2002-2006 Sam Leffler, Errno Consulting
 * Copyright (c) 2005-2006 Atheros Communications, Inc.
 * Copyright (c) 2010, Atheros Communications Inc.
 *
 * Redistribution and use in source and binary forms are permitted
 * provided that the following conditions are met:
 * 1. The materials contained herein are unmodified and are used
 *    unmodified.
 * 2. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following NO
 *    ''WARRANTY'' disclaimer below (''Disclaimer''), without
 *    modification.
 * 3. Redistributions in binary form must reproduce at minimum a
 *    disclaimer similar to the Disclaimer below and any redistribution
 *    must be conditioned upon including a substantially similar
 *    Disclaimer requirement for further binary redistribution.
 * 4. Neither the names of the above-listed copyright holders nor the
 *    names of any contributors may be used to endorse or promote
 *    product derived from this software without specific prior written
 *    permission.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF NONINFRINGEMENT,
 * MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE
 * FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGES.
 */

#include <adf_os_types.h>
#include "wma.h"
#include "regdomain.h"
#include "regdomain_common.h"

#define N(a) (sizeof(a)/sizeof(a[0]))

/*
 * By default, the regdomain tables reference the common tables
 * from regdomain_common.h.  These default tables can be replaced
 * by calls to populate_regdomain_tables functions.
 */
HAL_REG_DMN_TABLES ol_regdmn_Rdt = {
	ahCmnRegDomainPairs,    /* regDomainPairs */
	ahCmnAllCountries,      /* allCountries */
	ahCmnRegDomains,        /* allRegDomains */
	N(ahCmnRegDomainPairs),    /* regDomainPairsCt */
	N(ahCmnAllCountries),      /* allCountriesCt */
	N(ahCmnRegDomains),        /* allRegDomainCt */
};

static u_int16_t get_eeprom_rd(u_int16_t rd)
{
	return rd & ~WORLDWIDE_ROAMING_FLAG;
}

/*
 * Return whether or not the regulatory domain/country in EEPROM
 * is acceptable.
 */
static bool regdmn_is_eeprom_valid(u_int16_t rd)
{
	int32_t i;

	if (rd & COUNTRY_ERD_FLAG) {
		u_int16_t cc = rd & ~COUNTRY_ERD_FLAG;
		for (i = 0; i < ol_regdmn_Rdt.allCountriesCt; i++)
			if (ol_regdmn_Rdt.allCountries[i].countryCode == cc)
				return true;
	} else {
		for (i = 0; i < ol_regdmn_Rdt.regDomainPairsCt; i++)
			if (ol_regdmn_Rdt.regDomainPairs[i].regDmnEnum == rd)
				return true;
	}
	/* TODO: Bring it under debug level */
	adf_os_print("%s: invalid regulatory domain/country code 0x%x\n",
		     __func__, rd);
	return false;
}

/*
 * Find the pointer to the country element in the country table
 * corresponding to the country code
 */
static const COUNTRY_CODE_TO_ENUM_RD *find_country(u_int16_t country_code)
{
	int32_t i;

	for (i = 0; i < ol_regdmn_Rdt.allCountriesCt; i++) {
		if (ol_regdmn_Rdt.allCountries[i].countryCode == country_code)
			return &ol_regdmn_Rdt.allCountries[i];
	}
	return NULL;        /* Not found */
}

int32_t regdmn_find_ctry_by_name(u_int8_t *alpha2)
{
	int32_t i;

	for (i = 0; i < ol_regdmn_Rdt.allCountriesCt; i++) {
		if (ol_regdmn_Rdt.allCountries[i].isoName[0] == alpha2[0] &&
			 ol_regdmn_Rdt.allCountries[i].isoName[1] == alpha2[1])
			return ol_regdmn_Rdt.allCountries[i].countryCode;
	}
	return CTRY_DEFAULT;        /* Not found */
}

static u_int16_t regdmn_get_default_country(u_int16_t rd)
{
	int32_t i;

	if (rd & COUNTRY_ERD_FLAG) {
		const COUNTRY_CODE_TO_ENUM_RD *country = NULL;
		u_int16_t cc = rd & ~COUNTRY_ERD_FLAG;

		country = find_country(cc);
		if (country)
			return cc;
	}

	/*
	 * Check reg domains that have only one country
	 */
	for (i = 0; i < ol_regdmn_Rdt.regDomainPairsCt; i++) {
		if (ol_regdmn_Rdt.regDomainPairs[i].regDmnEnum == rd) {
			if (ol_regdmn_Rdt.regDomainPairs[i].singleCC != 0)
				return ol_regdmn_Rdt.regDomainPairs[i].singleCC;
			else
				i = ol_regdmn_Rdt.regDomainPairsCt;
		}
	}
	return CTRY_DEFAULT;
}

static const REG_DMN_PAIR_MAPPING *get_regdmn_pair(u_int16_t reg_dmn)
{
	int32_t i;

	for (i = 0; i < ol_regdmn_Rdt.regDomainPairsCt; i++) {
		if (ol_regdmn_Rdt.regDomainPairs[i].regDmnEnum == reg_dmn)
			return &ol_regdmn_Rdt.regDomainPairs[i];
	}
	return NULL;
}

static const REG_DOMAIN *get_regdmn(u_int16_t reg_dmn)
{
	int32_t i;

	for (i = 0; i < ol_regdmn_Rdt.regDomainsCt; i++) {
		if (ol_regdmn_Rdt.regDomains[i].regDmnEnum == reg_dmn)
			return &ol_regdmn_Rdt.regDomains[i];
	}
	return NULL;
}

static const COUNTRY_CODE_TO_ENUM_RD *get_country_from_rd(u_int16_t regdmn)
{
	int32_t i;

	for (i = 0; i < ol_regdmn_Rdt.allCountriesCt; i++) {
		if (ol_regdmn_Rdt.allCountries[i].regDmnEnum == regdmn)
			return &ol_regdmn_Rdt.allCountries[i];
	}
	return NULL;        /* Not found */
}

/*
 * Some users have reported their EEPROM programmed with
 * 0x8000 set, this is not a supported regulatory domain
 * but since we have more than one user with it we need
 * a solution for them. We default to 0x64
 */
static void regd_sanitize(struct regulatory *reg)
{
	if (reg->reg_domain != COUNTRY_ERD_FLAG)
		return;
	reg->reg_domain = 0x64;
}

/*
 * Returns country string for the given regulatory domain.
 */
int32_t regdmn_get_country_alpha2(struct regulatory *reg)
{
	u_int16_t country_code;
	u_int16_t regdmn, rd;
	const COUNTRY_CODE_TO_ENUM_RD *country = NULL;

	regd_sanitize(reg);
	rd = reg->reg_domain;

	if (!regdmn_is_eeprom_valid(rd))
		return -EINVAL;

	regdmn = get_eeprom_rd(rd);

	country_code = regdmn_get_default_country(regdmn);
	if (country_code == CTRY_DEFAULT && regdmn == CTRY_DEFAULT) {
		/* Set to CTRY_UNITED_STATES for testing */
		country_code = CTRY_UNITED_STATES;
	}

	if (country_code != CTRY_DEFAULT) {
		country = find_country(country_code);
		if (!country) {
			/* TODO: Bring it under debug level */
			adf_os_print(KERN_ERR "Not a valid country code\n");
			return -EINVAL;
		}
		regdmn = country->regDmnEnum;
	}

	reg->regpair = get_regdmn_pair(regdmn);
	if (!reg->regpair) {
		/* TODO: Bring it under debug level */
		adf_os_print(KERN_ERR "No regpair is found, can not proceeed\n");
		return -EINVAL;
	}
	reg->country_code = country_code;

	if (!country)
		country = get_country_from_rd(regdmn);

	if (country) {
		reg->alpha2[0] = country->isoName[0];
		reg->alpha2[1] = country->isoName[1];
	} else {
		reg->alpha2[0] = '0';
		reg->alpha2[1] = '0';
	}

	return 0;
}

/*
 * Returns regulatory domain for given country string
 */
int32_t regdmn_get_regdmn_for_country(u_int8_t *alpha2)
{
	u_int8_t i;

	for (i = 0; i < ol_regdmn_Rdt.allCountriesCt; i++) {
		if ((ol_regdmn_Rdt.allCountries[i].isoName[0] == alpha2[0]) &&
		    (ol_regdmn_Rdt.allCountries[i].isoName[1] == alpha2[1]))
			return ol_regdmn_Rdt.allCountries[i].regDmnEnum;
	}
	return -1;
}

/*
 * Test to see if the bitmask array is all zeros
 */
static bool
isChanBitMaskZero(const u_int64_t *bitmask)
{
	int i;

	for (i = 0; i < BMLEN; i++) {
		if (bitmask[i] != 0)
			return false;
	}
	return true;
}

/*
 * Return the mask of available modes based on the hardware
 * capabilities and the specified country code and reg domain.
 */
u_int32_t regdmn_getwmodesnreg(u_int32_t modesAvail,
		const COUNTRY_CODE_TO_ENUM_RD *country,
		const REG_DOMAIN *rd5GHz)
{

	/* Check country regulations for allowed modes */
	if ((modesAvail & (REGDMN_MODE_11A_TURBO|REGDMN_MODE_TURBO)) &&
			(!country->allow11aTurbo))
		modesAvail &= ~(REGDMN_MODE_11A_TURBO | REGDMN_MODE_TURBO);

	if ((modesAvail & REGDMN_MODE_11G_TURBO) &&
			(!country->allow11gTurbo))
		modesAvail &= ~REGDMN_MODE_11G_TURBO;

	if ((modesAvail & REGDMN_MODE_11G) &&
			(!country->allow11g))
		modesAvail &= ~REGDMN_MODE_11G;

	if ((modesAvail & REGDMN_MODE_11A) &&
			(isChanBitMaskZero(rd5GHz->chan11a)))
		modesAvail &= ~REGDMN_MODE_11A;

	if ((modesAvail & REGDMN_MODE_11NG_HT20) &&
			(!country->allow11ng20))
		modesAvail &= ~REGDMN_MODE_11NG_HT20;

	if ((modesAvail & REGDMN_MODE_11NA_HT20) &&
			(!country->allow11na20))
		modesAvail &= ~REGDMN_MODE_11NA_HT20;

	if ((modesAvail & REGDMN_MODE_11NG_HT40PLUS) &&
			(!country->allow11ng40))
		modesAvail &= ~REGDMN_MODE_11NG_HT40PLUS;

	if ((modesAvail & REGDMN_MODE_11NG_HT40MINUS) &&
			(!country->allow11ng40))
		modesAvail &= ~REGDMN_MODE_11NG_HT40MINUS;

	if ((modesAvail & REGDMN_MODE_11NA_HT40PLUS) &&
			(!country->allow11na40))
		modesAvail &= ~REGDMN_MODE_11NA_HT40PLUS;

	if ((modesAvail & REGDMN_MODE_11NA_HT40MINUS) &&
			(!country->allow11na40))
		modesAvail &= ~REGDMN_MODE_11NA_HT40MINUS;

	if ((modesAvail & REGDMN_MODE_11AC_VHT20) &&
			(!country->allow11na20))
		modesAvail &= ~REGDMN_MODE_11AC_VHT20;

	if ((modesAvail & REGDMN_MODE_11AC_VHT40PLUS) &&
			(!country->allow11na40))
		modesAvail &= ~REGDMN_MODE_11AC_VHT40PLUS;

	if ((modesAvail & REGDMN_MODE_11AC_VHT40MINUS) &&
			(!country->allow11na40))
		modesAvail &= ~REGDMN_MODE_11AC_VHT40MINUS;

	if ((modesAvail & REGDMN_MODE_11AC_VHT80) &&
			(!country->allow11na80))
		modesAvail &= ~REGDMN_MODE_11AC_VHT80;

	if ((modesAvail & REGDMN_MODE_11AC_VHT20_2G) &&
			(!country->allow11ng20))
		modesAvail &= ~REGDMN_MODE_11AC_VHT20_2G;

	return modesAvail;
}

void regdmn_get_ctl_info(struct regulatory *reg, u_int32_t modesAvail,
     u_int32_t modeSelect)
{
	const REG_DOMAIN *regdomain2G = NULL;
	const REG_DOMAIN *regdomain5G = NULL;
	int8_t ctl_2g, ctl_5g, ctl;
	const REG_DOMAIN *rd = NULL;
	const struct cmode *cm;
	const COUNTRY_CODE_TO_ENUM_RD *country;
	const REG_DMN_PAIR_MAPPING *regpair;

	regpair = reg->regpair;
	regdomain2G = get_regdmn(regpair->regDmn2GHz);
	if (!regdomain2G) {
		adf_os_print(KERN_ERR "Failed to get regdmn 2G");
		return;
	}

	regdomain5G = get_regdmn(regpair->regDmn5GHz);
	if (!regdomain5G) {
		adf_os_print(KERN_ERR "Failed to get regdmn 5G");
		return;
	}

	/* find first nible of CTL */
	ctl_2g = regdomain2G->conformance_test_limit;
	ctl_5g = regdomain5G->conformance_test_limit;

	/* find second nible of CTL */
	country = find_country(reg->country_code);
	if (country != NULL)
		modesAvail = regdmn_getwmodesnreg(modesAvail, country, regdomain5G);

	for (cm = modes; cm < &modes[N(modes)]; cm++) {

		if ((cm->mode & modeSelect) == 0)
			continue;

		if ((cm->mode & modesAvail) == 0)
			continue;

		switch (cm->mode) {
		case REGDMN_MODE_TURBO:
			rd = regdomain5G;
			ctl = rd->conformance_test_limit | CTL_TURBO;
			break;
		case REGDMN_MODE_11A:
		case REGDMN_MODE_11NA_HT20:
		case REGDMN_MODE_11NA_HT40PLUS:
		case REGDMN_MODE_11NA_HT40MINUS:
		case REGDMN_MODE_11AC_VHT20:
		case REGDMN_MODE_11AC_VHT40PLUS:
		case REGDMN_MODE_11AC_VHT40MINUS:
		case REGDMN_MODE_11AC_VHT80:
			rd = regdomain5G;
			ctl = rd->conformance_test_limit;
			break;
		case REGDMN_MODE_11B:
			rd = regdomain2G;
			ctl = rd->conformance_test_limit | CTL_11B;
			break;
		case REGDMN_MODE_11G:
		case REGDMN_MODE_11NG_HT20:
		case REGDMN_MODE_11NG_HT40PLUS:
		case REGDMN_MODE_11NG_HT40MINUS:
		case REGDMN_MODE_11AC_VHT20_2G:
		case REGDMN_MODE_11AC_VHT40_2G:
		case REGDMN_MODE_11AC_VHT80_2G:
			rd = regdomain2G;
			ctl = rd->conformance_test_limit | CTL_11G;
			break;
		case REGDMN_MODE_11G_TURBO:
			rd = regdomain2G;
			ctl = rd->conformance_test_limit | CTL_108G;
			break;
		case REGDMN_MODE_11A_TURBO:
			rd = regdomain5G;
			ctl = rd->conformance_test_limit | CTL_108G;
			break;
		default:
			adf_os_print(KERN_ERR "%s: Unkonwn HAL mode 0x%x\n",
					__func__, cm->mode);
			continue;
		}

		if (rd == regdomain2G)
			ctl_2g = ctl;

		if (rd == regdomain5G)
			ctl_5g = ctl;
	}
	wma_send_regdomain_info(reg->reg_domain, regpair->regDmn2GHz,
			regpair->regDmn5GHz, ctl_2g, ctl_5g);
}

void regdmn_set_regval(struct regulatory *reg)
{
	void *vos_context = vos_get_global_context(VOS_MODULE_ID_WDA, NULL);
	tp_wma_handle wma = vos_get_context(VOS_MODULE_ID_WDA, vos_context);
	u_int32_t modeSelect = 0xFFFFFFFF;

	if (!wma) {
		WMA_LOGE("%s: Unable to get WMA handle", __func__);
		return;
	}

	wma_get_modeselect(wma, &modeSelect);

	regdmn_get_ctl_info(reg, wma->reg_cap.wireless_modes, modeSelect);
	return;
}

/* get the ctl from regdomain */
u_int8_t regdmn_get_ctl_for_regdmn(u_int32_t reg_dmn)
{
   u_int8_t i;
   u_int8_t default_regdmn_ctl = FCC;

   if (reg_dmn == CTRY_DEFAULT)
   {
      return default_regdmn_ctl;
   }
   else
   {
      for (i = 0; i < ol_regdmn_Rdt.regDomainsCt; i++)
      {
         if (ol_regdmn_Rdt.regDomains[i].regDmnEnum == reg_dmn)
            return ol_regdmn_Rdt.regDomains[i].conformance_test_limit;
      }
   }
   return -1;
}

/*
 * Get the 5G reg domain value for reg doamin
 */
u_int16_t get_regdmn_5g(u_int32_t reg_dmn)
{
	u_int16_t i;

	for (i = 0; i < ol_regdmn_Rdt.regDomainPairsCt; i++)
	{
		if (ol_regdmn_Rdt.regDomainPairs[i].regDmnEnum == reg_dmn)
		{
			return ol_regdmn_Rdt.regDomainPairs[i].regDmn5GHz;
		}
	}
	adf_os_print("%s: invalid regulatory domain/country code 0x%x\n",
		     __func__, reg_dmn);
	return 0;
}
