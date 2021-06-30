/*
 * sensor_dt_test - sensor device tree test
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __SENSOR_DT_TEST_H__
#define __SENSOR_DT_TEST_H__

#include <linux/string.h>
#include <linux/types.h>

/**
 * TVCF versions to be tested against.
 *
 * The semantics of the test versioning is such that:
 *   + Any version, V, where V is of MAJOR.MINOR.PATCH, will be
 *     matched against floor(V) where floor(V) is the greatest
 *     TVCF version that is less than OR equal to V.
 *
 *     For example, given some input version V and sensor dt test
 *     names support for versions 1.0.0, 2.0.0, and 2.5.0:
 *        -> floor(V = 1.0.0) = 1.0.0
 *        -> floor(V = 1.8.2) = 1.0.0
 *        -> floor(V = 2.0.5) = 2.0.0
 *        -> floor(V = 3.1.9) = 2.5.0
 *
 *     Given this, the sensor DT test can support any number of
 *     TVCF versions WITHOUT modifications granted there are no
 *     DT modifications between versions. Said another way, only
 *     versions incompatible with other versions need explicitly
 *     defined.
 *
 * To add support for a TVCF version that requires specific DT
 * checks:
 *     1) Add version to `enum sv_dt_link_version`
 *     2) Add version to `struct sv_dt_tvcf_version`
 *     3) Add version to `MAKE_LINK_ALL_FULL` macro
 *     4) Update properties accordingly
 */
enum sv_dt_link_version {
	TVCF_VERSION_V1_0_0 = 0,
	TVCF_VERSION_V2_0_0,
	TVCF_NVERSIONS,
};
#define TVCF_MIN_VERSION     (TVCF_VERSION_V1_0_0)
#define TVCF_VERSION_INVALID (-1)

/**
 * Allows a DT property to be tested only with a specific mode type.
 */
#define MODE_TYPE_ANY     (0U)
#define MODE_TYPE_BAYER   (1 << 0U)
#define MODE_TYPE_WDR_DOL (1 << 1U)
#define MODE_TYPE_WDR_PWL (1 << 2U)
#define MODE_TYPE_MAX     (1 << 31U)

/**
 * Relations between a source property X, and a sink property Y.
 *
 * Only ONE relation may exist for each property pair for each
 * supported TVCF version. Any property can form any number of
 * unique property pair relations for each supported TVCF version.
 * Relations are uni-directional from X to Y in all cases.
 *
 * Unless explicitly stated any relation between X and Y is
 * LTYPE_FORBIDDEN. For example, if a LTYPE_REQUIRED relations exists
 * for X and Y for TVCF version 2.5.0 any other relation between
 * X and Y for TVCF != 2.5.0 is LTYPE_FORBIDDEN.
 *
 * LTYPE_FORBIDDEN:       The presence of X forbids the presence of Y.
 *                        The converse IS NOT true - if X is not present
 *                        Y can assume any state.
 * LTYPE_OPTIONAL:        The presence of X makes Y optional.
 * LTYPE_REQUIRED:        The presence of X requires the presence of Y.
 *                        This type has special meaning when X has a
 *                        LTYPE_ALTERNATIVE* relation to some property Z.
 *                        If X does not exist but Z does any LTYPE_REQUIRED
 *                        relations from X (i.e. to Y) become LTYPE_FORBIDDEN.
 *                        In other words, if an alternative to X is found,
 *                        X and all of its dependencies will be invalidated.
 * LTYPE_DEPRECATED:      The presence of X deprecates Y. This is not a failure
 *                        condition but a warning will be posted.
 * LTYPE_ALTERNATIVE:     Y is an alternative property to X. If X exists then
 *                        Y (and its dependents) are invalidated. If X does not
 *                        exist but Y does Y is validated in place of X. Then, X
 *                        and any of its dependents are invalidated.
 * LTYPE_ALT*_DEPRECATED: Same behavior as LTYPE_ALTERNATIVE except a warning
 *                        is posted. In other words, Y is an alternative to X
 *                        but Y is deprecated.
 */
enum sv_dt_link_type {
	LTYPE_FORBIDDEN = 0,
	LTYPE_OPTIONAL,
	LTYPE_REQUIRED,
	LTYPE_DEPRECATED,
	LTYPE_ALTERNATIVE,
	LTYPE_ALTERNATIVE_DEPRECATED
};

struct sv_dt_link_attr {
	u32 modes;
	enum sv_dt_link_type type;
};
#define MAKE_ATTRS(_n) \
	struct sv_dt_link_attr _n[TVCF_NVERSIONS]

struct sv_dt_node;
int sv_dt_make_link(struct sv_dt_node *node,
		struct sv_dt_link_attr *attrs,
		const char *source, const char *sink, ...);
int sv_dt_write_attr(
		struct sv_dt_link_attr *attrs,
		const enum sv_dt_link_version version,
		const enum sv_dt_link_type type,
		const u32 modes);

/**
 * Make a link attribute by its full specification.
 *
 * @attrs:   link attributes.
 * @version: sensor driver version attribute applies towards.
 * @status:  status of attribute.
 * @modes:   mode types attribute applies towards.
 */
#define MAKE_LATTR_FULL(attrs, version, status, modes)                        \
	sv_dt_write_attr(attrs, version, status, modes)

/**
 * Make a link attribute that applies to any mode type.
 *
 * @attrs:   link attributes
 * @version: sensor driver version attribute applies towards.
 * @status:  status of attribute.
 */
#define MAKE_LATTR(attrs, version, status)                                    \
	MAKE_LATTR_FULL(attrs, version, status, MODE_TYPE_ANY)

/**
 * Make a DT link by its full specification.
 *
 * @node:   node link will directly (or indirectly) belong to.
 * @attrs:  link attributes.
 * @source: name of source prop (or NULL if none)
 * @sink:   name of prop to be added
 * @...:    variable list of link attributes to make i.e. MAKE_LATTR*
 */
#define MAKE_LINK_FULL(node, attrs, source, sink, ...)                        \
	do {                                                                  \
		memset(attrs, 0, sizeof(*attrs) * TVCF_NVERSIONS);            \
		if (sv_dt_make_link(node, attrs, source, sink,                \
				##__VA_ARGS__) != 0)                          \
			goto make_link_fail;                                  \
	} while (0)

/**
 * Make a DT link with type and mode(s) that applies to all sensor
 * driver versions.
 *
 * @node:   node link will directly (or indirectly) belong to.
 * @attrs:  link attributes.
 * @source: name of source prop (or NULL if one)
 * @sink:   name of prop to be added
 * @type:   link type
 * @modes:  mode types attribute applies towards.
 */
#define MAKE_LINK_ALL_FULL(node, attrs, source, sink, type, modes)            \
	MAKE_LINK_FULL(node, attrs, source, sink,                             \
		MAKE_LATTR_FULL(attrs, TVCF_VERSION_V1_0_0, type, modes),     \
		MAKE_LATTR_FULL(attrs, TVCF_VERSION_V2_0_0, type, modes)      \
	)

/**
 * Make a DT link with type that applies to all sensor driver versions
 * and all mode types.
 *
 * @node:   node link will directly (or indirectly) belong to.
 * @attrs:  link attributes.
 * @source: name of source prop (or NULL if one)
 * @sink:   name of prop to be added
 * @type:   link type
 */
#define MAKE_LINK_ALL(node, attrs, source, sink, type)                        \
	MAKE_LINK_ALL_FULL(node, attrs, source, sink, type, MODE_TYPE_ANY)

#endif // __SENSOR_DT_TEST_H__
