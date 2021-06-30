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

#include <linux/device.h>
#include <linux/glob.h>
#include <linux/kref.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/slab.h>

#include "media/tegracam_core.h"
#include "media/tegracam_utils.h"
#include "tegracam_tests.h"
#include "utils/tegracam_log.h"

#include "sensor_dt_test.h"
#include "sensor_dt_test_nodes.h"

#define SV_DT_MAX_LINK_DEPTH          (8)
#define MODE_TYPE_INVALID             (-1)

#define PROP_OK                       "(OK)"
#define PROP_WARN                     "(WARN)"
#define PROP_FAIL                     "(FAIL)"

#define TVCF_VERSION_BUFF_SIZE        (32U)

/**
 * sv_dt_tvcf_version - decomposed tvcf version
 */
struct sv_dt_tvcf_version {
	u8 major;
	u8 minor;
	u8 patch;
};

#define MAKE_TVCF_VERSION(_major, _minor, _patch)                             \
	{                                                                     \
		.major = (_major),                                            \
		.minor = (_minor),                                            \
		.patch = (_patch)                                             \
	}

/**
 * sv_dt_prop - gold DT property
 *
 * @name:     name of prop
 * @links:    links to other properties
 * @refcount: refcount
 * @list:     list entry
 */
struct sv_dt_prop {
	const char *name;
	struct list_head links;
	struct kref refcount;
	struct list_head list;
};

/**
 * sv_dt_link - connection between two properties
 *
 * @attrs:  attribute states for all TVCF versions
 * @source: source property
 * @sink:   sink property
 * @list:   list entry
 */
struct sv_dt_link {
	struct sv_dt_link_attr attrs[TVCF_NVERSIONS];
	struct sv_dt_prop *source;
	struct sv_dt_prop *sink;
	struct list_head list;
};

/**
 * sv_dt_node - gold DT node
 *
 * @name:     name of node
 * @root:     sentinel property for property graph
 * @children: child nodes
 * @list:     list_entry
 * @visited:  flag for visitation state
 */
struct sv_dt_node {
	const char *name;
	struct sv_dt_prop root;
	struct list_head children;
	struct list_head list;
	bool visited;
};

/**
 * sv_dt_ctx - general test context
 *
 * @version: TVCF version being tested against
 * @mode:    capability of mode (bayer, DOL etc.)
 */
struct sv_dt_ctx {
	enum sv_dt_link_version version;
	u32 mode;
};


static const struct sv_dt_tvcf_version sv_dt_tvcf_versions[TVCF_NVERSIONS] = {
	[TVCF_VERSION_V1_0_0] = MAKE_TVCF_VERSION(1, 0, 0),
	[TVCF_VERSION_V2_0_0] = MAKE_TVCF_VERSION(2, 0, 0)
};
static struct sv_dt_ctx sv_ctx;

/**
 * sv_dt_map_tvcf_version - map input TVCF version to nearest testable version.
 *
 * @in_tvcf: tvcf version to map
 */
static enum sv_dt_link_version sv_dt_map_tvcf_version(
		const u32 in_tvcf)
{
	enum sv_dt_link_version lv = 0;
	const struct sv_dt_tvcf_version *tv;
	u32 map_tvcf;
	u32 diff = U32_MAX; // Prime for a large difference
	int i;

	tv = &sv_dt_tvcf_versions[TVCF_MIN_VERSION];
	if (in_tvcf < tegracam_version(tv->major, tv->minor, tv->patch)) {
		camtest_log(KERN_WARNING
				"Input version is less than minimum\n");
		return TVCF_MIN_VERSION;
	}

	for (i = 0; i < TVCF_NVERSIONS; i++) {
		tv = &sv_dt_tvcf_versions[i];
		map_tvcf = tegracam_version(tv->major, tv->minor, tv->patch);
		if (map_tvcf <= in_tvcf) {
			if (in_tvcf - map_tvcf < diff) {
				lv = i;
				diff = in_tvcf - map_tvcf;
			}
		}
	}

	return lv;
}

/**
 * sv_dt_query_sensor_mode_type - query "mode_type" of a modeX node
 *
 * @node: DT mode node
 */
static u32 sv_dt_query_sensor_mode_type(
		struct device_node *node)
{
	struct property *prop;
	const char *type;
	int ret;

	prop = of_find_property(node, "pixel_t", NULL);
	if (prop != NULL)
		return MODE_TYPE_BAYER;

	prop = of_find_property(node, "mode_type", NULL);
	if (prop == NULL)
		return MODE_TYPE_ANY;

	ret = of_property_read_string(node, "mode_type", &type);
	if (ret != 0) {
		camtest_log(KERN_ERR, "Unable to query mode type\n");
		return MODE_TYPE_INVALID;
	}

	if (strcmp(type, "bayer") == 0)
		return MODE_TYPE_BAYER;
	else if (strcmp(type, "bayer_wdr_dol") == 0)
		return MODE_TYPE_WDR_DOL;
	else if (strcmp(type, "bayer_wdr_pwl") == 0)
		return MODE_TYPE_WDR_PWL;

	camtest_log(KERN_WARNING
			"Unknown sensor mode type - defaulting to ANY\n");
	return MODE_TYPE_ANY;
}

static struct sv_dt_prop *__sv_dt_find_prop(struct sv_dt_prop *prop,
		const char *prop_name, const int depth)
{
	struct sv_dt_link *link;
	struct sv_dt_prop *found;

	if (depth > SV_DT_MAX_LINK_DEPTH)
		return NULL;

	if ((prop->name != NULL) && (strcmp(prop->name, prop_name) == 0))
		return prop;

	list_for_each_entry(link, &prop->links, list) {
		found = __sv_dt_find_prop(link->sink, prop_name, depth + 1);
		if (found != NULL)
			return found;
	}

	return NULL;
}

/**
 * sv_dt_find_prop - find a property in property graph
 *
 * @node:      node which property belongs to
 * @prop_name: name of property to find
 *
 * Note: This DFS is limited to a depth of SV_DT_MAX_LINK_DEPTH. If the
 *       depth exceeds this amount NULL is returned as if the property
 *       was not found in a scenario with infinite stack space.
 */
static inline struct sv_dt_prop *sv_dt_find_prop(struct sv_dt_node *node,
		const char *prop_name)
{
	return __sv_dt_find_prop(&node->root, prop_name, 0);
}

/**
 * sv_dt_make_prop - make a gold DT property
 *
 * @name: name of property
 */
static struct sv_dt_prop *sv_dt_make_prop(const char *name)
{
	struct sv_dt_prop *prop;

	prop = kzalloc(sizeof(*prop), GFP_KERNEL);
	if (prop == NULL)
		return NULL;

	prop->name = name;
	INIT_LIST_HEAD(&prop->links);
	INIT_LIST_HEAD(&prop->list);
	kref_init(&prop->refcount);

	return prop;
}

/**
 * sv_dt_put_prop - free a gold DT property
 *
 * @kref: refcount struct
 *
 * Note: This is only intended to be used by kref_*()
 */
static void sv_dt_put_prop(struct kref *kref)
{
	struct sv_dt_prop *prop =
		container_of(kref, struct sv_dt_prop, refcount);
	kfree(prop);
}

/**
 * sv_dt_free_props - free an entire property graph
 *
 * @prop: sentinel property to begin traversal from
 */
static void sv_dt_free_props(struct sv_dt_prop *prop)
{
	struct sv_dt_link *link;
	struct sv_dt_link *temp;

	list_for_each_entry_safe(link, temp, &prop->links, list) {
		sv_dt_free_props(link->sink);
		kref_put(&link->sink->refcount, sv_dt_put_prop);
		list_del(&link->list);
		kfree(link);
	}
}

/**
 * sv_dt_free_node - free a gold DT node and its properties
 *
 * @node: node to free
 */
static inline void sv_dt_free_node(struct sv_dt_node *node)
{
	sv_dt_free_props(&node->root);
	kfree(node);
}

/**
 * sv_dt_make_node - make a gold DT node
 *
 * @name:       name of gold node
 * @make_props: fp defining property creation for the node
 */
static struct sv_dt_node *sv_dt_make_node(const char *name,
		int (*make_props)(struct sv_dt_node *node))
{
	struct sv_dt_node *node;
	int err;

	node = kzalloc(sizeof(*node), GFP_KERNEL);
	if (node == NULL)
		return NULL;

	node->name = name;
	INIT_LIST_HEAD(&node->root.links);
	node->root.name = NULL;
	INIT_LIST_HEAD(&node->children);
	INIT_LIST_HEAD(&node->list);
	node->visited = false;

	err = make_props(node);
	if (err != 0) {
		sv_dt_free_node(node);
		return NULL;
	}

	return node;
}

/**
 * sv_dt_link_nodes - link a source node to a (child) sink node
 *
 * @source: source node
 * @sink:   (child) sink node
 */
static inline void sv_dt_link_nodes(struct sv_dt_node *source,
		struct sv_dt_node *sink)
{
	list_add_tail(&sink->list, &source->children);
}

/**
 * sv_dt_log_prop_reason - format a "reason" for some property relation
 *
 * @status:  status tag
 * @dt_prop: actual DT property who is the recipient of @verb
 * @link:    link of two properties
 * @verb:    verb of how link source relates to link sink
 */
static void sv_dt_log_prop_reason(const char *status,
		struct property *dt_prop,
		struct sv_dt_link *link,
		const char *verb)
{
	if (link->source->name == NULL)
		camtest_log(KERN_INFO "  %s: prop %s %s\n", status,
			(dt_prop == NULL) ? link->sink->name : dt_prop->name,
			verb);
	else
		camtest_log(KERN_INFO "  %s: prop %s %s by %s\n", status,
			(dt_prop == NULL) ? link->sink->name : dt_prop->name,
			verb, link->source->name);
}

/**
 * sv_dt_prop_applies_mode_type - check if mode relation of link applies
 *
 * @link: link
 *
 * For two properties with a relation defined by @link checks to see if the
 * relation is valid for the "mode_type" of the current mode. Practically,
 * if the relation exists (ROOT -> SOME_DOL_PROP) this function returns
 * true if the link mode type (DOL) matches the mode being tested. If the
 * mode being tested is not DOL (in this example) false is returned.
 */
static inline bool sv_dt_prop_applies_mode_type(struct sv_dt_link *link)
{
	u32 modes;

	modes = link->attrs[sv_ctx.version].modes;
	if ((modes == MODE_TYPE_ANY) || (modes & sv_ctx.mode))
		return true;

	return false;
}

/**
 * sv_dt_invalidate_link - invalidates a link
 *
 * @dt_node: node being tested
 * @link:    link
 *
 * "Invalidation" is the process ensuring a link's sink does not exist.
 */
static int sv_dt_invalidate_link(struct device_node *dt_node,
		struct sv_dt_link *link)
{
	struct property *dt_prop;

	dt_prop = of_find_property(dt_node, link->sink->name, NULL);
	if ((dt_prop != NULL) && sv_dt_prop_applies_mode_type(link)) {
		sv_dt_log_prop_reason(PROP_FAIL, dt_prop, link, "precluded");
		return -1;
	}

	return 0;
}

/**
 * sv_dt_warn_link_mode_type_mismatch - warn on property mode type mismatches
 *
 * @dt_prop: node being tested
 * @link:    link
 *
 * Warns if any property doesn't apply to the current mode type, i.e., having
 * a property specific to DOL in a bayer mode_type modeX node.
 */
static void sv_dt_warn_link_mode_type_mismatch(struct property *dt_prop,
		struct sv_dt_link *link)
{
	if (sv_dt_prop_applies_mode_type(link))
		return;

	camtest_log(KERN_INFO "  " PROP_WARN
			": prop %s only valid for mode types:\n",
			dt_prop->name);
	if (link->attrs[sv_ctx.version].modes & MODE_TYPE_BAYER)
		camtest_log(KERN_INFO "    -> bayer\n");
	if (link->attrs[sv_ctx.version].modes & MODE_TYPE_WDR_PWL)
		camtest_log(KERN_INFO "    -> wdr_pwl\n");
	if (link->attrs[sv_ctx.version].modes & MODE_TYPE_WDR_DOL)
		camtest_log(KERN_INFO "    -> wdr_dol\n");
}

/**
 * sv_dt_match_link - find a matching real DT property to gold property
 *
 * @dt_node:    node being tested
 * @link:       link
 * @link_found: bool that is set to indicate if a match was found
 */
static int sv_dt_match_link(struct device_node *dt_node,
		struct sv_dt_link *link,
		bool *link_found)
{
	struct property *dt_prop;
	int status = 0;

	*link_found = false;
	for_each_property_of_node(dt_node, dt_prop) {
		if (glob_match(link->sink->name, dt_prop->name)) {
			*link_found = true;
			sv_dt_warn_link_mode_type_mismatch(dt_prop, link);

			switch (link->attrs[sv_ctx.version].type) {
			case LTYPE_FORBIDDEN:
				sv_dt_log_prop_reason(PROP_FAIL, dt_prop,
						link, "forbidden");
				status = -1;
				break;
			case LTYPE_OPTIONAL:
				sv_dt_log_prop_reason(PROP_OK, dt_prop,
						link, "optional");
				break;
			case LTYPE_REQUIRED:
				sv_dt_log_prop_reason(PROP_OK, dt_prop,
						link, "required");
				break;
			case LTYPE_DEPRECATED:
				sv_dt_log_prop_reason(PROP_WARN, dt_prop,
						link, "deprecated");
				break;
			case LTYPE_ALTERNATIVE:
			case LTYPE_ALTERNATIVE_DEPRECATED:
				/* Processed later */
				break;
			default:
				camtest_log(KERN_ERR "Unknown link type!\n");
				return -1;
			}
		}
	}

	return status;
}

/**
 * sv_dt_link_not_found_ok - returns whether the absence of a link's sink is OK
 *
 * @link: link
 */
static bool sv_dt_link_not_found_ok(struct sv_dt_link *link)
{
	if (!sv_dt_prop_applies_mode_type(link))
		return true;

	switch (link->attrs[sv_ctx.version].type) {
	case LTYPE_FORBIDDEN:
	case LTYPE_OPTIONAL:
		return true;
	default:
		return false;
	}
}

static int sv_dt_verify_link(struct device_node *dt_node,
		struct sv_dt_link *link);

/**
 * sv_dt_masquerade_link - masquerades an imposter link as a real link
 *
 * @dt_node:  node being tested
 * @real:     real link
 * @imposter: imposter who will assume the real link's identity
 *
 * Masquerading is the process of substituting some property in a graph
 * with one of its potential alternatives. In this case, if B is an alternative
 * to A and A fails some criteria we perform a temporary node substitution
 * of B onto A, i.e. B will masquerade as A for verification purposes.
 */
static int sv_dt_masquerade_link(struct device_node *dt_node,
		struct sv_dt_link *real, struct sv_dt_link *imposter)
{
	enum sv_dt_link_type ltype;
	int status;

	ltype = imposter->attrs[sv_ctx.version].type;
	imposter->source = real->source;

	switch (ltype) {
	case LTYPE_ALTERNATIVE:
		imposter->attrs[sv_ctx.version].type = LTYPE_REQUIRED;
		break;
	case LTYPE_ALTERNATIVE_DEPRECATED:
		imposter->attrs[sv_ctx.version].type = LTYPE_DEPRECATED;
		break;
	default:
		camtest_log(KERN_ERR "Unknown alternative type\n");
		return -1;
	}

	status = sv_dt_verify_link(dt_node, imposter);
	imposter->attrs[sv_ctx.version].type = ltype;
	imposter->source = real->sink;

	return status;
}

/**
 * sv_dt_verify_link - verify a link between two properties
 *
 * @dt_node: node being tested
 * @link:    link
 */
static int sv_dt_verify_link(struct device_node *dt_node,
		struct sv_dt_link *link)
{
	LIST_HEAD(alternatives);
	struct sv_dt_link *link_next;
	struct sv_dt_link *temp;
	bool link_found;
	int status = 0;

	/*
	 * Any LTYPE_ALTERNATIVE* links will be separated out into their
	 * own list for convenience to be conditionally handled later.
	 */
	list_for_each_entry_safe(link_next, temp, &link->sink->links, list) {
		switch (link_next->attrs[sv_ctx.version].type) {
		case LTYPE_ALTERNATIVE:
		case LTYPE_ALTERNATIVE_DEPRECATED:
			list_move(&link_next->list, &alternatives);
			break;
		default:
			break;
		}
	}

	status = sv_dt_match_link(dt_node, link, &link_found);
	if (link_found && (status != 0))
		return -1;

	if (!link_found && sv_dt_link_not_found_ok(link))
		return 0;
	else if (!link_found) {
		/* An alternative could potentially exist - do not return */
		status = -1;
	}

	if (status == 0) {
		/*
		 * Link was found and all is OK. Start recursing on children
		 * and invalidate any alternatives.
		 */
		list_for_each_entry(link_next, &link->sink->links, list)
			status |= sv_dt_verify_link(dt_node, link_next);

		list_for_each_entry(link_next, &alternatives, list)
			status |= sv_dt_invalidate_link(dt_node, link_next);
	} else {
		/*
		 * Link is not OK. If there are any alternatives pursue those,
		 * otherwise fail.
		 */
		if (list_empty(&alternatives)) {
			switch (link->attrs[sv_ctx.version].type) {
			case LTYPE_REQUIRED:
				sv_dt_log_prop_reason(PROP_FAIL, NULL, link,
						"required but not found");
				break;
			case LTYPE_DEPRECATED:
				sv_dt_log_prop_reason(PROP_FAIL, NULL, link,
						"deprecated but not found");
				break;
			default:
				break;
			}
			return -1;
		}

		/*
		 * Alternatives exist - reset status and retry validation
		 * masquerading alternatives as the "true" property that has
		 * failed.
		 */
		status = 0;
		list_for_each_entry(link_next, &alternatives, list)
			status |= sv_dt_masquerade_link(dt_node, link,
					link_next);

		/*
		 * Invalidate original dependents from the original property as
		 * we're looking at alternatives now.
		 */
		list_for_each_entry(link_next, &link->sink->links, list)
			status |= sv_dt_invalidate_link(dt_node, link_next);
	}

	list_splice(&alternatives, &link->sink->links);

	return status;
}

/**
 * sv_dt_verify_dt_node - verify a gold DT node
 *
 * @dt_node: node being tested
 * @gnode:   golden representation of @dt_node
 */
static int sv_dt_verify_dt_node(struct device_node *dt_node,
		const struct sv_dt_node *gnode)
{
	struct sv_dt_link *link;
	int status = 0;

	camtest_log(KERN_INFO "Verifying node \"%s\"\n", dt_node->name);
	sv_ctx.mode = sv_dt_query_sensor_mode_type(dt_node);
	list_for_each_entry(link, &gnode->root.links, list) {
		status |= sv_dt_verify_link(dt_node, link);
	}

	if (status != 0)
		camtest_log(KERN_INFO "Node \"%s\" failed verification\n",
				dt_node->name);
	else
		camtest_log(KERN_INFO "Node \"%s\" passed verification\n",
				dt_node->name);
	camtest_log(KERN_INFO "\n");

	return status;
}

/**
 * sv_dt_find_gchild - find a golden child node from a golden parent node
 *
 * @key:     name of golden child node
 * @gparent: golden parent node
 */
static struct sv_dt_node *sv_dt_find_gchild(const char *key,
		const struct sv_dt_node *gparent)
{
	struct sv_dt_node *gchild;

	list_for_each_entry(gchild, &gparent->children, list) {
		if ((gchild->name != NULL) && (glob_match(gchild->name, key)))
			return gchild;
	}

	return NULL;
}

/**
 * sv_dt_walk_dt - verify and walk DT node
 *
 * @dt_node: real DT node
 * @gnode:   gold DT node
 */
static int sv_dt_walk_dt(struct device_node *dt_node,
		struct sv_dt_node *gnode)
{
	struct device_node *dt_next;
	struct sv_dt_node *gold_next;
	int status = 0;

	status = sv_dt_verify_dt_node(dt_node, gnode);
	gnode->visited = true;

	for_each_child_of_node(dt_node, dt_next) {
		gold_next = sv_dt_find_gchild(dt_next->name, gnode);
		if (gold_next != NULL)
			status |= sv_dt_walk_dt(dt_next, gold_next);
	}

	return status;
}

/**
 * sv_dt_check_unvisited_gnode - check for unvisited golden nodes
 *
 * @gnode: gold node root
 */
static int sv_dt_check_unvisited_gnode(struct sv_dt_node *gnode)
{
	int status = 0;
	struct sv_dt_node *gold_next = NULL;

	if (!gnode->visited) {
		camtest_log(KERN_INFO "  Required node \"%s\" not found\n",
				gnode->name);
		return -1;
	}
	camtest_log(KERN_INFO "  Required node \"%s\" visited\n", gnode->name);

	list_for_each_entry(gold_next, &gnode->children, list) {
		status |= sv_dt_check_unvisited_gnode(gold_next);
	}

	return status;
}

/**
 * sv_dt_verify_full_dt - entry point for DT verification
 *
 * @dt_root: real DT root
 * @groot:   gold DT root
 */
static int sv_dt_verify_full_dt(struct device_node *dt_root,
		struct sv_dt_node *groot)
{
	int status = 0;
	int ret;

	camtest_log(KERN_INFO "Sensor DT test starting for root node \"%s\"\n",
			dt_root->name);
	status = sv_dt_walk_dt(dt_root, groot);

	camtest_log(KERN_INFO
			"Checking all required nodes have been visited\n");
	ret = sv_dt_check_unvisited_gnode(groot);
	if (ret == 0)
		camtest_log(KERN_INFO "All required nodes visited\n");
	else
		camtest_log(KERN_INFO "Required nodes missing\n");
	status |= ret;

	return status;
}

/**
 * sv_dt_make_link - make a link
 *
 * @node:   gold DT node who the property will belong to
 * @attrs:  potential attributes for the link
 * @source: source property name
 * @sink:   sink property name
 *
 * If the source property name is NULL @node's sentinel property will be
 * made the source.
 */
int sv_dt_make_link(struct sv_dt_node *node,
		struct sv_dt_link_attr *attrs,
		const char *source, const char *sink, ...)
{
	struct list_head *link_source;
	struct sv_dt_link *link;
	struct sv_dt_prop *prop;

	if ((node == NULL) || (attrs == NULL) || (sink == NULL))
		return -EINVAL;

	/*
	 * If a link source is NULL the link starts at the node's
	 * "sentinel" property. Otherwise, the link starts from some other
	 * property deeper in the graph which needs found.
	 */
	if (source == NULL)
		link_source = &node->root.links;
	else {
		prop = sv_dt_find_prop(node, source);
		if (prop == NULL) {
			camtest_log(KERN_ERR
				"Cannot make prop with unknown source %s\n",
				source);
			return -1;
		}
		link_source = &prop->links;
	}

	link = kzalloc(sizeof(*link), GFP_KERNEL);
	if (link == NULL)
		return -ENOMEM;

	/*
	 * If the sink is found links can be made immediately. Otherwise
	 * the sink property needs created.
	 */
	prop = sv_dt_find_prop(node, sink);
	if (prop == NULL) {
		prop = sv_dt_make_prop(sink);
		if (prop == NULL) {
			kfree(link);
			return -ENOMEM;
		}
	} else
		kref_get(&prop->refcount);

	link->sink = prop;
	link->source = container_of(link_source, struct sv_dt_prop, links);

	/* Apply attributes to the link */
	memcpy(link->attrs, attrs, sizeof(*attrs) * TVCF_NVERSIONS);

	list_add_tail(&link->list, link_source);

	return 0;
}
EXPORT_SYMBOL_GPL(sv_dt_make_link);

/**
 * sv_dt_write_attr - write a link attribute
 *
 * @attrs:   array of attributes to write to
 * @version: link version
 * @type:    link type
 * @modes:   link mode_type modes
 */
int sv_dt_write_attr(
		struct sv_dt_link_attr *attrs,
		const enum sv_dt_link_version version,
		const enum sv_dt_link_type type,
		const u32 modes)
{
	if ((version < 0) || (version > TVCF_NVERSIONS))
		return -EINVAL;

	attrs[version].type = type;
	attrs[version].modes = modes;

	return 0;
}
EXPORT_SYMBOL_GPL(sv_dt_write_attr);

/**
 * sensor_verify_dt - entry point for sensor DT test
 *
 * @node:         root DT node
 * @tvcf_version: tvcf framework version
 */
int sensor_verify_dt(struct device_node *node, const u32 tvcf_version)
{
	struct sv_dt_node *root_node = NULL;
	struct sv_dt_node *modeX_node = NULL;
	char tvcf_buff[TVCF_VERSION_BUFF_SIZE];
	const struct sv_dt_tvcf_version *sv_tv;
	u32 tv;
	int err = 0;

	if (node == NULL)
		return -EINVAL;

	sv_ctx.version = sv_dt_map_tvcf_version(tvcf_version);

	if (sv_ctx.version == TVCF_VERSION_INVALID) {
		camtest_log(KERN_ERR "Invalid TVCF version\n");
		return -EINVAL;
	}

	sv_tv = &sv_dt_tvcf_versions[sv_ctx.version];
	tv = tegracam_version(sv_tv->major, sv_tv->minor, sv_tv->patch);
	format_tvcf_version(tv, tvcf_buff, TVCF_VERSION_BUFF_SIZE);
	camtest_log(KERN_INFO "Testing sensor DT against TVCF version %s\n",
			tvcf_buff);

	/* Construct DT representation */
	root_node = sv_dt_make_node("root", sv_dt_make_root_node_props);
	if (root_node == NULL) {
		camtest_log(KERN_ERR "Could not create root node\n");
		goto sv_fail;
	}
	modeX_node = sv_dt_make_node("mode[0-9]*", sv_dt_make_modeX_node_props);
	if (modeX_node == NULL) {
		camtest_log(KERN_ERR "Could not create modeX node\n");
		goto sv_fail;
	}
	sv_dt_link_nodes(root_node, modeX_node);

	err = sv_dt_verify_full_dt(node, root_node);

sv_fail:
	sv_dt_free_node(root_node);
	sv_dt_free_node(modeX_node);

	if (err == 0)
		camtest_log(KERN_INFO "Sensor DT test passed\n");
	else
		camtest_log(KERN_INFO "Sensor DT test failed\n");

	return err;
}
EXPORT_SYMBOL_GPL(sensor_verify_dt);
