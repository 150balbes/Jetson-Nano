/*
 * dc_client.c: Functions implementing tegra_dc_client interface.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
 * Author: Arun Swain <arswain@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/err.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"

/*
 * tegra_dc_client_allocate_client_id() - Allocates client id.
 * @clients_info : Pointer to a head specific client info.
 *
 * Allocates unique client id and the first available client id from the
 * client id pool.
 *
 * Return : The id if successful else error value.
 */
static int tegra_dc_client_allocate_client_id(
		struct tegra_dc_clients_info *clients_info)
{
	int id;

	id = find_first_zero_bit(&clients_info->client_id_map,
						MAX_NO_DC_CLIENTS);
	if (id >= MAX_NO_DC_CLIENTS)
		return -EMFILE;

	set_bit(id, &clients_info->client_id_map);

	return id;
}

/*
 * tegra_dc_client_remove_client_id() - De-allocates a client id.
 * @client_id : the id to be removed.
 * @clients_info : Pointer to a head specific client info.
 *
 * Returns back a client id to the resource pool.
 *
 * Retrun : 0 if successful else error value.
 */
static int tegra_dc_client_remove_client_id(int client_id,
			struct tegra_dc_clients_info *clients_info)
{
	if (client_id >= MAX_NO_DC_CLIENTS)
		return -ENOENT;

	clear_bit(client_id, &clients_info->client_id_map);

	return 0;
}

/*
 * tegra_dc_client_read_callback_fn_data() - parses the callback data from
 *					the client.
 * @client : Pointer to the client structure received during registration.
 * @clnt_data : Pointer the individual client specific data for which the
 * parsing is needed.
 *
 * Parses the callback data and stores them in the clients specific data
 * structure in tegra_dc.
 *
 * Retrun : 0 if successful else error value.
 */
static int tegra_dc_client_read_callback_fn_data(struct tegra_dc_client *client,
					struct tegra_dc_client_data *clnt_data)
{
	int i;
	enum tegra_dc_client_cllbck_event_type callback_type;

	for (i = 0; i < client->nr_callbacks; i++) {
		void *ptr = NULL;

		callback_type = client->callback_data[i].callback_type;
		if (callback_type >= MAX_EVENT)
			return -ENOENT;

		ptr = client->callback_data[i].callback_fn;
		if (!ptr) {
			pr_warn("Invalid callback fn in %s\n", __func__);
			return -EINVAL;
		}

		clnt_data->callback_fn[callback_type] = ptr;
	}

	return 0;
}

/*
 * tegra_dc_register_client - used by clients to register with dc driver
 * @client : pointer to client's data
 *
 * - registers the client to pertinent dc.
 * - allocates an unique client id.
 * - parses and stores the callback functions requested by the client.
 *
 * Return: 0 if no errors else corresponding error value.
 */
int tegra_dc_register_client(struct tegra_dc_client *client)
{
	int ret;
	struct tegra_dc *dc;

	if (!client)
		return -EINVAL;

	dc = tegra_dc_get_dc(client->disp_id);
	if (!dc)
		return -ENODEV;

	client->client_id =
		tegra_dc_client_allocate_client_id(&dc->clients_info);
	if (client->client_id < 0)
		return client->client_id;

	dc->clients_info.client_data[client->client_id].usr_ctx =
							client->usr_ctx;

	ret = tegra_dc_client_read_callback_fn_data(client,
		&dc->clients_info.client_data[client->client_id]);
	if (ret) {
		dev_err(&dc->ndev->dev,
			"tegra_dc_client_parse_callback_fn_data failed with ret = %d",
			ret);
		tegra_dc_unregister_client(client);
		return ret;
	}

	dc->clients_info.client_data[client->client_id].registered = true;

	return 0;
}
EXPORT_SYMBOL(tegra_dc_register_client);

/*
 * tegra_dc_unregister_client - used by clients to unregister with dc driver
 * @client : pointer to client's data
 *
 * - removes the client from pertinent dc.
 * - de-allocates an unique client id.
 *
 * Return: 0 if no errors else corresponding error value.
 */

int tegra_dc_unregister_client(struct tegra_dc_client *client)
{
	int ret;
	bool status;
	struct tegra_dc *dc;

	if (!client)
		return -EINVAL;

	dc = tegra_dc_get_dc(client->disp_id);
	if (!dc)
		return -ENODEV;

	status = dc->clients_info.client_data[client->client_id].registered;
	if (!status) {
		dev_warn(&dc->ndev->dev, "%s: client not registered\n",
								__func__);
		return -ENOENT;
	}
	ret = tegra_dc_client_remove_client_id(client->client_id,
						&dc->clients_info);
	if (ret)
		return ret;

	memset(&(dc->clients_info.client_data[client->client_id]),
				0, sizeof(struct tegra_dc_client_data));

	return 0;
}
EXPORT_SYMBOL(tegra_dc_unregister_client);

/*
 * handle_state_dc_enabled_event() - notifies the client of DC_ENABLED_EVENT.
 * @client_data : Pointer the individual client specific data.
 * @disp_id : The disp_id for which this event has occurred.
 *
 * Return : 0 always for now.
 */
static int handle_state_dc_enabled_event(struct tegra_dc_client_data
						*client_data, int disp_id)
{
	tegra_dc_notify_dc_enabled_event callback_fn;

	callback_fn = (tegra_dc_notify_dc_enabled_event)
			client_data->callback_fn[NOTIFY_DC_ENABLED_EVENT];
	if (callback_fn)
		callback_fn(disp_id, client_data->usr_ctx);

	return 0;
}

/*
 * handle_state_dc_disabled_event() - notifies the client of DC_DISABLED_EVENT.
 * @client_data : Pointer the individual client specific data.
 * @disp_id : The disp_id for which this event has occurred.
 *
 * Return : 0 always for now.
 */
static int handle_state_dc_disabled_event(struct tegra_dc_client_data
						*client_data, int disp_id)
{
	tegra_dc_notify_dc_disabled_event callback_fn;

	callback_fn = (tegra_dc_notify_modeset_event)
			client_data->callback_fn[NOTIFY_DC_DISABLED_EVENT];

	if (callback_fn)
		callback_fn(disp_id, client_data->usr_ctx);

	return 0;
}

/*
 * handle_modeset_event() - notifies the client of MODESET_EVENT.
 * @client_data : Pointer the individual client specific data.
 * @disp_id : The disp_id for which this event has occurred.
 *
 * Return : 0 always for now.
 */
static int handle_modeset_event(struct tegra_dc_client_data *client_data,
								int disp_id)
{
	tegra_dc_notify_modeset_event callback_fn;

	callback_fn = (tegra_dc_notify_modeset_event)
			client_data->callback_fn[NOTIFY_MODESET_EVENT];

	if (callback_fn)
		callback_fn(disp_id, client_data->usr_ctx);

	return 0;
}

/*
 * tegra_dc_client_handle_event() - handles all the events registered by
 *					clients for notifications.
 * @dc : ptr to tegra_dc for which an event has occurred.
 * @event_type : specifies the type of event.
 *
 * Return : 0 if successful else error value.
 */
int tegra_dc_client_handle_event(struct tegra_dc *dc,
		enum tegra_dc_client_cllbck_event_type event_type)
{
	int i;
	struct tegra_dc_clients_info *clients_info;

	if (!dc)
		return -ENODEV;

	clients_info = &dc->clients_info;
	for_each_set_bit(i, &clients_info->client_id_map, MAX_NO_DC_CLIENTS) {

		switch (event_type) {

		case NOTIFY_DC_ENABLED_EVENT:
			handle_state_dc_enabled_event(
				&clients_info->client_data[i], dc->ndev->id);
			break;
		case NOTIFY_DC_DISABLED_EVENT:
			handle_state_dc_disabled_event(
				&clients_info->client_data[i], dc->ndev->id);
			break;
		case NOTIFY_MODESET_EVENT:
			handle_modeset_event(&clients_info->client_data[i],
								dc->ndev->id);
			break;
		default:
			dev_warn(&dc->ndev->dev, "%s: invalid event type\n",
								__func__);
		}
	}

	return 0;
}
