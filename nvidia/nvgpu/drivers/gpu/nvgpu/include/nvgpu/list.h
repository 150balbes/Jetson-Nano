/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_LIST_H
#define NVGPU_LIST_H
#include <nvgpu/types.h>

struct nvgpu_list_node {
	struct nvgpu_list_node *prev;
	struct nvgpu_list_node *next;
};

static inline void nvgpu_init_list_node(struct nvgpu_list_node *node)
{
	node->prev = node;
	node->next = node;
}

static inline void nvgpu_list_add(struct nvgpu_list_node *new_node, struct nvgpu_list_node *head)
{
	new_node->next = head->next;
	new_node->next->prev = new_node;
	new_node->prev = head;
	head->next = new_node;
}

static inline void nvgpu_list_add_tail(struct nvgpu_list_node *new_node, struct nvgpu_list_node *head)
{
	new_node->prev = head->prev;
	new_node->prev->next = new_node;
	new_node->next = head;
	head->prev = new_node;
}

static inline void nvgpu_list_del(struct nvgpu_list_node *node)
{
	node->prev->next = node->next;
	node->next->prev = node->prev;
	nvgpu_init_list_node(node);
}

static inline bool nvgpu_list_empty(struct nvgpu_list_node *head)
{
	return head->next == head;
}

static inline void nvgpu_list_move(struct nvgpu_list_node *node, struct nvgpu_list_node *head)
{
	nvgpu_list_del(node);
	nvgpu_list_add(node, head);
}

static inline void nvgpu_list_replace_init(struct nvgpu_list_node *old_node, struct nvgpu_list_node *new_node)
{
	new_node->next = old_node->next;
	new_node->next->prev = new_node;
	new_node->prev = old_node->prev;
	new_node->prev->next = new_node;
	nvgpu_init_list_node(old_node);
}

#define nvgpu_list_entry(ptr, type, member)	\
	type ## _from_ ## member(ptr)

#define nvgpu_list_next_entry(pos, type, member)	\
	nvgpu_list_entry((pos)->member.next, type, member)

#define nvgpu_list_first_entry(ptr, type, member)	\
	nvgpu_list_entry((ptr)->next, type, member)

#define nvgpu_list_last_entry(ptr, type, member)	\
	nvgpu_list_entry((ptr)->prev, type, member)

#define nvgpu_list_for_each_entry(pos, head, type, member)	\
	for (pos = nvgpu_list_first_entry(head, type, member);	\
		&pos->member != (head);				\
		pos = nvgpu_list_next_entry(pos, type, member))

#define nvgpu_list_for_each_entry_safe(pos, n, head, type, member)	\
	for (pos = nvgpu_list_first_entry(head, type, member),		\
			n = nvgpu_list_next_entry(pos, type, member);	\
		&pos->member != (head);					\
		pos = n, n = nvgpu_list_next_entry(n, type, member))

#endif /* NVGPU_LIST_H */
