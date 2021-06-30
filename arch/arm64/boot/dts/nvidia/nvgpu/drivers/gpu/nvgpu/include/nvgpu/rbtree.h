/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_RBTREE_H
#define NVGPU_RBTREE_H

#include <nvgpu/types.h>

struct nvgpu_rbtree_node {
	u64 key_start;
	u64 key_end;

	bool is_red; /* !IsRed == IsBlack */

	struct nvgpu_rbtree_node *parent;
	struct nvgpu_rbtree_node *left;
	struct nvgpu_rbtree_node *right;
};

/**
 * nvgpu_rbtree_insert - insert a new node into rbtree
 *
 * @new_node	Pointer to new node.
 * @root	Pointer to root of tree
 *
 * Nodes with duplicate key_start and overlapping ranges
 * are not allowed
 */
void nvgpu_rbtree_insert(struct nvgpu_rbtree_node *new_node,
		    struct nvgpu_rbtree_node **root);

/**
 * nvgpu_rbtree_unlink - delete a node from rbtree
 *
 * @node	Pointer to node to be deleted
 * @root	Pointer to root of tree
 */
void nvgpu_rbtree_unlink(struct nvgpu_rbtree_node *node,
		    struct nvgpu_rbtree_node **root);

/**
 * nvgpu_rbtree_search - search a given key in rbtree
 *
 * @key_start	Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key against key_start of each node
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_search(u64 key_start, struct nvgpu_rbtree_node **node,
			     struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_range_search - search a node with key falling in range
 *
 * @key		Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key and find a node where key value
 * falls within range of {start, end} keys
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_range_search(u64 key,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_less_than_search - search a node with key lesser than given key
 *
 * @key_start	Key to be searched in rbtree
 * @node	Node pointer to be returned
 * @root	Pointer to root of tree
 *
 * This API will match given key and find a node with highest
 * key value lesser than given key
 * In case of a hit, node points to a node with given key
 * In case of a miss, node is NULL
 */
void nvgpu_rbtree_less_than_search(u64 key_start,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_enum_start - enumerate tree starting at the node with specified value
 *
 * @key_start	Key value to begin enumeration from
 * @node	Pointer to first node in the tree
 * @root	Pointer to root of tree
 *
 * This API returns node pointer pointing to first node in the rbtree
 */
void nvgpu_rbtree_enum_start(u64 key_start,
			struct nvgpu_rbtree_node **node,
			struct nvgpu_rbtree_node *root);

/**
 * nvgpu_rbtree_enum_next - find next node in enumeration
 *
 * @node	Pointer to next node in the tree
 * @root	Pointer to root of tree
 *
 * This API returns node pointer pointing to next node in the rbtree
 */
void nvgpu_rbtree_enum_next(struct nvgpu_rbtree_node **node,
		       struct nvgpu_rbtree_node *root);

#endif /* NVGPU_RBTREE_H */
