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

#include <nvgpu/rbtree.h>

/*
 * rotate node x to left
 */
static void rotate_left(struct nvgpu_rbtree_node **root,
			struct nvgpu_rbtree_node *x)
{
	struct nvgpu_rbtree_node *y = x->right;

	/* establish x->right link */
	x->right = y->left;
	if (y->left) {
		y->left->parent = x;
	}

	/* establish y->parent link */
	y->parent = x->parent;
	if (x->parent) {
		if (x == x->parent->left) {
			x->parent->left = y;
		} else {
			x->parent->right = y;
		}
	} else {
		*root = y;
	}

	/* link x and y */
	y->left = x;
	x->parent = y;
}

/*
 * rotate node x to right
 */
static void rotate_right(struct nvgpu_rbtree_node **root,
			 struct nvgpu_rbtree_node *x)
{
	struct nvgpu_rbtree_node *y = x->left;

	/* establish x->left link */
	x->left = y->right;
	if (y->right) {
		y->right->parent = x;
	}

	/* establish y->parent link */
	y->parent = x->parent;
	if (x->parent) {
		if (x == x->parent->right) {
			x->parent->right = y;
		} else {
			x->parent->left = y;
		}
	} else {
		*root = y;
	}

	/* link x and y */
	y->right = x;
	x->parent = y;
}

/*
 * maintain red-black tree balance after inserting node x
 */
static void insert_fixup(struct nvgpu_rbtree_node **root,
			 struct nvgpu_rbtree_node *x)
{
	/* check red-black properties */
	while ((x != *root) && x->parent->is_red) {
		/* we have a violation */
		if (x->parent == x->parent->parent->left) {
			struct nvgpu_rbtree_node *y = x->parent->parent->right;

			if ((y != NULL) && (y->is_red)) {
				/* uncle is RED */
				x->parent->is_red = false;
				y->is_red = false;
				x->parent->parent->is_red = true;
				x = x->parent->parent;
			} else {
				/* uncle is BLACK */
				if (x == x->parent->right) {
					/* make x a left child */
					x = x->parent;
					rotate_left(root, x);
				}

				/* recolor and rotate */
				x->parent->is_red = false;
				x->parent->parent->is_red = true;
				rotate_right(root, x->parent->parent);
			}
		} else {
			/* mirror image of above code */
			struct nvgpu_rbtree_node *y = x->parent->parent->left;

			if ((y != NULL) && (y->is_red)) {
				/* uncle is RED */
				x->parent->is_red = false;
				y->is_red = false;
				x->parent->parent->is_red = true;
				x = x->parent->parent;
			} else {
				/* uncle is BLACK */
				if (x == x->parent->left) {
					x = x->parent;
					rotate_right(root, x);
				}
				x->parent->is_red = false;
				x->parent->parent->is_red = true;
				rotate_left(root, x->parent->parent);
			}
		}
	}

	(*root)->is_red = false;
}

void nvgpu_rbtree_insert(struct nvgpu_rbtree_node *new_node,
		    struct nvgpu_rbtree_node **root)
{
	struct nvgpu_rbtree_node *curr;
	struct nvgpu_rbtree_node *parent;

	/* find future parent */
	curr = *root;
	parent = NULL;

	while (curr) {
		parent = curr;
		if (new_node->key_start < curr->key_start) {
			curr = curr->left;
		} else if (new_node->key_start > curr->key_start) {
			curr = curr->right;
		} else {
			return; /* duplicate entry */
		}
	}

	/* the caller allocated the node already, just fix the links */
	new_node->parent = parent;
	new_node->left = NULL;
	new_node->right = NULL;
	new_node->is_red = true;

	/* insert node in tree */
	if (parent) {
		if (new_node->key_start < parent->key_start) {
			parent->left = new_node;
		} else {
			parent->right = new_node;
		}
	} else {
		*root = new_node;
	}

	insert_fixup(root, new_node);
}

/*
 * maintain red-black tree balance after deleting node x
 */
static void _delete_fixup(struct nvgpu_rbtree_node **root,
			  struct nvgpu_rbtree_node *parent_of_x,
			  struct nvgpu_rbtree_node *x)
{
	while ((x != *root) && ((x == NULL) || (!x->is_red))) {
		/*
		 * NULL nodes are sentinel nodes. If we delete a sentinel
		 * node (x==NULL) it must have a parent node (or be the root).
		 * Hence, parent_of_x == NULL with
		 * x==NULL is never possible (tree invariant)
		 */

		if ((parent_of_x != NULL) && (x == parent_of_x->left)) {
			struct nvgpu_rbtree_node *w = parent_of_x->right;

			if ((w != NULL) && (w->is_red)) {
				w->is_red = false;
				parent_of_x->is_red = true;
				rotate_left(root, parent_of_x);
				w = parent_of_x->right;
			}

			if ((w == NULL) || (((w->left == NULL) || (!w->left->is_red)) &&
					    ((w->right == NULL) || (!w->right->is_red)))) {
				if (w != NULL) {
					w->is_red = true;
				}
				x = parent_of_x;
			} else {
				if ((w->right == NULL) || (!w->right->is_red)) {
					w->left->is_red = false;
					w->is_red = true;
					rotate_right(root, w);
					w = parent_of_x->right;
				}
				w->is_red = parent_of_x->is_red;
				parent_of_x->is_red = false;
				w->right->is_red = false;
				rotate_left(root, parent_of_x);
				x = *root;
			}
		} else if (parent_of_x != NULL) {
			struct nvgpu_rbtree_node *w = parent_of_x->left;

			if ((w != NULL) && (w->is_red)) {
				w->is_red = false;
				parent_of_x->is_red = true;
				rotate_right(root, parent_of_x);
				w = parent_of_x->left;
			}

			if ((w == NULL) || (((w->right == NULL) || (!w->right->is_red)) &&
					    ((w->left == NULL) || (!w->left->is_red)))) {
				if (w != NULL) {
					w->is_red = true;
				}
				x = parent_of_x;
			} else {
				if ((w->left == NULL) || (!w->left->is_red)) {
					w->right->is_red = false;
					w->is_red = true;
					rotate_left(root, w);
					w = parent_of_x->left;
				}
				w->is_red = parent_of_x->is_red;
				parent_of_x->is_red = false;
				w->left->is_red = false;
				rotate_right(root, parent_of_x);
				x = *root;
			}
		}
		parent_of_x = x->parent;
	}

	if (x != NULL) {
		x->is_red = false;
	}
}

void nvgpu_rbtree_unlink(struct nvgpu_rbtree_node *node,
		    struct nvgpu_rbtree_node **root)
{
	struct nvgpu_rbtree_node *x;
	struct nvgpu_rbtree_node *y;
	struct nvgpu_rbtree_node *z;
	struct nvgpu_rbtree_node *parent_of_x;
	bool y_was_black;

	z = node;

	/* unlink */
	if ((z->left == NULL) || (z->right == NULL)) {
		/* y has a SENTINEL node as a child */
		y = z;
	} else {
		/* find tree successor */
		y = z->right;
		while (y->left) {
			y = y->left;
		}
	}

	/* x is y's only child */
	if (y->left) {
		x = y->left;
	} else {
		x = y->right;
	}

	/* remove y from the parent chain */
	parent_of_x = y->parent;
	if (x != NULL) {
		x->parent = parent_of_x;
	}

	if (y->parent) {
		if (y == y->parent->left) {
			y->parent->left = x;
		} else {
			y->parent->right = x;
		}
	} else {
		*root = x;
	}

	y_was_black = !y->is_red;
	if (y != z) {
		/* we need to replace z with y so
		 * the memory for z can be freed
		 */
		y->parent = z->parent;
		if (z->parent) {
			if (z == z->parent->left) {
				z->parent->left = y;
			} else {
				z->parent->right = y;
			}
		} else {
			*root = y;
		}

		y->is_red = z->is_red;

		y->left = z->left;
		if (z->left) {
			z->left->parent = y;
		}

		y->right = z->right;
		if (z->right) {
			z->right->parent = y;
		}

		if (parent_of_x == z) {
			parent_of_x = y;
		}
	}

	if (y_was_black) {
		_delete_fixup(root, parent_of_x, x);
	}
}

void nvgpu_rbtree_search(u64 key_start, struct nvgpu_rbtree_node **node,
			     struct nvgpu_rbtree_node *root)
{
	struct nvgpu_rbtree_node *curr = root;

	while (curr) {
		if (key_start < curr->key_start) {
			curr = curr->left;
		} else if (key_start > curr->key_start) {
			curr = curr->right;
		} else {
			*node = curr;
			return;
		}
	}

	*node = NULL;
}

void nvgpu_rbtree_range_search(u64 key,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root)
{
	struct nvgpu_rbtree_node *curr = root;

	while (curr) {
		if (key >= curr->key_start &&
				key < curr->key_end) {
			*node = curr;
			return;
		} else if (key < curr->key_start) {
			curr = curr->left;
		} else {
			curr = curr->right;
		}
	}

	*node = NULL;
}

void nvgpu_rbtree_less_than_search(u64 key_start,
			       struct nvgpu_rbtree_node **node,
			       struct nvgpu_rbtree_node *root)
{
	struct nvgpu_rbtree_node *curr = root;

	while (curr) {
		if (key_start <= curr->key_start) {
			curr = curr->left;
		} else {
			*node = curr;
			curr = curr->right;
		}
	}
}

void nvgpu_rbtree_enum_start(u64 key_start, struct nvgpu_rbtree_node **node,
			struct nvgpu_rbtree_node *root)
{
	*node = NULL;

	if (root) {
		struct nvgpu_rbtree_node *curr = root;

		while (curr) {
			if (key_start < curr->key_start) {
				*node = curr;
				curr = curr->left;
			} else if (key_start > curr->key_start) {
				curr = curr->right;
			} else {
				*node = curr;
				break;
			}
		}
	}
}

void nvgpu_rbtree_enum_next(struct nvgpu_rbtree_node **node,
		       struct nvgpu_rbtree_node *root)
{
	struct nvgpu_rbtree_node *curr = NULL;

	if ((root != NULL) && (*node != NULL)) {
		/* if we don't have a right subtree return the parent */
		curr = *node;

		/* pick the leftmost node of the right subtree ? */
		if (curr->right) {
			curr = curr->right;
			for (; curr->left;) {
				curr = curr->left;
			}
		} else {
			/* go up until we find the right inorder node */
			for (curr = curr->parent; curr; curr = curr->parent) {
				if (curr->key_start > (*node)->key_start) {
					break;
				}
			}
		}
	}

	*node = curr;
}
