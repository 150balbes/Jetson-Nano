#!/usr/bin/env python3
#
# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# usage: nvgpu_debug_hal.py [-h] [--csv] [--gk20a GK20A] [gops_filename]
#
# Analyze the HAL debugfs interface's output. With no arguments, prints out
# statistics on the gpu_ops struct based on analysis of gk20a.h
#
# positional arguments:
#   gops_filename  debugfs interface output file (from /d/gpu.0/hal/gops)
#
# optional arguments:
#   -h, --help     show this help message and exit
#   --csv          csv formatted output
#   --gk20a GK20A  path to gk20a.h

import argparse
import re
from os import environ

description_str = ('Analyze the HAL debugfs interface\'s output. '
'With no arguments, prints out statistics on the gpu_ops struct based on '
'analysis of gk20a.h')

parser = argparse.ArgumentParser(description=description_str);
parser.add_argument("--csv", help="csv formatted output", action="store_true");
parser.add_argument("--gk20a", help="path to gk20a.h");
parser.add_argument("gops_filename", help="debugfs interface output file (from /d/gpu.0/hal/gops)", nargs='?');
args = parser.parse_args();

if args.gk20a:
	gk20a_h_path = args.gk20a
else:
	top = environ.get('TOP');
	if top is None:
		print("$TOP is undefined, unable to find gk20a.h");
		exit(-1);
	gk20a_h_path = top + "/kernel/nvgpu/drivers/gpu/nvgpu/gk20a/gk20a.h"

def get_function_pointer_name(line):
	matches = re.search('.*\(\*(?P<function_name>\w+)\)\(', line);
	if matches is None:
		return None
	else:
		return matches.group("function_name");

# Build the list of gpu_ops member function pointers from gk20a.h
non_function_pointer_members = [];
formatted_members = [];
gops_members = dict();
substruct_names = [];
lone_members = [];
with open(gk20a_h_path) as gk20a_h:
	# Skip to start of gpu_ops struct
	while gk20a_h.readline() != "struct gpu_ops {\n":
		continue;

	line = gk20a_h.readline();
	while line != "};\n":
		# If this is a substruct
		if re.match('\t+struct.+\{', line):
			# Read the contents of the substruct
			line = gk20a_h.readline();
			struct_contents = ""
			while not re.match("\t*\} (\w+);", line):
				struct_contents += line;
				line = gk20a_h.readline();
			# Split out the substruct name and the function pointer names
			struct_name = re.match("\t*\} (?P<struct_name>\w+);", line).group("struct_name");
			struct_members = re.findall(r".+?\(\s*\*\s*(\w+)\s*\).+?;", struct_contents, flags=re.DOTALL)

			# Store the substruct as an entry
			substruct_names.append(struct_name);
			gops_members[struct_name] = struct_members;
			# Format members
			for member in struct_members:
				formatted_members.append(struct_name + "." + member);
		else:
			# Lone members (function pointers or stuff not in a substruct)
			match = re.match(".*\(\*(?P<function_name>\w+)\)\(", line);
			if match is not None:
				# It's a function pointer, keep track of it
				lone_members.append(match.group("function_name"));
				formatted_members.append(match.group("function_name"));
			else:
				# Not a function pointer, may also catch comments etc.
				non_function_pointer_members.append(line.strip());
		line = gk20a_h.readline();
if args.gops_filename:
	# Interpret gops file
	with open(args.gops_filename) as gops:
		i = 0;
		# Option for csv output
		if args.csv:
			format_string = '{0},{1}';
		else:
			format_string = '{0:<60} = {1}';
		for line in gops:
			print(format_string.format(formatted_members[i], line[:-1]));
			i += 1;
else:
	# Just print some stats on the gpu_ops struct
	total = 0;
	print("----- Lone Function Pointers -----");
	print("Count =", len(lone_members));
	total += len(lone_members);
	for line in lone_members:
		print(line);
	print("----- Substruct Counts -----");
	for name in substruct_names:
		print(name, "=", len(gops_members[name]));
		total += len(gops_members[name])
	print("\n Total =", total);
