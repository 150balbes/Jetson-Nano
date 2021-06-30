#
# Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the version 2 of the GNU General Public License
# as published by the Free Software Foundation
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

obj-$(CONFIG_MTTCAN) := mttcan.o

mttcan-y = m_ttcan_linux.o m_ttcan_sys.o ../hal/m_ttcan.o
mttcan-y += ../hal/m_ttcan_intr.o ../hal/m_ttcan_list.o  ../hal/m_ttcan_ram.o
mttcan-y += ../hal/m_ttcan_tt.o

ccflags-y += -I$(overlay)/drivers/net/can/mttcan/include
