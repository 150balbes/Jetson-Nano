#include <linux/types.h>
#include <asm/cputype.h>
#include <asm/cpu.h>
#include <asm/smp_plat.h>
#include <asm/cputype.h>

/* MIDR Variant and Revision masked*/
#define MIDR_CPU_MASK		0xFF0FFFF0
#define MIDR_CPU_DENVER2	0x4E0F0030
#define MIDR_CPU_A57		0x410FD070

static inline u8 tegra18_logical_to_cluster(u8 cpu) {
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 1);
}

static inline u8 tegra18_logical_to_cpu(u8 cpu) {
	return MPIDR_AFFINITY_LEVEL(cpu_logical_map(cpu), 0);
}

/* check if CPU is Denver2 */
static inline int tegra18_is_cpu_denver(u8 cpu)
{
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);
	return ((cpuinfo->reg_midr & MIDR_CPU_MASK) == MIDR_CPU_DENVER2);
}

/* check if CPU is A57 */
static inline int tegra18_is_cpu_arm(u8 cpu)
{
	struct cpuinfo_arm64 *cpuinfo = &per_cpu(cpu_data, cpu);
	return ((cpuinfo->reg_midr & MIDR_CPU_MASK) == MIDR_CPU_A57);
}

static inline int tegra18_logical_to_physical_cpu(u8 cpu)
{
	return (tegra18_logical_to_cluster(cpu) << 2) +
		tegra18_logical_to_cpu(cpu);
}
