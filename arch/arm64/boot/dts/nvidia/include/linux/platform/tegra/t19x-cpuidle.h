#ifndef __T19x_CPUIDLE_C6_LATENCY__
#define __T19x_CPUIDLE_C6_LATENCY__

void force_idle_c6(u64 delay);
int read_cpu_counter(void);
void clear_cpu_counter(void);
#endif

