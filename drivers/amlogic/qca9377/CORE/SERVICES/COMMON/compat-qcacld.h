/*
 * Compatibility header created in order to build the module for several
 * version of the kernel.
 * Originally, CodeAurora release targets kernel 3.10 only.
 * This has been tested against a 3.14 kernel.
 */
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,11,0))
#define cfg80211_send_unprot_disassoc(_dev, _buf, _len) cfg80211_rx_unprot_mlme_mgmt(_dev, _buf, _len)
#define cfg80211_send_unprot_deauth(_dev, _buf, _len) cfg80211_rx_unprot_mlme_mgmt(_dev, _buf, _len)
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,13,0))
#define INIT_COMPLETION(var) reinit_completion(&var)
#endif

#define WIPHY_FLAG_DFS_OFFLOAD BIT(22)

