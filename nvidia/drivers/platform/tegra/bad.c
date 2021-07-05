#include <linux/module.h>
#include <linux/random.h>
#include <linux/string.h>


int __init bad_access(void)
{
	static char source[] = "Twenty characters!!!";
	char dest[10];
	memcpy(dest, source, strlen(source));
        pr_err("%s\n", dest);
	return 0;
}

module_init(bad_access);
