#include <zephyr/kernel.h>
#include <string.h>
#include <soc.h>
#include "trace.h"
#include <zephyr/init.h>
// #include "os_sched.h"

extern void os_zephyr_patch_init(void);
#ifdef CONFIG_PLATFORM_SPECIFIC_INIT
void z_arm_platform_init(void)
{
	DBG_DIRECT("Enter z_arm_platform_init");
}
#endif

static int rtk_platform_init(void)
{
	DBG_DIRECT("Enter rtk_platform_init");
	/* osif */
	// os_zephyr_patch_init();
	// os_init();
	// extern bool osif_test_main(void);
	// osif_test_main();
	return 0;

}

static int do_nothing(){
	return 0;
}

SYS_INIT(do_nothing, APPLICATION, 0);
SYS_INIT(rtk_platform_init, EARLY, 0);
