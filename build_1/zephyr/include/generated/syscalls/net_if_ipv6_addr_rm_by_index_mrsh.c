/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/net_if.h>

extern bool z_vrfy_net_if_ipv6_addr_rm_by_index(int index, const struct in6_addr * addr);
uintptr_t z_mrsh_net_if_ipv6_addr_rm_by_index(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg2;	/* unused */
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; int val; } parm0;
	parm0.x = arg0;
	union { uintptr_t x; const struct in6_addr * val; } parm1;
	parm1.x = arg1;
	bool ret = z_vrfy_net_if_ipv6_addr_rm_by_index(parm0.val, parm1.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}

