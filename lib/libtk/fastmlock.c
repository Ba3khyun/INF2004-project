/*
 *----------------------------------------------------------------------
 *    micro T-Kernel 3.00.00
 *
 *    Copyright (C) 2006-2019 by Ken Sakamura.
 *    This software is distributed under the T-License 2.1.
 *----------------------------------------------------------------------
 *
 *    Released by TRON Forum(http://www.tron.org) at 2019/12/11.
 *
 *----------------------------------------------------------------------
 */

/*
 *	fastmlock.c
 *
 *	High-speed exclusive control multi-lock 
 */

#include <config.h>
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#if USE_MULTI_LOCK
/* Buddy 4 diagnostic-only recorder for the device-manager FastMLock.
 * No printf, delays, flag writes or mutex operations are added here.
 * The device manager registers its own lock after CreateMLock().
 * Trace fields are observational only, not synchronized across cores.
 */
#define B4_MLOCK_RING 16
static FastMLock *b4_trace_target;
static volatile UW b4_trace_seq;
static volatile UW b4_trace_kind[B4_MLOCK_RING];
static volatile UW b4_trace_no[B4_MLOCK_RING];
static volatile UW b4_trace_tid[B4_MLOCK_RING];
static volatile UW b4_trace_us[B4_MLOCK_RING];
static volatile UW b4_trace_owner[2];
static volatile UW b4_trace_acqs[2], b4_trace_rels[2], b4_trace_contends[2];
static volatile UW b4_trace_lastwaiter[2];
#define B4_RAW_US (*(volatile UW *)0x40054028UL)

EXPORT void b4_mlock_trace_register(FastMLock *lock)
{
    b4_trace_target = lock;
    b4_trace_seq = 0U;
    b4_trace_owner[0] = b4_trace_owner[1] = 0U;
    b4_trace_acqs[0] = b4_trace_acqs[1] = 0U;
    b4_trace_rels[0] = b4_trace_rels[1] = 0U;
    b4_trace_contends[0] = b4_trace_contends[1] = 0U;
    b4_trace_lastwaiter[0] = b4_trace_lastwaiter[1] = 0U;
}

static void b4_mlock_record(FastMLock *lock, INT no, UW kind)
{
    UW tid, seq, ix;
    if (lock != b4_trace_target || (UINT)no >= 2U) return;
    tid = (UW)tk_get_tid();
    seq = ++b4_trace_seq;
    ix = seq % B4_MLOCK_RING;
    b4_trace_kind[ix] = kind;
    b4_trace_no[ix] = (UW)no;
    b4_trace_tid[ix] = tid;
    b4_trace_us[ix] = B4_RAW_US;
    if (kind == 1U) { b4_trace_owner[no] = tid; b4_trace_acqs[no]++; }
    else if (kind == 2U) { b4_trace_contends[no]++; b4_trace_lastwaiter[no] = tid; }
    else if (kind == 3U) { b4_trace_owner[no] = 0U; b4_trace_rels[no]++; }
}

/* Caller provides 10 words plus four arrays of 16 words. A seqlock is not
 * implied: races may produce one mixed snapshot; repeated reads distinguish. */
EXPORT void b4_mlock_trace_copy(UW *stats, UW *kind, UW *no, UW *tid, UW *us)
{
    UW i;
    stats[0] = b4_trace_seq;
    stats[1] = b4_trace_owner[0]; stats[2] = b4_trace_owner[1];
    stats[3] = b4_trace_acqs[0]; stats[4] = b4_trace_acqs[1];
    stats[5] = b4_trace_rels[0]; stats[6] = b4_trace_rels[1];
    stats[7] = b4_trace_contends[0]; stats[8] = b4_trace_contends[1];
    stats[9] = b4_trace_lastwaiter[0]; stats[10] = b4_trace_lastwaiter[1];
    for (i = 0; i < B4_MLOCK_RING; i++) {
        kind[i] = b4_trace_kind[i]; no[i] = b4_trace_no[i];
        tid[i] = b4_trace_tid[i]; us[i] = b4_trace_us[i];
    }
}

/* ------------------------------------------------------------------------ */
/*
 *	void INC( INT *val )		increment 
 *	void DEC( INT *val )		decrement 
 *	BOOL BTS( UINT *val, INT no )	bit test and set 
 *	void BR( UINT *val, INT no )	bit reset 
 *
 *	The above must be operated exclusively.
 */

/* Experimental SMP fix: local interrupt masking alone does not guard
 * shared FastMLock state against a different RP2040 core.
 * The kernel's public atomic API uses SIO spinlock #1 with IRQ preservation.
 * Keep the original single-core code untouched. No lock ownership override. */
#if TK_SUPPORT_SMP
Inline void INC( INT *val )
{
    (void)atomic_inc((UW *)val);
}

Inline void DEC( INT *val )
{
    (void)atomic_dec((UW *)val);
}

Inline BOOL BTS( UINT *val, INT no )
{
    UW mask = (UW)1U << no;
    return (BOOL)((atomic_bitset((UW *)val, mask) & mask) != 0U);
}

Inline void BR( UINT *val, INT no )
{
    (void)atomic_bitclr((UW *)val, (UW)1U << no);
}
#else
Inline void INC( INT *val )
{
	UINT	imask;

	DI(imask);
	(*val)++;
	EI(imask);
}

Inline void DEC( INT *val )
{
	UINT	imask;

	DI(imask);
	(*val)--;
	EI(imask);
}

Inline BOOL BTS( UINT *val, INT no )
{
	UINT	imask;
	UINT	b;
	UINT	bm = (UINT)(1 << no);

	DI(imask);
	b = *val & bm;
	*val |= bm;
	EI(imask);
	return (BOOL)b;
}

Inline void BR( UINT *val, INT no )
{
	UINT	imask;

	DI(imask);
	*val &= ~(UINT)(1 << no);
	EI(imask);
}


#endif /* TK_SUPPORT_SMP */

/*
 * Lock with wait time designation 
 *	no	lock number 0 - 31 
 */
EXPORT ER MLockTmo( FastMLock *lock, INT no, TMO tmo )
{
	UINT	ptn = (UINT)(1 << no);
	UINT	flg;
	ER	ercd;

	INC(&lock->wai);
	for ( ;; ) {
		if ( !BTS(&lock->flg, no) ) {
            b4_mlock_record(lock, no, 1U);
			ercd = E_OK;
			break;
		}

        b4_mlock_record(lock, no, 2U);
		ercd = tk_wai_flg(lock->id, ptn, TWF_ORW|TWF_BITCLR, &flg, tmo);
		if ( ercd < E_OK ) {
			break;
		}
	}
	DEC(&lock->wai);

	return ercd;
}

/*
 * Lock 
 *	no	Lock number 0 - 31 
 */
EXPORT ER MLock( FastMLock *lock, INT no )
{
	return MLockTmo(lock, no, TMO_FEVR);
}

/*
 * Lock release 
 *	no	Lock number 0 - 31 
 */
EXPORT ER MUnlock( FastMLock *lock, INT no )
{
	UINT	ptn = (UINT)(1 << no);
	ER	ercd;

    b4_mlock_record(lock, no, 3U);
	BR(&lock->flg, no);
	ercd = ( lock->wai == 0 )? E_OK: tk_set_flg(lock->id, ptn);

	return ercd;
}

/*
 * Create multi-lock 
 */
EXPORT ER CreateMLock( FastMLock *lock, CONST UB *name )
{
	T_CFLG	cflg;
	ER	ercd;

	cflg.flgatr  = TA_TPRI | TA_WMUL;
	cflg.iflgptn = 0;
	SetOBJNAME(cflg.exinf, "MLOC");

	lock->id = ercd = tk_cre_flg(&cflg);
	if ( ercd < E_OK ) {
		return ercd;
	}

	lock->wai = 0;
	lock->flg = 0;
	lock->name = name;

	return E_OK;
}

/*
 * Delete multi-lock 
 */
EXPORT ER DeleteMLock( FastMLock *lock )
{
	ER	ercd;

	if ( lock->id <= 0 ) {
		return E_PAR;
	}

	ercd = tk_del_flg(lock->id);
	if ( ercd < E_OK ) {
		return ercd;
	}

	lock->id = 0;

	return E_OK;
}

#endif /* USE_MULTI_LOCK */