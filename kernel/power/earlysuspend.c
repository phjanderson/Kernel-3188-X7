/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"
/****add by nition at 20130527 s**************/
#define ACT8846_PWMLED 
#ifdef ACT8846_PWMLED
#include <linux/i2c.h>  //add by nition
#include <linux/delay.h>  //add by nition
struct act8846 {
	struct device *dev;
	struct mutex io_lock;
	struct i2c_client *i2c;
	int num_regulators;
	struct regulator_dev **rdev;
	struct early_suspend act8846_suspend;
};
//DECLARE_RWSEM(g_act8846);
//EXPORT_SYMBOL_GPL (g_act8846);  //add by nition
extern struct act8846 *g_act8846;  //add by nition
static int act8846_i2c_read(struct i2c_client *i2c, char reg, int count,	u16 *dest)
{
      int ret;
    struct i2c_adapter *adap;
    struct i2c_msg msgs[2];

    if(!i2c)
		return ret;

	if (count != 1)
		return -EIO;  
  
    adap = i2c->adapter;		
    
    msgs[0].addr = i2c->addr;
    msgs[0].buf = &reg;
    msgs[0].flags = i2c->flags;
    msgs[0].len = 1;
    msgs[0].scl_rate = 200*1000;
    
    msgs[1].buf = (u8 *)dest;
    msgs[1].addr = i2c->addr;
    msgs[1].flags = i2c->flags | I2C_M_RD;
    msgs[1].len = 1;
    msgs[1].scl_rate = 200*1000;
    ret = i2c_transfer(adap, msgs, 2);

	//DBG("***run in %s %d msgs[1].buf = %d\n",__FUNCTION__,__LINE__,*(msgs[1].buf));

	return 0;   
}

static int act8846_i2c_write(struct i2c_client *i2c, char reg, int count, const u16 src)
{
	int ret=-1;
	
	struct i2c_adapter *adap;
	struct i2c_msg msg;
	char tx_buf[2];

	if(!i2c)
		return ret;
	if (count != 1)
		return -EIO;
    
	adap = i2c->adapter;		
	tx_buf[0] = reg;
	tx_buf[1] = src;
	
	msg.addr = i2c->addr;
	msg.buf = &tx_buf[0];
	msg.len = 1 +1;
	msg.flags = i2c->flags;   
	msg.scl_rate = 200*1000;	

	ret = i2c_transfer(adap, &msg, 1);
	return ret;	
}

static u8 act8846_reg_read(struct act8846 *act8846, u8 reg)
{
	u16 val = 0;

	mutex_lock(&act8846->io_lock);

	act8846_i2c_read(act8846->i2c, reg, 1, &val);

	//DBG("reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);

	mutex_unlock(&act8846->io_lock);

	return val & 0xff;	
}

static int act8846_set_bits(struct act8846 *act8846, u8 reg, u16 mask, u16 val)
{
	u16 tmp;
	int ret;

	mutex_lock(&act8846->io_lock);

	ret = act8846_i2c_read(act8846->i2c, reg, 1, &tmp);
	//DBG("1 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);
	tmp = (tmp & ~mask) | val;
	if (ret == 0) {
		ret = act8846_i2c_write(act8846->i2c, reg, 1, tmp);
		//DBG("reg write 0x%02x -> 0x%02x\n", (int)reg, (unsigned)val&0xff);
	}
	act8846_i2c_read(act8846->i2c, reg, 1, &tmp);
	//DBG("2 reg read 0x%02x -> 0x%02x\n", (int)reg, (unsigned)tmp&0xff);
	mutex_unlock(&act8846->io_lock);

	return 0;//ret;	
}
#endif
/****add by nition at 20130527 e**************/
enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
	DEBUG_VERBOSE = 1U << 3,
};
static int debug_mask = DEBUG_USER_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

#ifdef CONFIG_PLAT_RK
	if (system_state != SYSTEM_RUNNING)
		return;
#endif

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		if (pos->suspend != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("early_suspend: calling %pf\n", pos->suspend);
			pos->suspend(pos);
		}
	}
	mutex_unlock(&early_suspend_lock);

#ifdef CONFIG_SUSPEND_SYNC_WORKQUEUE
	suspend_sys_sync_queue();
#else
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: sync\n");

	sys_sync();
#endif
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

#ifdef CONFIG_PLAT_RK
	if (system_state != SYSTEM_RUNNING)
		return;
#endif

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link) {
		if (pos->resume != NULL) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("late_resume: calling %pf\n", pos->resume);

			pos->resume(pos);
		}
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
}

void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;
#ifdef ACT8846_PWMLED
	int ret,k=10;	//add by nition 
       u16 val = 0;//add by nition 
#endif
#ifdef CONFIG_PLAT_RK
	if (system_state != SYSTEM_RUNNING)
		return;
#endif

	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
/****add by nition at 20130527 s********************/
#ifdef ACT8846_PWMLED
       if(new_state==3){  //sleep
       struct act8846 *act8846 = g_act8846;
	//ret = act8846_set_bits(act8846, 0xe4,(0x1<<7),(0x0<<7));
		val=0x87;//0xf;//close led
		
	ret = act8846_i2c_write(act8846->i2c, 0xe4, 1, val);
	printk("act8846_i2c_write ret =%d ***********sleep\n",ret);
	while(ret<0){
		ret = act8846_i2c_write(act8846->i2c, 0xe4, 1, val);
		if(ret>0){
			printk("act8846_i2c_write error to ok -----sleep nition\n");
			break;}
		mdelay(20);
		k--;
		if(k<0){
			printk("act8846_i2c_write error -----sleep nition\n");
			break;}
	}
	act8846_i2c_read(act8846->i2c, 0xe4, 1, &val);
	printk("act8846_suspend 0xe4 = %d******************nition\n",val);
       	}
	 else if(new_state==0){  //wakeup 
       struct act8846 *act8846 = g_act8846;
	//ret = act8846_set_bits(act8846, 0xe4,(0x1<<7),(0x1<<7));
		val=0x8f;
	ret = act8846_i2c_write(act8846->i2c, 0xe4, 1, val);	
	printk("act8846_i2c_write ret =%d  ***********wakeup\n",ret);
	while(ret<0){
		ret = act8846_i2c_write(act8846->i2c, 0xe4, 1, val);	
		if(ret>0){
			printk("act8846_i2c_write error to ok  -----wakeup nition\n");
			break;
			}
		mdelay(25);
		k--;
		if(k<0){
			printk("act8846_i2c_write error -----wakeup nition\n");
			break;
			}
	}	
	act8846_i2c_read(act8846->i2c, 0xe4, 1, &val);
	printk("act8846_resume 0xe4 = %d******************nition\n",val);
	 	}
#endif
/****add by nition at 20130527 e**********************/
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		state |= SUSPEND_REQUESTED;
		queue_work(suspend_work_queue, &early_suspend_work);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
		queue_work(suspend_work_queue, &late_resume_work);
	}
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
