/*
 * Threads.cpp - Library for threading on the Teensy.
 *
 *******************
 * 
 * Copyright 2017 by Fernando Trias.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
 * and associated documentation files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or 
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *******************
 */
#include "TeensyThreads.h"

int current_thread_id = 0 ;
int thread_count = 0;
int thread_error = 0;

int OS_DEFAULT_STACK_SIZE = 4092;
int OS_DEFAULT_TICKS = 10;

//ADDED, total time to spend asleep
volatile int substractor = 0;

//ADDED, per task sleep time info
struct scheduler_info{
  volatile int sleep_time_till_end_tick;
} task_info[MAX_THREADS];

#define __flush_cpu_memory() __asm__ volatile("DMB");

/*
  * The maximum number of threads is hard-coded. Alternatively, we could implement
  * a linked list which would mean using up less memory for a small number of
  * threads while allowing an unlimited number of possible threads. This would
  * probably not slow down thread switching too much, but it would introduce
  * complexity and possibly bugs. So to simplifiy for now, we use an array.
  * But in the future, a linked list might be more appropriate.
  */
ThreadInfo *system_threads[MAX_THREADS];

Threads threads;

unsigned int time_start;
unsigned int time_end;

// These variables are used by the assembly context_switch() function.
// They are copies or pointers to data in Threads and ThreadInfo
// and put here seperately in order to simplify the code.
extern "C" {
  int current_use_systick;      // using Systick vs PIT/GPT
  int current_active_state;          // state of the system (first, start, stop)
  int current_count;
  ThreadInfo *current_thread;  // the thread currently THREAD_running
  void *current_save;
  int current_msp;             // Stack pointers to save
  void *current_sp;
  void load_next_thread_asm() {
    os_get_next_thread();
  }
}

/*
*   @brief allows our program to "yield" out of current subroutine
*   @notes 
*/
extern "C" void _os_yield(void){
    __asm volatile("svc %0" : : "i"(WILL_OS_SVC_NUM));
}

/*
* @brief Sleeps the thread through a hypervisor call. 
* @notes Checks in roughly every milliscond until thread is ready to start THREAD_running again
* @params int milliseconds since last system tick
* @returns none
*/
extern void os_thread_delay_ms(int millisecond){
  int start_del = millis();
  // So let the hypervisor take us away from this thread, and check 
  // Each millisecond 
  while((int)millis() - start_del < millisecond)
    _os_yield();
}

extern "C" void stack_overflow_default_isr() { 
  current_thread->flags = THREAD_ENDED;
}

extern "C" void stack_overflow_isr(void)       __attribute__ ((weak, alias("stack_overflow_default_isr")));

extern unsigned long _estack;   // the main thread 0 stack

os_isr_function_t save_systick_isr;
os_isr_function_t save_svcall_isr;

extern volatile uint32_t systick_millis_count;
extern "C" void systick_isr();
void __attribute((naked, noinline)) threads_systick_isr(void)
{
  if (save_systick_isr) {
    asm volatile("push {r0-r4,lr}");
    (*save_systick_isr)();
    asm volatile("pop {r0-r4,lr}");
  }

  // TODO: Teensyduino 1.38 calls MillisTimer::runFromTimer() from SysTick
  if (current_use_systick) {
    // we branch in order to preserve LR and the stack
    __asm volatile("b context_switch");
  }
  __asm volatile("bx lr");
}

void __attribute((naked, noinline)) threads_svcall_isr(void)
{
  if (save_svcall_isr) {
    asm volatile("push {r0-r4,lr}");
    (*save_svcall_isr)();
    asm volatile("pop {r0-r4,lr}");
  }

  // Get the right stack so we can extract the PC (next instruction)
  // and then see the SVC calling instruction number
  __asm volatile("TST lr, #4 \n"
                 "ITE EQ \n"
                 "MRSEQ r0, msp \n"
                 "MRSNE r0, psp \n");
  register unsigned int *rsp __asm("r0");
  unsigned int svc = ((uint8_t*)rsp[6])[-2];
  if (svc == (WILL_OS_SVC_NUM)) {
    __asm volatile("b context_switch_direct");
  }
  else if (svc == WILL_OS_SVC_NUM_ACTIVE) {
    current_active_state = OS_STARTED;
    __asm volatile("b context_switch_direct_active");
  }
  __asm volatile("bx lr");
}

extern "C" void unused_interrupt_vector(void);

static void __attribute((naked, noinline)) gpt1_isr() {
  GPT1_SR |= GPT_SR_OF1;  // clear set bit
  __asm volatile ("dsb"); // see github bug #20 by manitou48
  __asm volatile("b context_switch");
}

static void __attribute((naked, noinline)) gpt2_isr() {
  GPT2_SR |= GPT_SR_OF1;  // clear set bit
  __asm volatile ("dsb"); // see github bug #20 by manitou48
  __asm volatile("b context_switch");
}

bool gtp1_init(unsigned int microseconds){
  // Initialization code derived from @manitou48.
  // See https://github.com/manitou48/teensy4/blob/master/gpt_isr.ino
  // See https://forum.pjrc.com/threads/54265-Teensy-4-testing-mbed-NXP-MXRT1050-EVKB-(600-Mhz-M7)?p=193217&viewfull=1#post193217

  // keep track of which GPT timer we are using
  static int gpt_number = 0;
  // not configured yet, so find an inactive GPT timer
  if (gpt_number == 0) {
    if (! NVIC_IS_ENABLED(IRQ_GPT1)) {
      attachInterruptVector(IRQ_GPT1, &gpt1_isr);
      NVIC_SET_PRIORITY(IRQ_GPT1, 255);
      NVIC_ENABLE_IRQ(IRQ_GPT1);
      gpt_number = 1;
    }
    else if (! NVIC_IS_ENABLED(IRQ_GPT2)) {
      attachInterruptVector(IRQ_GPT2, &gpt2_isr);
      NVIC_SET_PRIORITY(IRQ_GPT2, 255);
      NVIC_ENABLE_IRQ(IRQ_GPT2);
      gpt_number = 2;
    }
    else {
      // if neither timer is free, we fail
      return false;
    }
  }
  switch (gpt_number) {
    case 1:
      CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON) ;  // enable GPT1 module
      GPT1_CR = 0;                                    // disable timer
      GPT1_PR = 23;                                   // prescale: divide by 24 so 1 tick = 1 microsecond at 24MHz
      GPT1_OCR1 = microseconds - 1;                   // compare value
      GPT1_SR = 0x3F;                                 // clear all prior status
      GPT1_IR = GPT_IR_OF1IE;                         // use first timer
      GPT1_CR = GPT_CR_EN | GPT_CR_CLKSRC(1) ;        // set to peripheral clock (24MHz)
      break;
    case 2:
      CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON) ;  // enable GPT1 module
      GPT2_CR = 0;                                    // disable timer
      GPT2_PR = 23;                                   // prescale: divide by 24 so 1 tick = 1 microsecond at 24MHz
      GPT2_OCR1 = microseconds - 1;                   // compare value
      GPT2_SR = 0x3F;                                 // clear all prior status
      GPT2_IR = GPT_IR_OF1IE;                         // use first timer
      GPT2_CR = GPT_CR_EN | GPT_CR_CLKSRC(1) ;        // set to peripheral clock (24MHz)
      break;
    
    // We weren't able to setup the GP1 module properly :(
    default:
      return false;
  }
  return true;
}

void threads_init(void){
  // initilize thread slots to THREAD_empty
  for(int i=0; i<MAX_THREADS; i++) {
    system_threads[i] = NULL;
  }
  // fill thread 0, which is always THREAD_running
  system_threads[0] = new ThreadInfo();

  // initialize context_switch() globals from thread 0, which is MSP and always THREAD_running
  current_thread = system_threads[0];        // thread 0 is active
  current_save = &system_threads[0]->save;
  current_msp = 1;
  current_sp = 0;
  current_count = OS_DEFAULT_TICKS;
  current_active_state = OS_FIRST_RUN;
  
  system_threads[0]->flags = THREAD_RUNNING;
  system_threads[0]->ticks = OS_DEFAULT_TICKS;
  system_threads[0]->stack = (uint8_t*)&_estack - DEFAULT_STACK0_SIZE;
  system_threads[0]->stack_size = DEFAULT_STACK0_SIZE;

  // commandeer SVCall & use GTP1 Interrupt
  save_svcall_isr = _VectorsRam[11];
  if (save_svcall_isr == unused_interrupt_vector) save_svcall_isr = 0;
  _VectorsRam[11] = threads_svcall_isr;

  current_use_systick = 0; // disable Systick calls
  gtp1_init(1000);       // tick every millisecond
}

int os_start(int prev_state = -1){
  __disable_irq();
  int old_state = current_active_state;
  if (prev_state == -1) prev_state = OS_STARTED;
  current_active_state = prev_state;
  __enable_irq();
  return old_state;
}

/*
 * os_stop() - Stop threading, even if active.
 *
 * If threads have already OS_started, this should be called sparingly
 * because it could destabalize the system if thread 0 is OS_STOPPED.
 */
int os_stop() {
  __disable_irq();
  int old_state = current_active_state;
  current_active_state = OS_STOPPED;
  __enable_irq();
  return old_state;
}


void os_get_next_thread() {

  // First, save the current_sp set by context_switch
  current_thread->sp = current_sp;

  // did we overflow the stack (don't check thread 0)?
  // allow an extra 8 bytes for a call to the ISR and one additional call or variable
  if (current_thread_id && ((uint8_t*)current_thread->sp - current_thread->stack <= 8)) {
    stack_overflow_isr();
  }

  // Find the next THREAD_running thread
  while(1) {
    current_thread_id++;
    if (current_thread_id >= MAX_THREADS) {
      current_thread_id = 0; // thread 0 is MSP; always active so return
      break;
    }
    if (system_threads[current_thread_id] && system_threads[current_thread_id]->flags == THREAD_RUNNING) break;
  }
  current_count = system_threads[current_thread_id]->ticks;

  current_thread = system_threads[current_thread_id];
  current_save = &system_threads[current_thread_id]->save;
  current_msp = (current_thread_id==0?1:0);
  current_sp = system_threads[current_thread_id]->sp;

#ifdef DEBUG
  current_thread->cyclesStart = ARM_DWT_CYCCNT;
#endif
}

/*
 * THREAD_Empty placeholder for IntervalTimer class
 */
static void context_pit_THREAD_empty() {}

/*
 * Store the PIT timer flag register for use in assembly
 */
volatile uint32_t *context_timer_flag;

/*
 * Defined in assembly code
 */
extern "C" void context_switch_pit_isr();

/*
 * Stop using the SysTick interrupt and start using
 * the IntervalTimer timer. The parameter is the number of microseconds
 * for each tick.
 */
int Threads::setMicroTimer(int tick_microseconds)
{
  gtp1_init(tick_microseconds);
  return 1;
}

/*
 * Set each time slice to be 'microseconds' long
 */
int Threads::setSliceMicros(int microseconds)
{
  setMicroTimer(microseconds);
  setDefaultTimeSlice(1);
  return 1;
}

/*
 * Set each time slice to be 'milliseconds' long
 */
int Threads::setSliceMillis(int milliseconds)
{
  if (current_use_systick) {
    setDefaultTimeSlice(milliseconds);
  }
  else {
    // if we're using the PIT, we should probably really disable it and
    // re-establish the systick timer; but this is easier for now
    setSliceMicros(milliseconds * 1000);
  }
  return 1;
}

void os_del_process(void){
  int old_state = os_stop();
  ThreadInfo *me = system_threads[current_thread_id];
  // Would love to delete stack here but the thread doesn't
  // end now. It continues until the next tick.
  // if (me->my_stack) {
  //   delete[] me->stack;
  //   me->stack = 0;
  // }
  thread_count--;
  me->flags = THREAD_ENDED; //clear the flags so thread can stop and be reused
  os_start(old_state);
  while(1); // just in case, keep working until context change when execution will not return to this thread
}

/*
 * Initializes a thread's stack. Called when thread is created
 */
void *os_loadstack(thread_func_t p, void * arg, void *stackaddr, int stack_size)
{
  interrupt_stack_t * process_frame = (interrupt_stack_t *)((uint8_t*)stackaddr + stack_size - sizeof(interrupt_stack_t) - 8);
  process_frame->r0 = (uint32_t)arg;
  process_frame->r1 = 0;
  process_frame->r2 = 0;
  process_frame->r3 = 0;
  process_frame->r12 = 0;
  process_frame->lr = (uint32_t)os_del_process;
  process_frame->pc = ((uint32_t)p);
  process_frame->xpsr = 0x1000000;
  uint8_t *ret = (uint8_t*)process_frame;
  return (void*)ret;
}

/*
 * Add a new thread to the queue.
 *    add_thread(fund, arg)
 *
 *    fund : is a function pointer. The function prototype is:
 *           void *func(void *param)
 *    arg  : is a void pointer that is passed as the first parameter
 *           of the function. In the example above, arg is passed
 *           as param.
 *    stack_size : the size of the buffer pointed to by stack. If
 *           it is 0, then "stack" must also be 0. If so, the function
 *           will allocate the default stack size of the heap using new().
 *    stack : pointer to new data stack of size stack_size. If this is 0,
 *           then it will allocate a stack on the heap using new() of size
 *           stack_size. If stack_size is 0, a default size will be used.
 *    return: an integer ID to be used for other calls
 */

int os_add_thread(thread_func_t p, void * arg, int stack_size, void *stack){
  int old_state = os_stop();
  if (stack_size == -1) stack_size = OS_DEFAULT_STACK_SIZE;
  for (int i=1; i < MAX_THREADS; i++) {
    if (system_threads[i] == NULL) { // THREAD_empty thread, so fill it
      system_threads[i] = new ThreadInfo();
      
    }
    if (system_threads[i]->flags == THREAD_ENDED || system_threads[i]->flags == THREAD_EMPTY) { // free thread
      ThreadInfo *tp = system_threads[i]; // working on this thread
      if (tp->stack && tp->my_stack) {
        delete[] tp->stack;
      }
      if (stack==0) {
        stack = new uint8_t[stack_size];
        tp->my_stack = 1;
      }
      else {
        tp->my_stack = 0;
      }
      tp->stack = (uint8_t*)stack;
      tp->stack_size = stack_size;
      void *psp = os_loadstack(p, arg, tp->stack, tp->stack_size);
      tp->sp = psp;
      tp->ticks = OS_DEFAULT_TICKS;
      tp->flags = THREAD_RUNNING;
      tp->save.lr = 0xFFFFFFF9;

      current_active_state = old_state;
      thread_count++;
      
      if (old_state == OS_STARTED || old_state == OS_FIRST_RUN) 
        os_start();
      return i;
    }
  }
  
  if (old_state == OS_STARTED) 
    os_start();
  return -1;
}

int Threads::getState(int id)
{
  return system_threads[id]->flags;
}

int Threads::setState(int id, int state)
{
  system_threads[id]->flags = state;
  return state;
}

int Threads::wait(int id, unsigned int timeout_ms)
{
  unsigned int start = millis();
  // need to store state in temp volatile memory for optimizer.
  // "while (thread[id].flags != THREAD_RUNNING)" will be optimized away
  volatile int state;
  while (1) {
    if (timeout_ms != 0 && millis() - start > timeout_ms) return -1;
    state = system_threads[id]->flags;
    if (state != THREAD_RUNNING) break;
    yield();
  }
  return id;
}

int Threads::kill(int id)
{
  system_threads[id]->flags = THREAD_ENDED;
  return id;
}

int Threads::suspend(int id)
{
  system_threads[id]->flags = THREAD_ENDED;
  return id;
}

int Threads::restart(int id)
{
  system_threads[id]->flags = THREAD_RUNNING;
  return id;
}

void Threads::setTimeSlice(int id, unsigned int ticks)
{
  system_threads[id]->ticks = ticks - 1;
}

void Threads::setDefaultTimeSlice(unsigned int ticks)
{
  OS_DEFAULT_TICKS = ticks - 1;
}

void Threads::setDefaultStackSize(unsigned int bytes_size)
{
  OS_DEFAULT_STACK_SIZE = bytes_size;
}

void Threads::yield() {
  __asm volatile("svc %0" : : "i"(WILL_OS_SVC_NUM));
}

void Threads::yield_and_start() {
  __asm volatile("svc %0" : : "i"(WILL_OS_SVC_NUM_ACTIVE));
}

void Threads::delay(int millisecond) {
  int mx = millis();
  while((int)millis() - mx < millisecond) yield();
}

void Threads::idle() {
	volatile bool needs_run[thread_count];
	volatile int i, j;
	volatile int task_id_ends;
	__disable_irq();
	task_id_ends = 0;
	//get lowest sleep interval from sleeping tasks into task_id_ends
	for (i = 0; i < thread_count; i++) {
		//sort by ending time first
		for (j = i + 1; j < thread_count; ++j) {
			if (task_info[i].sleep_time_till_end_tick > task_info[j].sleep_time_till_end_tick) {
				//if end time soonest
				if (getState(i+1) == THREAD_ENDED) {
					task_id_ends = j; //store next task
				}
			}
		}
	}
	//set the sleeping time to substractor
	substractor = task_info[task_id_ends].sleep_time_till_end_tick;
	
	if (substractor > 0) {
		//if sleep is needed
		volatile int time_spent_asleep = enter_sleep(substractor);
		//store new data based on time spent asleep
		for (i = 0; i < thread_count; i++) {
			needs_run[i] = 0;
				if (getState(i+1) == THREAD_ENDED) {
				task_info[i].sleep_time_till_end_tick -= time_spent_asleep; //substract sleep time
				//time to run?
				if (task_info[i].sleep_time_till_end_tick <= 0) {
					needs_run[i] = 1;
				} else {
					needs_run[i] = 0;
				}
			}
		}
		//for each thread when slept, resume if needed
		for (i = 0; i < thread_count; i++) {
			if (needs_run[i]) {
				setState(i+1, THREAD_RUNNING);
				task_info[i].sleep_time_till_end_tick = 60000;
			}
		}
	}
	//reset substractor
	substractor = 60000;
	__enable_irq();
	yield();
}

void Threads::sleep(int ms) {
	int i = id();
	if (getState(i) == THREAD_RUNNING) {
		__disable_irq();
		task_info[i-1].sleep_time_till_end_tick = ms;
		setState(i, THREAD_ENDED);
		__enable_irq();
		yield();
	}
}

int Threads::id() {
  volatile int ret;
  __disable_irq();
  ret = current_thread_id;
  __enable_irq();
  return ret;
}

int Threads::getStackUsed(int id) {
  return system_threads[id]->stack + system_threads[id]->stack_size - (uint8_t*)system_threads[id]->sp;
}

int Threads::getStackRemaining(int id) {
  return (uint8_t*)system_threads[id]->sp - system_threads[id]->stack;
}

#ifdef DEBUG
unsigned long Threads::getCyclesUsed(int id) {
  os_stop();
  unsigned long ret = system_threads[id]->cyclesAccum;
  os_start();
  return ret;
}
#endif

/*
 * On creation, stop threading and save state
 */
Threads::Suspend::Suspend() {
  __disable_irq();
  save_state = current_active_state;
  current_active_state = 0;
  __enable_irq();
}

/*
 * On destruction, restore threading state
 */
Threads::Suspend::~Suspend() {
  __disable_irq();
  current_active_state = save_state;
  __enable_irq();
}

int Threads::Mutex::getState() {
  int p = os_stop();
  int ret = state;
  os_start(p);
  return ret;
}

int __attribute__ ((noinline)) Threads::Mutex::lock(unsigned int timeout_ms) {
  if (try_lock()) return 1; // we're good, so avoid more checks

  uint32_t start = systick_millis_count;
  while (1) {
    if (try_lock()) return 1;
    if (timeout_ms && (systick_millis_count - start > timeout_ms)) return 0;
    if (waitthread==-1) { // can hold 1 thread suspend until unlock
      int p = os_stop();
      waitthread = current_thread_id;
      waitcount = current_count;
      threads.suspend(waitthread);
      os_start(p);
    }
    threads.yield();
  }
  __flush_cpu_memory();
  return 0;
}

int Threads::Mutex::try_lock() {
  int p = os_stop();
  if (state == 0) {
    state = 1;
    os_start(p);
    return 1;
  }
  os_start(p);
  return 0;
}

int __attribute__ ((noinline)) Threads::Mutex::unlock() {
  int p = os_stop();
  if (state==1) {
    state = 0;
    if (waitthread >= 0) { // reanimate a suspTHREAD_ended thread waiting for unlock
      threads.restart(waitthread);
      waitthread = -1;
      __flush_cpu_memory();
      threads.yield_and_start();
      return 1;
    }
  }
  __flush_cpu_memory();
  os_start(p);
  return 1;
}
