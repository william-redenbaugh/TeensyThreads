/*
 * Threads.h - Library for threading on the Teensy.
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
 *
 * This project was heavily modified by William Redenbaugh in 2020. But props to the original creator! He did the hard work
 */

#ifndef _THREADS_H
#define _THREADS_H

// Importing primary libraries. 
#include <Arduino.h>
#include <stdint.h>

/*
* @brief Enumerated State of different operating system states. 
* @notes Used for dealing with different threading purposes. 
*/
enum os_state_t{
  OS_UNINITIALIZED  = -1,
  OS_STARTED        = 1, 
  OS_STOPPED        = 2, 
  OS_FIRST_RUN      = 3
};

/*
* @brief Enumerated state of different thread states
* @notes Used for dealing with different threading purposes such as creating, enabling, and deleting a thread. 
*/
enum thread_state_t{
  THREAD_EMPTY    = 0, 
  THREAD_RUNNING  = 1, 
  THREAD_ENDED    = 2, 
  THREAD_ENDING   = 3, 
  THREAD_SUSPENDED    = 4
}; 

/*
*   @brief Maximum amount of threads the Will-OS supports
*   @notes Unless we transition to a linked list(which is unlikely), this will remain the max limit
*/
static const int MAX_THREADS = 128;

/*
* @brief Default Tick set to 100 microseconds per tick
* @notes As far as I know, this isn't the real tick, considering ISR happens 1,000/s not 10,000/s
*/
static const int DEFAULT_TICK_MICROSECONDS = 100;

/*
*   @brief Supervisor call number, used for yielding the program
*/
const int WILL_OS_SVC_NUM = 33; 

/*
*   @brief Supervisor call number, used for yielding the program
*/
const int WILL_OS_SVC_NUM_ACTIVE = 34; 

/*
* @brief Default stack size of thread0, or the loop thread. 
* @notes This should be changed based off how much the user needs. Can be changed using macro 
*/
#ifndef EXTERNAL_STACK0_SIZE
static const int DEFAULT_STACK0_SIZE = 10240; 
#else 
static const int DEFAULT_STACK0_SIZE = EXTERNAL_STACK0_SIZE; 
#endif 
/*
*   @brief register stack frame saved by interrupt
*   @notes Used so that when we get interrupts, we can revert back the original registers. 
*/ 
typedef struct {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
} interrupt_stack_t;

/*
*   @brief Stack frame saved by context switch
*   @notes Used for switching between threads, we save all relevant registers between threads somewhere, and get them when needed
*/ 
typedef struct {
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t lr;
  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t s4;
  uint32_t s5;
  uint32_t s6;
  uint32_t s7;
  uint32_t s8;
  uint32_t s9;
  uint32_t s10;
  uint32_t s11;
  uint32_t s12;
  uint32_t s13;
  uint32_t s14;
  uint32_t s15;
  uint32_t s16;
  uint32_t s17;
  uint32_t s18;
  uint32_t s19;
  uint32_t s20;
  uint32_t s21;
  uint32_t s22;
  uint32_t s23;
  uint32_t s24;
  uint32_t s25;
  uint32_t s26;
  uint32_t s27;
  uint32_t s28;
  uint32_t s29;
  uint32_t s30;
  uint32_t s31;
  uint32_t fpscr;
} software_stack_t;

/*
*   @brief Struct that contains information for each thread
*   @notes Used to deal with thread context switching
*/
typedef struct thread_t{
  int stack_size;
  uint8_t *stack=0;
  int my_stack = 0;
  software_stack_t save;
  volatile int flags = 0;
  void *sp;
  int ticks;
};

/*
* @brief Redeclaration of thread function
* @notes Holds pointer to begining of thread function subroutine. Holds register space for void pointer 
*/
typedef void (*thread_func_t)(void*);

/*
* @brief Redclaration of thread function with integer parameter
* @notes  Holds pointer to begining of thread function subroutine with registers set asside for integer manipulation
*/
typedef void (*thread_func_tInt)(int);

/*
* @brief Redeclaration of thread function with no parameter
* @notes  Holds pointer of begining of thread function subroutine. 
*/
typedef void (*thread_func_tNone)();

/*
* @brief Interrupt service function call
* @notes  Used to deal with ISR functions
*/
typedef void (*os_isr_function_t)();

extern "C" {

/*
* @brief  Assembly call to the context switch subroutine 
* @notes  Uses for switching "contexts" between threads. 
*/
void context_switch(void);

/*
* @brief Assembly call to the context switch direct subroutine"
* @notes  Used for switching "contexts" between threads, bypasses some of the typical context switch subrountes
*/
void context_switch_direct(void);

/*
* @brief Assembly call helping deal with context switching
* @notes n/a
*/
void context_switch_pit_isr(void);

/*
* @brief function that is to be called in assembly to access contexts for thread switching
* @notes  Should only be called for assembly use, not user use
*/
void load_next_thread_asm();

/*
* @brief If our stack overflows, we call this subroutine
* @notes  n/a
*/
void stack_overflow_isr(void);

/*
* @brief isr helping deal with thread stuff
* @notes  n/a
*/
void threads_svcall_isr(void);

/*
* @brief isr helping deal with system tick stuff
* @notes  n/a
*/
void threads_systick_isr(void);
}

/*
*   @brief allows our program to "yield" out of current subroutine
*   @notes Call's hypervisor command to look into something else. 
*/
extern "C" void _os_yield(void);

/*
* @brief Sleeps the thread through a hypervisor call. 
* @notes Checks in roughly every milliscond until thread is ready to start running again
* @params int milliseconds since last system tick
* @returns none
*/
extern void os_thread_delay_ms(int millisecond);

/*
*   @brief Used to startup the Will-OS "Kernel" of sorts
*   @notes Must be called before you do any multithreading with the willos kernel
*   @params none
*   @returns none
*/
void threads_init(void);

void os_get_next_thread();

extern "C" void unused_isr(void);

extern "C" int enter_sleep(int ms);

/*
 * Threads handles all the threading interaction with users. It gets
 * instantiated in a global variable "threads".
 */
class Threads {
public:

  //ADDED, please run in infinite loop
  void idle();
  //ADDED, put mcu in sleep till next execution. doesn't work with delay
  void sleep(int ms);

protected:

public: // public for debugging

public:

  // Create a new thread for function "p", passing argument "arg". If stack is 0,
  // stack allocated on heap. Function "p" has form "void p(void *)".
  int addThread(thread_func_t p, void * arg=0, int stack_size=-1, void *stack=0);
  // For: void f(int)
  int addThread(thread_func_tInt p, int arg=0, int stack_size=-1, void *stack=0) {
    return addThread((thread_func_t)p, (void*)arg, stack_size, stack);
  }
  // For: void f()
  int addThread(thread_func_tNone p, int arg=0, int stack_size=-1, void *stack=0) {
    return addThread((thread_func_t)p, (void*)arg, stack_size, stack);
  }

  // Get the state; see class constants. Can be ThHREAD_EMPTY, RUNNING, etc.
  int getState(int id);
  // Explicityly set a state. See getState(). Call with care.
  int setState(int id, int state);
  // Wait until thread returns up to timeout_ms milliseconds. If ms is 0, wait
  // indefinitely.
  int wait(int id, unsigned int timeout_ms = 0);
  // Permanently stop a running thread. Thread will end on the next thread slice tick.
  int kill(int id);
  // Suspend a thread (on the next slice tick). Can be restarted with restart().
  int suspend(int id);
  // Restart a suspended thread.
  int restart(int id);
  // Set the slice length time in ticks for a thread (1 tick = 1 millisecond, unless using MicroTimer)
  void setTimeSlice(int id, unsigned int ticks);
  // Set the slice length time in ticks for all new threads (1 tick = 1 millisecond, unless using MicroTimer)
  void setDefaultTimeSlice(unsigned int ticks);
  // Set the stack size for new threads in bytes
  void setDefaultStackSize(unsigned int bytes_size);
  // Use the microsecond timer provided by IntervalTimer & PIT; instead of 1 tick = 1 millisecond,
  // 1 tick will be the number of microseconds provided (default is 100 microseconds)
  int setMicroTimer(int tick_microseconds = DEFAULT_TICK_MICROSECONDS);
  // Simple function to set each time slice to be 'milliseconds' long
  int setSliceMillis(int milliseconds);
  // Set each time slice to be 'microseconds' long
  int setSliceMicros(int microseconds);

  // Get the id of the currently running thread
  int id();
  int getStackUsed(int id);
  int getStackRemaining(int id);
#ifdef DEBUG
  unsigned long getCyclesUsed(int id);
#endif

  // Yield current thread's remaining time slice to the next thread, causing immediate
  // context switch
  void yield();
  // Wait for milliseconds using yield(), giving other slices your wait time
  void delay(int millisecond);
  
  // Start/restart threading system; returns previous state: STARTED, STOPPED, OS_FIRST_RUN
  // can pass the previous state to restore
  int start(int old_state = -1);
  // Stop threading system; returns previous state: STARTED, STOPPED, OS_FIRST_RUN
  int stop();

  // Allow these static functions and classes to access our members
  friend void context_switch(void);
  friend void context_switch_direct(void);
  friend void context_pit_isr(void);
  friend void threads_systick_isr(void);
  friend void threads_svcall_isr(void);
  friend void loadNextThread();
  friend class ThreadLock;

protected:
  void getNextThread();
  void *loadstack(thread_func_t p, void * arg, void *stackaddr, int stack_size);
  static void force_switch_isr();

private:
  static void del_process(void);
  void yield_and_start();

public:
  class Mutex {
  private:
    volatile int state = 0;
    volatile int waitthread = -1;
    volatile int waitcount = 0;
  public:
    int getState(); // get the lock state; 1=locked; 0=unlocked
    int lock(unsigned int timeout_ms = 0); // lock, optionally waiting up to timeout_ms milliseconds
    int try_lock(); // if lock available, get it and return 1; otherwise return 0
    int unlock();   // unlock if locked
  };

  class Scope {
  private:
    Mutex *r;
  public:
    Scope(Mutex& m) { r = &m; r->lock(); }
    ~Scope() { r->unlock(); }
  };

  class Suspend {
  private:
    int save_state;
  public:
    Suspend();      // Stop threads and save thread state
    ~Suspend();     // Restore saved state
  };

  template <class C> class GrabTemp {
    private:
      Mutex *lkp;
    public:
      C *me;
      GrabTemp(C *obj, Mutex *lk) { me = obj; lkp=lk; lkp->lock(); }
      ~GrabTemp() { lkp->unlock(); }
      C &get() { return *me; }
  };

  template <class T> class Grab {
    private:
      Mutex lk;
      T *me;
    public:
      Grab(T &t) { me = &t; }
      GrabTemp<T> grab() { return GrabTemp<T>(me, &lk); }
      operator T&() { return grab().get(); }
      T *operator->() { return grab().me; }
      Mutex &getLock() { return lk; }
  };
};
extern Threads threads;

int os_add_thread(thread_func_t p, void * arg, int stack_size, void *stack);

#endif
