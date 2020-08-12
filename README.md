# TeensyThreads Context Switching
Even though this is a fork from the TeensyThreads library, in reality it's a complete re-write and refractor. Most of the command calls have been moved around to c-style cpp, in reality this is my codebase now execpt for the context switching. 

##Documentation: 

### Creating a new thread example: 
Here is an example where we create a thread, and sleep it every 1/10th of a second. 

```

os_thread_t target_thread; 

void example_thread(void *parameters){
  while(1){
    os_thread_delay_ms(100);
  }
}

void setup(){
  os_init();
  // Instanciate the thread. 
  target_thread = os_add_thread((thread_func_t)blinkthread, 0, -1, 0);
}

void loop(){
  os_thread_delay_ms(100);
}

```
