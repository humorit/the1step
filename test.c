//uartputc函数：将字符添加到输出缓冲区uart
void
uartputc(int c)
{
  acquire(&uart_tx_lock);//获取一个关于缓冲区的锁，防止多进程同时写入，做好进程互斥控制
//申请到资源之后，即可以开始
  if(panicked){//如果panicked为1，表明已经panic，不断循环，卡死在此处
    for(;;)
      ;
  }
//用一个while循环判定缓冲区是否有空间
//uart_tx_w：写指针
//uart_tx_r：读指针
//如何判断是否有空间的逻辑：如果写指针追上读指针一圈，就表示缓冲区已满
  while(uart_tx_w == uart_tx_r + UART_TX_BUF_SIZE){
    // buffer is full.缓冲区已满
    // wait for uartstart() to open up space in the buffer.等待uartstart()消耗缓冲区数据
sleep(&uart_tx_r, &uart_tx_lock); //这是挂起当前线程函数，&uart_tx_r是等待条件，等待&uart_tx_r发生变化，即表示有新空间空出来了
//&uart_tx_lock是当前的锁，在挂起过程中会释放资源，根据预防死锁的四种解决方法的其中的破坏请求和保持方式。
  }
//缓冲区为空
  uart_tx_buf[uart_tx_w % UART_TX_BUF_SIZE] = c;//写入数据
  uart_tx_w += 1;//写指针移动
  uartstart();//调用uartstart，检查设备是否真的完成了发送，并将下一个缓冲的输出字符交给设备。
	//因此，如果一个进程向控制台写入多个字节，通常第一个字节将由uartputc调用uartstart发送
	//而剩余的缓冲字节将由uartintr调用uartstart发送，直到传输完成中断到来。
  release(&uart_tx_lock);//释放资源
}


//关于paniced，panicked是一个全局变量，用来标记系统是崩溃，如果置成1，表示系统已经panic，不能再进行同步等操作
void
panic(char *s)
{
  pr.locking = 0;
  printf("panic: ");
  printf("%s\n", s);
  panicked = 1; // freeze uart output from other CPUs
  for(;;)
    ;
}

//uartputc -->塞数据 --> uartstart -->触发发送，这里是拿来输出的
void
uartstart()
{
  while(1){
    if(uart_tx_w == uart_tx_r){
      // transmit buffer is empty.缓冲区为空，读ISR（清除中断或确认状态），直接退出
      ReadReg(ISR);
      return;
    }
    
    if((ReadReg(LSR) & LSR_TX_IDLE) == 0){
      // the UART transmit holding register is full, 
      // so we cannot give it another byte.
      // it will interrupt when it's ready for a new byte.
// UART硬件在忙，不能发送数据，发送完后会有中断来唤醒再发
      return;
    }
    
    int c = uart_tx_buf[uart_tx_r % UART_TX_BUF_SIZE];//取一个要发的字节
    uart_tx_r += 1;//更新读指针
    
    // maybe uartputc() is waiting for space in the buffer.
    wakeup(&uart_tx_r);//唤醒可能因为缓冲区满而sleep的uartputc()，通知可以继续加字节了
    
    WriteReg(THR, c);//把字节写入UART的发送寄存器，硬件负责发出去
  }
}

//剩余的缓冲字节将由uartintr调用uartstart发送，直到传输完成中断到来
void
uartintr(void)
{
  // read and process incoming characters.
  while(1){
    int c = uartgetc();//读取UART接受寄存器
    if(c == -1)//如果uartgetc()放回-1说明没有数据了，则退出循环
      break;
    consoleintr(c);//每读一个字节就扔给 consoleintr(c)
  }

  // send buffered characters.
  acquire(&uart_tx_lock);//上锁，进行进程冲突管理
  uartstart();//调用uartstart()，把缓冲区里还没发的字节继续发。
  release(&uart_tx_lock);//释放资源锁
}



void
timerinit()
{
  // enable supervisor-mode timer interrupts.
  w_mie(r_mie() | MIE_STIE);//打开mie寄存器里的Supervisor Timer Interrupt Enable (STIE) 位，允许管理模式响应定时器中断
  
  // enable the sstc extension (i.e. stimecmp).
  w_menvcfg(r_menvcfg() | (1L << 63)); //启用stimecmp寄存器，让S-mode 可以做定时器中断比较，当stimecmp<=time时触发STIP中断
  
  // allow supervisor to use stimecmp and time.
  w_mcounteren(r_mcounteren() | 2);//控制S/VS-mode对stimecmp和vstimecmp的访问权限，允许S-mode访问stime和stimecmp
  
  // ask for the very first timer interrupt.
  w_stimecmp(r_time() + 1000000);//设置第一个定时器中断时间点，当前时间加上1,000,000个时钟周期，到点就触发定时器中断

}

static inline void 
w_mie(uint64 x)
{
  asm volatile("csrw mie, %0" : : "r" (x));//汇编指令，将变量写入mie寄存器
}

static inline void 
w_menvcfg(uint64 x)
{
  // asm volatile("csrw menvcfg, %0" : : "r" (x));
  asm volatile("csrw 0x30a, %0" : : "r" (x));//汇编指令，将变量写入menvcfg寄存器编号（CSR地址）
}
// Machine-mode Counter-Enable
static inline void 
w_mcounteren(uint64 x)
{
  asm volatile("csrw mcounteren, %0" : : "r" (x));//一样效果
}

static inline void 
w_stimecmp(uint64 x)
{
  // asm volatile("csrw stimecmp, %0" : : "r" (x));
  asm volatile("csrw 0x14d, %0" : : "r" (x));
}

//处理由定时器中断产生的软件中断的代码
int
devintr()
{
  uint64 scause = r_scause();//读scause寄存器，判别中断异常原因

  if(scause == 0x8000000000000009L){//如果是来源是PLIC的supervisor外部中断
    // this is a supervisor external interrupt, via PLIC.

    // irq indicates which device interrupted.
    int irq = plic_claim();//向PLIC询问哪个设备发了中断号

    if(irq == UART0_IRQ){//如果是UART0设备对应的中断号，用uartintr()函数处理
      uartintr();
    } else if(irq == VIRTIO0_IRQ){//如果是VIRTO0设备对应的中断号，用virtio_disk_intr ()函数处理
      virtio_disk_intr();
    } else if(irq){//不然则打印信息，表示这是未知中断号
      printf("unexpected interrupt irq=%d\n", irq);
    }

    // the PLIC allows each device to raise at most one
    // interrupt at a time; tell the PLIC the device is
// now allowed to interrupt again.
//通知PLIC这次中断处理完了，可以给这个设备放行下一个中断
    if(irq)
      plic_complete(irq);

    return 1;//表示处理了一个外部设备处理
  } else if(scause == 0x8000000000000005L){//如果是supervisor定时器中断
    // timer interrupt.
    clockintr();//调用clockintr()函数处理定时器中断
    return 2;//表示定时器中断
  } else {
    return 0;//表示没处理，不是设备和定时器中断
  }

//处理定时器中断
void
clockintr()
{
  if(cpuid() == 0){//只使用CPU0来更新时间，防止多核同时加载
    acquire(&tickslock);//锁，进行互斥控制
    ticks++;//滴答数加一
    wakeup(&ticks);//叫醒因tick（滴答）而sleep等待时间过去的进程
    release(&tickslock);//释放锁
  }

  // ask for the next timer interrupt. this also clears
  // the interrupt request. 1000000 is about a tenth
  // of a second.
  w_stimecmp(r_time() + 1000000);//安排下一次中断
}
