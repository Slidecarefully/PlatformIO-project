/* 
    Author: Michele Grisafi
    Email: michele.grisafi@unitn.it
    License: MIT 
*/
/*
    Core TCM, containing the verification task that makes sure an image is compliant
    with the AC policy.
    核心 TCM，包含确保映像符合 AC 策略的验证任务。
*/
// Basic MSP430 and driverLib #includes // 基本 MSP430 和驱动程序库 #includes

#include "stdlib.h"
#include "core1.h"

#include <Arduino.h>
#define REJECT 1 /* IF SET to 1 THEN WE REJECT THE APPLICATION WHEN AN INSTRUCTION FAILS VERIFICATION  如果设置为 "1"，则当某项指令未通过验证时，我们将拒绝申请 */ 
#define DEBUG 0 /* IF SET TO 1 WE TAKE NOTE OF WHAT WORD CAUSED THE VERIFICATION TO FAIL 如果设为 1，我们将记录导致验证失败的单词*/
//flash
const uint32_t appBottomText        = 0x000020B4;   //外设驱动程序位置
const uint32_t appTopText           = 0x00012000;

/** These addresses need to be synchronised with the memory layout of the target device **/
/** 这些地址需要与目标设备的内存布局同步 **/

const uint32_t flashTop             = 0x00040000;
const uint32_t flashBottom          = 0x00002000;

const uint32_t ramBottom            = 0x20000000;
const uint32_t ramTop               = 0x20008000;

/** This address is used to store the CFI temporary data during verification**/
/** It must be different depending on whether an update has being received or not:
    If an update has been received then we need the elfData and thus we use the 
    AppROData. This will then be written after the verification.
    If no update is being received then we must use the elfDAta section  **/
/** 该地址用于在验证过程中存储 CFI 临时数据***/
/** 它必须根据是否收到更新而有所不同：
    如果已收到更新，则我们需要 elfData，因此我们使用 
    AppROData。这将在验证后写入。
    如果没有收到更新，则必须使用 elfDAta 部分 **/
__attribute__((section(".tcm:code"))) void secureBoot(){
 // 开启串口通信
   Serial.println("hello");
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);

  //FlashLock(lock1); // 锁定所有闪存区块
  delay(500);// wait for a second
  //noInterrupts();
  digitalWrite(LED_BUILTIN, LOW);  // turn the LED on (HIGH is the voltage level)
  verify(appBottomText,appTopText,0);
  delay(100);
}

__attribute__((section(".tcm:code"))) bool verify(uint32_t address, uint32_t lastAddress, bool cfi){
    //Erase CFI data holder section //擦除CFI数据保持器部分
    //FlashErase();


  /**
    * ********************** WRITE PROTECTION *************    写保护
  */

  /* Temporary variables */   //临时变量
  uint16_t word;
  uint16_t opCode;
  uint16_t operand1;
  uint16_t operand2;
  uint32_t opCodePointer;
  uint32_t operand1Pointer;



  /*************   START VERIFICATION ***************/    //开始验证
  //Count number of instructions 计算指令的数量
  uint32_t count = 0;

  //Value of the PC     PC的值
  uint32_t pc_old;

  while (address + (count*2) < lastAddress)
  {
    
    //Retrieve the value of the PC and of the current word
    //Old value of the PC is used when the current word is not the base word
    //检索PC的值和当前字的值
      //当当前字不是基字时，使用PC的旧值
    pc_old = address+(count*2); 
    //printHex32(pc_old);
    word = *(uint16_t *)pc_old;//当前指令字

      // Decompose instruction into its parts
    //将指令分解为多个部分

    opCodePointer = pc_old ;
    operand1Pointer = pc_old + 2 ;
    opCode      = *(uint16_t *)opCodePointer;
    operand1    = *(uint16_t *)operand1Pointer;

    /************** Detect OP Codes ****************/   // 检查OP代码
      //Serial.print("BL qian ok");
       //BL
    if(((opCode & 0xf800) == 0xf000)&&((operand1 & 0xc000) == 0xc000)){
      if(checkBLAddressInRange(opCode, operand1, pc_old)){
        //printHex32(pc_old);
        //Serial.println("BL in tcm.\n");
      }
      else{
        //printHex32(pc_old);
        //Serial.println("BL out tcm.\n");
      }
      
    }

    //B
    else if((opCode & 0xF800) == 0xE000){
      if(checkBAddressInRange(opCode, pc_old)){
        printHex32(pc_old);
        Serial.println(" B in tcm.\n");
        delay(10);
      }
      else{
        printHex32(pc_old);
        Serial.println(" B out tcm.\n");
        delay(10);
      }
    }
    count +=1;
  }
}
 // 定义锁定和解锁闪存的函数
void FlashLock(bool lock) {
  // 确保NVM就绪
  while (!NVMCTRL->INTFLAG.bit.READY);
  // 遍历所有块地址
  for (uint32_t address = 0x00002000; address < 0x00040000; address += 0x00004000) {
    NVMCTRL->ADDR.reg = address / 2; // 设置地址（以字为单位）
    NVMCTRL->CTRLA.reg = lock ? LOCK_COMMAND : UNLOCK_COMMAND; // 设置锁定或解锁命令
    while (!NVMCTRL->INTFLAG.bit.READY); // 等待操作完成
  }
}

//擦除闪存的函数
__attribute__((section(".tcm:code"))) void FlashErase() {
  // 确保NVM就绪
  while (!NVMCTRL->INTFLAG.bit.READY);
  // 遍历所有块地址
  for (uint32_t address = 0x00002000; address < 0x00040000; address += 0x00004000) {
    NVMCTRL->ADDR.reg = address / 2; // 设置地址（以字为单位）
    NVMCTRL->CTRLA.reg = ERASE_COMMAND; // 设置擦除命令
    while (!NVMCTRL->INTFLAG.bit.READY); // 等待操作完成
  }
}

__attribute__((section(".tcm:code"))) void initializealwDst(volatile uint16_t* array, int size) {
    for (int i = 0; i < size; ++i) {
        array[i] = 0;  // 为每个元素赋值为 0
    }
}

// 检查BL指令的跳转地址是否在指定范围内
__attribute__((section(".tcm:code"))) bool checkBLAddressInRange(uint16_t opcode, uint16_t operand, uint32_t pc) {
    uint32_t S = (opcode >> 10) & 0x1;
    uint32_t imm10 = opcode & 0x3FF;
    uint32_t J1 = (operand >> 13) & 0x1;
    uint32_t J2 = (operand >> 11) & 0x1;
    uint32_t imm11 = operand & 0x7FF;

    uint32_t I1 = !(J1 ^ S);
    uint32_t I2 = !(J2 ^ S);
    int32_t offset = (S << 24) | (I1 << 23) | (I2 << 22) | (imm10 << 12) | (imm11 << 1);
    if (offset & 0x01000000) {
        offset |= 0xFE000000;  // 执行符号扩展
    }

    uint32_t targetAddress = pc + offset+4; // 计算目标地址
    
    return targetAddress < 0x00017000 && targetAddress > 0x00013000; // 检查地址是否在范围内

}

__attribute__((section(".tcm:code")))  bool checkBAddressInRange(uint16_t opcode, uint32_t pc) {
    // 计算目标地址
  int16_t offset = (int16_t)(opcode << 5) >> 4;  // 提取偏移量，进行符号扩展
  uint32_t targetAddress = pc + (uint32_t)offset + 4;
  // 检查目标地址是否在范围之外
  return targetAddress < 0x00017000 && targetAddress > 0x00013000; 
}

__attribute__((section(".tcm:code")))  void printHex32(uint32_t num) {
    char hex[9]; // 存储16进制字符串（8个字符+终止符）
    sprintf(hex, "%08X", num); // 格式化为16进制，确保有8个字符宽度，不足的地方补零
    Serial.print("0x");
    Serial.println(hex); // 打印格式化后的字符串
}

__attribute__((section(".safepop"))) void tcm_pop_function() {
    asm volatile (
        //需要在POP代码之前写这些
        //BL   tcm_pop_function
        // Assembly code  
        "CMP     R2,#0xA5             \n"
        "BEQ     back_pop             \n"
        "POP     {R4,R5}              \n"
        "POP     {R1}                 \n"
        "PUSH    {R4,R5}              \n"
        "PUSH    {R0}                 \n"
        "MOV     R0, #0x13            \n"
        "LSL     R0, R0, #12          \n"
        "CMP     R0,R1                \n"
        "BLT     proceed_pop1         \n"
        "B       fault_pop            \n"
        "proceed_pop1:                 \n"
        "MOV     R0, #0x17            \n"
        "LSL     R0, R0, #12          \n"
        "CMP     R0,R1                \n"
        "BGT     proceed_pop          \n"
        "B       fault_pop            \n"
        "proceed_pop:                 \n");
        Serial.println("POP in tcm.\n");
        asm volatile ( 
        "POP     {R0}                 \n"
        "POP     {R4,R5}              \n"
        "push    {R1}                 \n"
        "push     {R4,R5}              \n"
        "POP     {R4,PC}               \n"
        "back_pop:                      \n"
        "pop     {R4,PC}                \n"
        "fault_pop:                   \n");
        Serial.println("POP out tcm.\n");
        asm volatile ( 
        "POP     {R0}                 \n"
        "POP     {R4,R5}              \n"
        "push    {R1}                 \n"
        "push     {R4,R5}              \n"
        "POP     {R4,PC}               \n"
        );
}

__attribute__((section(".safebx"))) void tcm_bx_function() {
    asm volatile (
        
        //需要在之前就将寄存器放到值赋值给r0，接下来是对r0进行操作
        //例如：BX R2
        //需要在BX代码之前写这些
        //MOV  R0,R2
        //BL   tcm_bl_function
        // Assembly code
        "CMP     R2,#0xA5             \n"
        "BEQ     back_bx          \n"
        "PUSH    {R1}                 \n"
        "MOVS    R1, #0x13            \n"
        "LSL     R1, R1, #12          \n"
        "CMP     R0,R1                \n"
        "BLT     proceed_bx              \n"
        "B       fault_bx                \n"

        "proceed_bx:                     \n"
        "POP     {R1}                 \n"
        "MOV     PC, LR               \n"

    
        "back_bx:                     \n"
        "MOV     PC, LR                 \n"
        "fault_bx:                       \n"
        "MOVS    r1, #1               \n"  // ulVal = 1
        "mov     r0, #0xD             \n"  // ulPin = 13
        "bl      digitalWrite         \n"
 // Approximate delay loop for 0.5 seconds
        "b       fault_bx                \n"
    );
}

__attribute__((section(".safeblx"))) void tcm_blx_function() {
    asm volatile (
        // Assembly code
        //需要在之前就将寄存器放到值赋值给r0，接下来是对r0进行操作
        //例如：BLX R2
        //需要在BLX代码之前写这些
        //PUSH {R0}
        //MOV  R0,R2
        //BL   tcm_blx_function
        // Assembly code
        "CMP     R2,#0xA5             \n"
        "BEQ     back_blx          \n"
        "PUSH    {R1}                 \n"
        "MOVS    R1, #0x13            \n"
        "LSL     R1,R1,#12          \n"
        "CMP     R0,R1                \n"
        "BLT     proceed_blx              \n"
        "B       fault_blx                \n"
        "back_blx:                      \n"
        "MOV     PC, LR                 \n"
        "proceed_blx:                     \n"
        "POP     {R0}                 \n"
        "POP     {R1}                 \n"
        "MOV     PC, LR               \n"

        "fault_blx:                       \n"
        "MOVS    r1, #1               \n"  // ulVal = 1
        "mov     r0, #0xD             \n"  // ulPin = 13
        "bl      digitalWrite         \n"
 // Approximate delay loop for 0.5 seconds
        "b       fault_blx                \n"
    );
}

__attribute__((section(".safemov"))) void tcm_mov_function() {
    asm volatile (
        // Assembly code
        //寄存器寻址的MOV指令只针对目的寄存器是PC来修改。
        //立即数寻址的MOV指令不用管，因为直接MOV的立即数只有8位。
        //需要在之前就将寄存器放到值赋值给r0，接下来是对r0进行操作
        //例如：MOV  PC,R2
        //需要在MOV代码之前写这些
        //PUSH{R0}
        //MOV  R0,R2
        //BL   tcm_mov_function
        // Assembly code
        "CMP     R2,#0xA5             \n"
        "BEQ     proceed_mov          \n"
        "PUSH    {R1}                 \n"
        "MOVS    R1, #0x13            \n"
        "LSL     R1,R1,#12          \n"
        "CMP     R0,R1                \n"
        "BLT     proceed_mov              \n"
        "B       fault_mov               \n"

        "proceed_mov:                     \n"
        "POP     {R0}                 \n"
        "POP     {R1}                 \n"
        "MOV     PC, LR               \n"
        "back_mov:                      \n"
        "MOV     PC, LR                 \n"
        "fault_mov:                       \n"
        "MOVS    r1, #1               \n"  // ulVal = 1
        "mov     r0, #0xD             \n"  // ulPin = 13
        "bl      digitalWrite         \n"
 // Approximate delay loop for 0.5 seconds
        "b       fault_mov                \n"
    );
}
