# PASTE LINK TO TEAM VIDEO BELOW
#
#

  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global Main
  .global  SysTick_Handler
  .global EXTI0_IRQHandler

  @ Definitions are in definitions.s to keep this file "clean"
  .include "./src/definitions.s"

  .equ    BLINK_PERIOD, 250

  .section .text

Main:
  PUSH  {R10-R12,LR}
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]
  LDR     R4, =GPIOE_MODER
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD3_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD3_PIN*2))      @ write 01 to bits 
  STR     R5, [R4]                      @ Write
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD5_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD5_PIN*2))      @ write 01 to bits 
  STR     R5, [R4]                      @ Write 
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD7_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD7_PIN*2))      @ write 01 to bits 
  STR     R5, [R4]                      @ Write 
  LDR     R5, [R4]                      @ Read ...
  BIC     R5, #(0b11<<(LD9_PIN*2))      @ Modify ...
  ORR     R5, #(0b01<<(LD9_PIN*2))      @ write 01 to bits 
  STR     R5, [R4]                      @ Write  

  LDR   R0, =patternArray @ output R0 = start address
  LDR   R11, =patternArrayLen         @ output R1 = length (measured in bytes)
  MOV   R12, #0

whileGameOngoing:
  CMP   R12, R11          @ R12 is the current length of the game
  BEQ   gameWon           @ if the length of LED array is finished, game is won
  
  MOV   R0, R10
  MOV   R1, R12
  BL    setLED            @ sets the LED based on the array

  MOV   R0, R10
  BL    checkFullInput    @ checks the entire user input, branching to timeToWait and others as needed
  ADD   R12, #1


  CMP   R0, #0            @ checks full input ad displays win or loss
  BNE   roundWon
  BL    displayOutcome
  B     gameLost
roundWon:
  BL    displayOutcome
  B     whileGameOngoing  
  
gameWon:
  MOV   R0, #2
  BL    displayOutcome    @ branches to slightly different win sequence
  
gameLost:

End_Main:
  POP   {R10-R12,PC}
@ inputs, R0 = start address of array, R1 = current length of sequence 
setLED:
  PUSH [R4-R6, LR]
  MOV R6, #0
  .LledLoop:
    CMP R6, R1
    BEQ .LdoneLed
    LDR R5, [R0, R6 , LSL #2]
    CMP R5, #1
    BEQ .Lpin1:
    CMP R5, #2
    BEQ .Lpin2:
    CMP R5, #3
    BEQ .Lpin3:
    CMP R5, #4
    BEQ .Lpin4:
  .LnextLoop:
    ADD R6, R6, #1
    B .LledLoop
  .Lpin1:
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    ORR R5, #(0b1<<(LD3_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    LDR     R0, =BLINK_PERIOD
    BL      delay_ms
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    AND R5, #(0b0<<(LD3_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    B .LnextLoop
  .Lpin2:
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    ORR R5, #(0b1<<(LD5_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    LDR     R0, =BLINK_PERIOD
    BL      delay_ms
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    AND R5, #(0b0<<(LD5_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    B .LnextLoop
  .Lpin3:
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    ORR R5, #(0b1<<(LD7_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    LDR     R0, =BLINK_PERIOD
    BL      delay_ms
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    AND R5, #(0b0<<(LD7_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    B .LnextLoop
  .Lpin4:
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    ORR R5, #(0b1<<(LD9_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    LDR     R0, =BLINK_PERIOD
    BL      delay_ms
    LDR R4, =GPIOE_ODR
    LDR R5, [R4] @ Read ...
    AND R5, #(0b0<<(LD9_PIN)) @ Modify ...
    STR R5, [R4] @ Write
    B .LnextLoop
  .LdoneLed:
    POP [R4-R6, PC]
checkFullInput:
  @ inputs, R0 = start address of array, R1 = current length of sequence
  @ output, R0 = 1 if input correct and R0 = 0 if incorrect

displayOutcome:
  @ input, R0 = type of output to show
  @ output, n/a
delay_ms:
  PUSH  {R4-R5,LR}

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR   R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR   R5, =7999                   @ Assuming a 8MHz clock
  STR   R5, [R4]                    @ 
  
  LDR   R4, =SYSTICK_VAL            @ Reset SysTick internal counter to 0
  LDR   R5, =0x1                    @   by writing any value
  STR   R5, [R4]  

  LDR   R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR   R5, =0x5                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR   R5, [R4]                    @   set ENABLE (bit 0) to 1

.LwhDelay:                          @ while (delay != 0) {
  CMP   R0, #0  
  BEQ   .LendwhDelay  
  
.Lwait:
  LDR   R5, [R4]                    @   Repeatedly load the CSR and check bit 16
  AND   R5, #0x10000                @   Loop until bit 16 is 1, indicating that
  CMP   R5, #0                      @     the SysTick internal counter has counted
  BEQ   .Lwait                      @     from 0x3E7F down to 0 and 1ms has elapsed 

  SUB   R0, R0, #1                  @   delay = delay - 1
  B     .LwhDelay                   @ }

.LendwhDelay:

  LDR   R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR   R5, =0                      @   by writing 0 to CSR
  STR   R5, [R4]                    @   CSR is the Control and Status Register
  
  POP   {R4-R5,PC}

--------------------

  @ Initialise the first countdown

  LDR     R4, =blink_countdown
  LDR     R5, =BLINK_PERIOD
  STR     R5, [R4]  

  @ Configure SysTick Timer to generate an interrupt every 1ms

  LDR     R4, =SCB_ICSR               @ Clear any pre-existing interrupts
  LDR     R5, =SCB_ICSR_PENDSTCLR     @
  STR     R5, [R4]                    @

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming 8MHz clock
  STR     R5, [R4]                    @ 

  LDR     R4, =SYSTICK_VAL            @   Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @     by writing any value
  STR     R5, [R4]

  LDR     R4, =SYSTICK_CSR            @   Start SysTick timer by setting CSR to 0x7
  LDR     R5, =0x7                    @     set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @     set TICKINT (bit 1) to 1 to enable interrupts
                                      @     set ENABLE (bit 0) to 1


  @
  @ Prepare external interrupt Line 0 (USER pushbutton)
  @ We'll count the number of times the button is pressed
  @

  @ Initialise count to zero
  LDR   R4, =button_count             @ count = 0;
  MOV   R5, #0                        @
  STR   R5, [R4]                      @

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt Line0
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on Line0
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt #6 (external interrupt Line0)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]

  @ Nothing else to do in Main
  @ Idle loop forever (welcome to interrupts!!)
Idle_Loop:
  B     Idle_Loop
  
End_Main:
  POP   {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH  {R4, R5, LR}

  LDR   R4, =blink_countdown        @ if (countdown != 0) {
  LDR   R5, [R4]                    @
  CMP   R5, #0                      @
  BEQ   .LelseFire                  @

  SUB   R5, R5, #1                  @   countdown = countdown - 1;
  STR   R5, [R4]                    @

  B     .LendIfDelay                @ }

.LelseFire:                         @ else {

  LDR     R4, =GPIOE_ODR            @   Invert LD3
  LDR     R5, [R4]                  @
  EOR     R5, #(0b1<<(LD3_PIN))     @   GPIOE_ODR = GPIOE_ODR ^ (1<<LD3_PIN);
  STR     R5, [R4]                  @ 

  LDR     R4, =blink_countdown      @   countdown = BLINK_PERIOD;
  LDR     R5, =BLINK_PERIOD         @
  STR     R5, [R4]                  @

.LendIfDelay:                       @ }

  LDR     R4, =SCB_ICSR             @ Clear (acknowledge) the interrupt
  LDR     R5, =SCB_ICSR_PENDSTCLR   @
  STR     R5, [R4]                  @

  @ Return from interrupt handler
  POP  {R4, R5, PC}



@
@ External interrupt line 0 interrupt handler
@   (count button presses)
@
  .type  EXTI0_IRQHandler, %function
EXTI0_IRQHandler:

  PUSH  {R4,R5,LR}

  LDR   R4, =button_count           @ count = count + 1
  LDR   R5, [R4]                    @
  ADD   R5, R5, #1                  @
  STR   R5, [R4]                    @

  LDR   R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV   R5, #(1<<0)                 @
  STR   R5, [R4]                    @

  @ Return from interrupt handler
  POP  {R4,R5,PC}


  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4
patternArray:
.word 1, 2, 3, 4
.equ    patternArrayLen, (.-patternArray)/4

  .end