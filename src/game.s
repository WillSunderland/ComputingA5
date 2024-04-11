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

  .equ    BLINK_PERIOD, 1000
  .equ    WAIT_PERIOD, 5000

  .section .text

Main:
  PUSH    {R10-R12,LR}
  LDR     R4, =RCC_AHBENR
  LDR     R5, [R4]
  ORR     R5, R5, #(0b1 << (RCC_AHBENR_GPIOEEN_BIT))
  STR     R5, [R4]
  LDR     R4, =GPIOE_MODER
  LDR     R5, =0x55550000
  STR     R5, [R4]                      @ Write  

  @ Initial count of button presses to 0
  @ Count must be maintained in memory - interrupt handlers
  @   must not rely on registers to maintain values across
  @   different invocations of the handler (i.e. across
  @   different presses of the pushbutton)
  LDR     R4, =button_count        @ count = 0;
  MOV     R5, #0            @
  STR     R5, [R4]          @

  @ Configure USER pushbutton (GPIO Port A Pin 0 on STM32F3 Discovery
  @   kit) to use the EXTI0 external interrupt signal
  @ Determined by bits 3..0 of the External Interrrupt Control
  @   Register (EXTIICR)
  @ STM32F303 Reference Manual 12.1.3 (pg. 249)
  LDR     R4, =SYSCFG_EXTIICR1
  LDR     R5, [R4]
  BIC     R5, R5, #0b1111
  STR     R5, [R4]

  @ Enable (unmask) interrupts on external interrupt EXTI0
  @ EXTI0 corresponds to bit 0 of the Interrupt Mask Register (IMR)
  @ STM32F303 Reference Manual 14.3.1 (pg. 297)
  LDR     R4, =EXTI_IMR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Set falling edge detection on EXTI0
  @ EXTI0 corresponds to bit 0 of the Falling Trigger Selection
  @   Register (FTSR)
  @ STM32F303 Reference Manual 14.3.4 (pg. 298)
  LDR     R4, =EXTI_FTSR
  LDR     R5, [R4]
  ORR     R5, R5, #1
  STR     R5, [R4]

  @ Enable NVIC interrupt channel (Nested Vectored Interrupt Controller)
  @ EXTI0 corresponds to NVIC channel #6
  @ Enable channels using the NVIC Interrupt Set Enable Register (ISER)
  @ Writing a 1 to a bit enables the corresponding channel
  @ Writing a 0 to a bit has no effect
  @ STM32 Cortex-M4 Programming Manual 4.3.2 (pg. 210)
  LDR     R4, =NVIC_ISER
  MOV     R5, #(1<<6)
  STR     R5, [R4]


  LDR     R10, =patternArray      @ arrayAddress = patternArray
  LDR     R12, =patternArrayLen   @ arrayLength = patternArrayLen    
  MOV     R11, #1     
.LFor2:                           @ for (i = 1, i <= arrayLength, i++) {
  CMP     R11, R12 
  BHI     .LendFor2
  MOV     R0, R10
  MOV     R1, R11
  BL      setLED                  @   setLED(arrayAddress, i)
  LDR     R3, =button_count       
  MOV     R4, #0                  @   button_count = 0
  STR     R4, [R3]
  LDR     R0, =WAIT_PERIOD
  BL      delay_ms                @   delay_ms(WAIT_PERIOD)
  MOV     R0, R10
  MOV     R1, R11 
  LDR     R3, =button_count
  LDR     R2, [R3]
  BL      checkIfSum              @   result = checkIfSum(arrayAddress, i, button_count)
  CMP     R0, #1                  @   if (!result) 
  BNE     End_Main                @       break
  BL      displayOutcome          @   displayOutcome(result)
  ADD     R11, R11, #1            
  B       .LFor2
.LendFor2:                        @ }
  
End_Main:
  CMP     R0, #1                  @ if (!result)
  BNE     notWon                  @     branch(notWon)
  MOV     R0, #2                  @ else
notWon:                           @     result = endResult
  BL      displayOutcome          @ displayOutcome(result)
  POP     {R10-R12,PC}
@ setLed sets the pattern of leds called in array
@ inputs, R0 = start address of array, R1 = current length of sequence 
setLED:                           
  PUSH    {R4-R8, LR}
  MOV     R7, R0
  MOV     R8, R1
  MOV     R6, #0
.LledLoop:
  CMP     R6, R8                  @while (currentLed != finalLEd )
  BEQ     .LdoneLed
  LDR     R5, [R7, R6 , LSL #2]   @Load currentLedOutput
  CMP     R5, #1                  @Switch currentLedOutput
  BEQ     .Lpin1                  @Case Pin1 B Pin1()
  CMP     R5, #2
  BEQ     .Lpin2                  @Case Pin2 B Pin2()
  CMP     R5, #3
  BEQ     .Lpin3                  @Case Pin3 B Pin3()
  CMP     R5, #4
  BEQ     .Lpin4                  @Case Pin4 B Pin4()
.LnextLoop:
  ADD     R6, R6, #1              @ currentLed = currentLed + 1
  B       .LledLoop
.Lpin1:
  LDR     R4, =GPIOE_ODR                      @Turn on Led1
  LDR     R5, [R4] @ Read ...
  ORR     R5, #(0b1<<(LD3_PIN)) @ Modify ...
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD                   @Pause a second
  BL      delay_ms
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  AND     R5, #(0b0<<(LD3_PIN)) @ Modify ...  @Turn off Led1
  STR     R5, [R4] @ Write
  B       .LnextLoop
.Lpin2:
  LDR     R4, =GPIOE_ODR                      @Turn on Led2
  LDR     R5, [R4] @ Read ...
  ORR     R5, #(0b1<<(LD5_PIN)) @ Modify ...
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD                   @Pause a second
  BL      delay_ms
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  AND     R5, #(0b0<<(LD5_PIN)) @ Modify ...
  STR     R5, [R4] @ Write                    @Turn off Led2
  B       .LnextLoop
.Lpin3:
  LDR     R4, =GPIOE_ODR                      @Turn on Led3
  LDR     R5, [R4] @ Read ...
  ORR     R5, #(0b1<<(LD7_PIN)) @ Modify ...
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD                   @Pause a second
  BL      delay_ms
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  AND     R5, #(0b0<<(LD7_PIN)) @ Modify ...
  STR     R5, [R4] @ Write                      @Turn off Led3
  B       .LnextLoop
.Lpin4:
  LDR     R4, =GPIOE_ODR                      @Turn on Led4
  LDR     R5, [R4] @ Read ...
  ORR     R5, #(0b1<<(LD9_PIN)) @ Modify ...
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD                   @Pause a second
  BL      delay_ms
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  AND     R5, #(0b0<<(LD9_PIN)) @ Modify ...      
  STR     R5, [R4] @ Write                    @Turn off Led4
  B       .LnextLoop
.LdoneLed:
  POP     {R4-R8, PC}



  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb
  
  .global pollButtonPresses
  .extern button_count

@ checkIfSum subroutine
@ inputs: R0 start address of array, R1 length of array, R2 number of button presses
@ return value: R0 1 if correct 0 if wrong
checkIfSum:
  PUSH    {R4-R9, LR}
  MOV     R4, R0                  
  MOV     R5, R1
  MOV     R6, R2
  MOV     R7, #0                @ arraySum = 0
  MOV     R8, #0              
.LFor1:                         @ for (i = 0, i < arrayLength, i++) {
  CMP     R7, R5
  BHS     .LendFor1   
  LDR     R9, [R4, R7, LSL #2]  
  ADD     R8, R8, R9            @     arraySum += array[i]
  ADD     R7, R7, #1
  B       .LFor1
.LendFor1:                      @ }
  CMP     R8, R6                @ if (button_count != arraySum)
  BEQ     .LCorrect             @     return false
  MOV     R0, #0                
  B       .LFinishCheckIfSum    
.LCorrect:                      @ else
  MOV     R0, #1                @     return true
.LFinishCheckIfSum:
  POP     {R4-R9, PC}



displayOutcome:
  PUSH    {R4-R9, LR} 
  MOV     R9, R0
  CMP     R0, #0                    @this outcome should be in an x shape
  BNE     .LgameNotLost
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  ORR     R5, #0b101010100000000
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms
  LDR     R5, [R4] @ Read ...
  MOV     R8, #0b010101011111111
  AND     R5, R8
  STR     R5, [R4] @ Write
  B       .LdoneOutcome

.LgameNotLost:
  CMP     R0, #1                    @this outcome should make all lights light up
  BNE     .LfullGameWon             @for when single round is guessed
  LDR     R4, =GPIOE_ODR
  LDR     R5, [R4] @ Read ...
  LDR     R8, =0b1111111100000000
  ORR     R5, R8
  STR     R5, [R4] @ Write
  LDR     R0, =BLINK_PERIOD
  BL      delay_ms
  LDR     R5, [R4] @ Read ...
  LDR     R8, =0b0000000011111111
  AND     R5, R8
  STR     R5, [R4] @ Write
  B       .LdoneOutcome

.LfullGameWon:
  MOV     R6, #(0b1<<(LD4_PIN))     @this outcome should make all lights light up one by one
  MOV     R7, #0
  LDR     R4, =GPIOE_ODR
.LwhFullGameWon:
  CMP     R7, #8                   
  BEQ     .LeWhFullGameWon
  LDR     R5, [R4]
  ORR     R5, R6
  LSL     R6, #1
  STR     R5, [R4]
  LDR     R0, =1000
  BL      delay_ms
  ADD     R7, #1
  B       .LwhFullGameWon   
.LeWhFullGameWon:
  LDR     R5, [R4] @ Read ...
  AND     R5, #0b000000001111111
  STR     R5, [R4] @ Write  
.LdoneOutcome:
  MOV     R0, R9
  POP     {R4-R9, PC}

delay_ms:
  PUSH    {R4-R5,LR}
  
  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
    
  LDR     R4, =SYSTICK_LOAD           @ Set SysTick LOAD for 1ms delay
  LDR     R5, =7999                   @ Assuming a 8MHz clock
  STR     R5, [R4]                    @ 
    
  LDR     R4, =SYSTICK_VAL            @ Reset SysTick internal counter to 0
  LDR     R5, =0x1                    @   by writing any value
  STR     R5, [R4]  
  
  LDR     R4, =SYSTICK_CSR            @ Start SysTick timer by setting CSR to 0x5
  LDR     R5, =0x5                    @   set CLKSOURCE (bit 2) to system clock (1)
  STR     R5, [R4]                    @   set ENABLE (bit 0) to 1

.LwhDelay:                          @ while (delay != 0) {
  CMP     R0, #0  
  BEQ     .LendwhDelay  
    
.Lwait:  
  LDR     R5, [R4]                    @   Repeatedly load the CSR and check bit 16
  AND     R5, #0x10000                @   Loop until bit 16 is 1, indicating that
  CMP     R5, #0                      @     the SysTick internal counter has counted
  BEQ     .Lwait                      @     from 0x3E7F down to 0 and 1ms has elapsed 
  
  SUB     R0, R0, #1                  @   delay = delay - 1
  B       .LwhDelay                   @ }

.LendwhDelay:

  LDR     R4, =SYSTICK_CSR            @ Stop SysTick timer
  LDR     R5, =0                      @   by writing 0 to CSR
  STR     R5, [R4]                    @   CSR is the Control and Status Register
  
  POP     {R4-R5,PC}


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
  LDR     R4, =button_count             @ count = 0;
  MOV     R5, #0                        @
  STR     R5, [R4]                      @

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

End_Main1:
  POP     {R4-R5,PC}



@
@ SysTick interrupt handler (blink LED LD3)
@
  .type  SysTick_Handler, %function
SysTick_Handler:

  PUSH    {R4, R5, LR}
  
  LDR     R4, =blink_countdown        @ if (countdown != 0) {
  LDR     R5, [R4]                    @
  CMP     R5, #0                      @
  BEQ     .LelseFire                  @
  
  SUB     R5, R5, #1                  @   countdown = countdown - 1;
  STR     R5, [R4]                    @
  
  B       .LendIfDelay                @ }

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

  PUSH    {R4,R5,LR}
  
  LDR     R4, =button_count           @ count = count + 1
  LDR     R5, [R4]                    @
  ADD     R5, R5, #1                  @
  STR     R5, [R4]                    @
  
  LDR     R4, =EXTI_PR                @ Clear (acknowledge) the interrupt
  MOV     R5, #(1<<0)                 @
  STR     R5, [R4]                    @

  @ Return from interrupt handler
  POP    {R4,R5,PC}


  .section .data
  
button_count:
  .space  4

blink_countdown:
  .space  4
patternArray:
.word 4, 2, 1, 3
.equ    patternArrayLen, (.-patternArray)/4

  .end