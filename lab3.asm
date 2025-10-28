.include "m2560def.inc"

.def temp1 = r2      ; scratch: general arithmetic
.def temp2 = r3      ; scratch / digit accumulator
.def temp3 = r4      ; scratch / decimal counters
.def temp4 = r5      ; scratch / multiplier helper
.def temp5 = r6      ; scratch / decimal printed flag
.def temp6 = r7      ; scratch / multiplication high byte
.def temp7 = r16     ; high scratch (CPI/LDI friendly)
.def temp8 = r17     ; high scratch (delay counter high)
.def aval = r8       ; input value a (0-255)
.def bval = r9       ; input value b (0-255)
.def cval = r10      ; input value c (0-255, practical 0-99)
.def input_state = r11 ; which value we are entering (0=a,1=b,2=c)
.def input_ready = r12 ; flag set once all three inputs collected
.def colmask = r19   ; column drive mask for keypad
.def col = r21       ; column index / reused as LED loop counter
.def row = r22       ; active keypad row
.def data = r23      ; LCD data bus value
.def llhs = r24      ; low byte of (a^2 + b^2)
.def hlhs = r25      ; high byte of (a^2 + b^2)


.equ PORTLDIR = 0xF0 ; PE7-4: output, PE3-0, input
.equ PORTFDIR = 0xFF
.equ ROWMASK =0x0F           ; mask for keypad row bits
.equ INIT_COL_MASK = 0xEF    ; initial column drive pattern
.equ INIT_ROW_MASK = 0x01    ; starting row bit when scanning
.equ LED_FLASH_DELAY = 40000 ; on/off hold time for LED flashes
.equ LED_FLASH_COUNT = 3     ; number of LED pulses for true result
.equ LCD_RS = 7
.equ LCD_E = 6
.equ LCD_RW = 5
.equ LCD_N = 3
.equ LCD_ID = 1
.equ LCD_S = 0
.equ LCD_D = 2
.equ LCD_C = 1
.equ LCD_B = 0
.equ LCD_BF = 7
.equ LCD_FUNC_SET = 0x30
.equ LCD_ENTRY_SET = 0x04
.equ LCD_SET_ADD = 0x80
.equ LCD_DISPLAY = 0x08
.equ LCD_CLEAR = 0x01
.equ LCD_RETURN = 0x02

; 1 mu second delay
; Simple software delay: temp7:temp8 carry the iteration count.
.macro delay
loop1:
 ldi r18, 0x3
loop2:
 dec r18
 nop
 brne loop2

 subi temp7, 1
 sbci temp8, 0
 brne loop1
.endmacro

; --- LCD helper macros ---
.macro lcd_write_com
 out PORTF, data ; set the data port's value up
 ldi temp7, (0 << LCD_RS)|(0 << LCD_RW)
 out PORTA, temp7 ; RS = 0, RW = 0 for a command write

 ldi temp7, low(100)
 ldi temp8, high(100)
 delay

 sbi PORTA, LCD_E ; turn on the enable pin

 ldi temp7, low(300)
 ldi temp8, high(300)
 delay

 cbi PORTA, LCD_E ; turn off the enable pin

 ldi temp7, low(300)
 ldi temp8, high(300)
 delay

.endmacro

.macro lcd_write_data
 out PORTF, data ; set the data port's value up
 ldi temp7, (1 << LCD_RS)|(0 << LCD_RW)
 out PORTA, temp7 ; RS = 1, RW = 0 for a data write

 ldi temp7, low(100)
 ldi temp8, high(100)
 delay

 sbi PORTA, LCD_E ; turn on the enable pin

 ldi temp7, low(300)
 ldi temp8, high(300)
 delay

 cbi PORTA, LCD_E ; turn off the enable pin

 ldi temp7, low(300)
 ldi temp8, high(300)
 delay
.endmacro

.macro lcd_wait_busy
 clr temp7
 out DDRF, temp7 ; Make port F as an input port for now
 ldi temp7, (0 << LCD_RS)|(1 << LCD_RW)
 out PORTA, temp7 ; RS = 0, RW = 1 for a command port read
  busy_loop:

 ldi temp7, low(100)
 ldi temp8, high(100)
 delay

 sbi PORTA, LCD_E ; turn on the enable pin

 ldi temp7, low(300)
 ldi temp8, high(300)
 delay

 in temp7, PINF ; read value from LCD
 cbi PORTA, LCD_E ; turn off the enable pin
 sbrc temp7, LCD_BF ; if the busy flag is set
 rjmp busy_loop ; repeat command read
 clr temp7 ; else
 out PORTA, temp7 ; turn off read mode,
 ser temp7 ;
 out DDRF, temp7 ; make port F an output port again
.endmacro


; ------------------------------------------------------------
; Core application
; ------------------------------------------------------------

start:
port_setting:
 ldi temp7, PORTFDIR          ; PF drives LCD data
 out DDRF, temp7              ; configure PORTF as output
 out DDRA, temp7              ; use PORTA for LCD control pins
 ldi temp7, PORTLDIR
 sts DDRL, temp7              ; PL lines drive keypad columns
 ser temp7
 out DDRC, temp7              ; PORTC drives LED bar

open_lcd:

 ldi temp7, low(15000)        ; initial LCD power-on waits
 ldi temp8, high(15000)
 delay

 ldi data, LCD_FUNC_SET | (1 << LCD_N) ; function set sequence
 lcd_write_com

 ldi temp7, low(4100)
 ldi temp8, high(4100)
 delay

 lcd_write_com                        ; repeat command per datasheet

 ldi temp7, low(100)
 ldi temp8, high(100)
 delay

 lcd_write_com                        ; third function-set pulse
 lcd_write_com                        ; final confirmation

 lcd_wait_busy
 ldi data, LCD_DISPLAY | (0 << LCD_D) ; display off during setup
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_CLEAR                  ; clear display RAM
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_ENTRY_SET | (1 << LCD_ID) ; increment mode
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_DISPLAY | (1 << LCD_D) | (1 << LCD_C) ; display+cursor on
 lcd_write_com


 clr aval            ; reset stored inputs
 clr bval
 clr cval
 clr input_state     ; expect to capture aval first
 clr input_ready     ; nothing ready yet

; --- Main keypad scan loop ---
KeyPad_loop:
 mov temp7, input_ready        ; once final '#' hit, display the result
 cpi temp7, 0
 breq KeyPad
 rcall display_values
 rjmp KeyPad_loop              ; restart scan after display

KeyPad:
 ldi colmask, INIT_COL_MASK    ; start driving first column low
 clr col
col_loop:
 cpi col, 4
 breq KeyPad
 sts PORTL, colmask           ; drive current column low, others high

 ldi temp7, low(35000)        ; allow signals to settle
 ldi temp8, high(35000)
 delay

check_row:
 lds temp7, PINL                ; read row inputs
 andi temp7, ROWMASK            ; mask to lower nibble
 cpi temp7, 0x0F
 breq next_col

 clr row

row_loop:
 sbrs temp7, 0
 rjmp convert
 inc row                     ; move to next row bit
 lsr temp7
 rjmp row_loop

next_col:
 lsl colmask                  ; move active column left
 inc col
 rjmp col_loop

convert:
 ; Translate the active row/column into an ASCII key value.
 cpi col, 3
 breq letter

 cpi row, 3
 breq symbols

 mov data, row                 ; convert 3x4 keypad index into ASCII
 lsl data
 add data, row
 add data, col
 subi data, -49
rjmp convert_end
letter:
 ldi data, 'A'               ; columns 0-2, rows 0-3 map to A-D
 add data, row
 rjmp convert_end

symbols:
 cpi col, 0
 breq star
 cpi col, 1
 breq zero
 ldi data, '#'
 rjmp convert_end
star:
 ldi data, '*'               ; (row 3, col 0)
 rjmp convert_end
zero:
 ldi data, '0'               ; (row 3, col 1)

convert_end:

 ; Route special keys vs digits.
 cpi data, '#'
 breq key_hash
 cpi data, '*'
 breq key_star
 cpi data, '0'
 brlt key_ignore_trampoline
 cpi data, ':'
 brge key_ignore_trampoline
 rjmp key_digit

key_ignore_trampoline:
 ; Keep short branches close; jump to shared ignore handler.
 rjmp key_ignore

key_hash:
 ; Advance to the next input field and mark ready after c.
 mov temp7, input_state
 cpi temp7, 3
 brge key_display_trampoline
 inc input_state              ; advance to b or c entry
 mov temp7, input_state
 cpi temp7, 3
 brne key_display_trampoline
 ldi temp7, 1
 mov input_ready, temp7       ; finished collecting all inputs
 rjmp key_display

key_display_trampoline:
 ; ensure short branches can reach key_display
 rjmp key_display

key_star:
 ; Reset all state when '*' is pressed.
 clr aval
 clr bval
 clr cval
 clr input_state
 clr input_ready

 lcd_wait_busy
 ldi data, LCD_CLEAR          ; wipe LCD while resetting
 lcd_write_com
 lcd_wait_busy
 rjmp key_post_delay

key_digit:
 ; Route digits into aval, bval, or cval based on input_state.
 mov temp1, data
 ldi temp7, '0'
 sub temp1, temp7
 mov temp7, input_state
 cpi temp7, 0
 breq key_digit_a
 cpi temp7, 1
 breq key_digit_b
 cpi temp7, 2
 breq key_digit_c
 rjmp key_ignore

key_digit_a:
 ; aval = aval * 10 + digit
 mov temp2, aval
 ldi temp7, 10
 mov temp4, temp7
 mul temp2, temp4
 mov temp2, r0
 clr r1
 add temp2, temp1
 mov aval, temp2
 rjmp key_display

key_digit_b:
 ; bval = bval * 10 + digit
 mov temp2, bval
 ldi temp7, 10
 mov temp4, temp7
 mul temp2, temp4
 mov temp2, r0
 clr r1
 add temp2, temp1
 mov bval, temp2
 rjmp key_display

key_digit_c:
 ; cval = cval * 10 + digit
 mov temp2, cval
 ldi temp7, 10
 mov temp4, temp7
 mul temp2, temp4
 mov temp2, r0
 clr r1
 add temp2, temp1
 mov cval, temp2
 rjmp key_display

key_display:
 lcd_wait_busy                ; echo pressed key to LCD
 lcd_write_data
 rjmp key_post_delay

key_ignore:
 ; Invalid key for current state, just debounce.
 rjmp key_post_delay

key_post_delay:
 ldi temp7, low(55000)        ; debounce delay
 ldi temp8, high(55000)
 delay

 rjmp KeyPad_loop

display_values:
 ; Render "(a, b, c)" on line 1 and the inequality result on line 2.
 push temp1
 push temp2
 push temp3
 push temp7
 push temp8
 push data

 lcd_wait_busy
 ldi data, LCD_CLEAR           ; blank previous contents
 lcd_write_com
 lcd_wait_busy

 ldi data, LCD_SET_ADD | 0x00  ; move cursor to line 1 column 0
 lcd_write_com

 ldi data, '('
 rcall lcd_put_char

 mov temp1, aval               ; print a
 rcall print_decimal

 ldi data, ','
 rcall lcd_put_char
 ldi data, ' '
 rcall lcd_put_char

 mov temp1, bval               ; print b
 rcall print_decimal

 ldi data, ','
 rcall lcd_put_char
 ldi data, ' '
 rcall lcd_put_char

 mov temp1, cval               ; print c
 rcall print_decimal

 ldi data, ')'
 rcall lcd_put_char

 ldi data, LCD_SET_ADD | 0x40  ; DDRAM offset for line 2
 lcd_write_com

 rcall evaluate_result_char    ; data <- 'T' or 'F'
 mov temp7, data               ; keep copy for LED logic
 rcall lcd_put_char
 clr input_ready               ; allow keypad loop to resume input entry
 cpi temp7, 'T'
 brne display_values_skip_flash
 rcall flash_led_true
display_values_skip_flash:      ; skip flashing when inequality is false

 pop data
 pop temp8
 pop temp7
 pop temp3
 pop temp2
 pop temp1
 ret

print_decimal:
 ; Output decimal representation of value in temp1 (0-255) without leading zeros.
 push temp2
 push temp3
 push temp4
 push temp5
 push temp7
 push temp8

 clr temp5
 mov temp2, temp1

 ldi temp7, 100
 clr temp3
pd_hund_loop:
 cp temp2, temp7              ; subtract 100 until value < 100
 brlo pd_hund_done
 sub temp2, temp7
 inc temp3
 rjmp pd_hund_loop
pd_hund_done:
 tst temp3
 breq pd_tens_prep
 ldi data, '0'
 add data, temp3
 rcall lcd_put_char
 ldi temp7, 1
 mov temp5, temp7
pd_tens_prep:
 ldi temp7, 10
 clr temp3
pd_tens_loop:
 cp temp2, temp7              ; subtract 10 until value < 10
 brlo pd_tens_done
 sub temp2, temp7
 inc temp3
 rjmp pd_tens_loop
pd_tens_done:
 tst temp3
 brne pd_tens_nonzero
 tst temp5
 breq pd_ones
 ldi data, '0'                ; we already printed hundreds, keep leading zero
 rcall lcd_put_char
 rjmp pd_ones
pd_tens_nonzero:
 ldi data, '0'
 add data, temp3              ; emit tens digit when non-zero
 rcall lcd_put_char
 ldi temp7, 1
 mov temp5, temp7
pd_ones:
 ldi data, '0'
 add data, temp2              ; emit ones digit (always printed)
 rcall lcd_put_char

 pop temp8
 pop temp7
 pop temp5
 pop temp4
 pop temp3
 pop temp2
 ret

lcd_put_char:
 ; Blocking write of `data` to the LCD data register.
 push temp7
 push temp8
 lcd_wait_busy
 lcd_write_data
 pop temp8
 pop temp7
 ret

; Decide whether a^2 + b^2 > c^2 and return 'T' or 'F' in data.
evaluate_result_char:
 push temp1
 push temp2
 push temp3
 push temp4
 push temp5
 push temp6
 push temp7
 push temp8
 push llhs
 push hlhs

 clr r1                     ; clear mul high byte per AVR ABI
 mov temp1, aval            ; temp1:temp2 <- a
 mov temp2, aval
 mul temp1, temp2           ; r1:r0 = a^2
 mov llhs, r0               ; store low byte of sum
 mov hlhs, r1               ; store high byte of sum

 clr r1
 mov temp1, bval            ; temp1:temp2 <- b
 mov temp2, bval
 mul temp1, temp2           ; r1:r0 = b^2
 mov temp3, r0
 mov temp4, r1

 add llhs, temp3            ; accumulate low bytes
 adc hlhs, temp4            ; propagate carry to high byte

 clr r1
 mov temp1, cval            ; temp1:temp2 <- c
 mov temp2, cval
 mul temp1, temp2           ; r1:r0 = c^2
 mov temp5, r0
 mov temp6, r1

 cp hlhs, temp6             ; compare high bytes
 cpc llhs, temp5            ; compare low bytes with carry
 brlo eval_false
 breq eval_false

 ldi data, 'T'
 rjmp eval_done

eval_false:
 ldi data, 'F'

eval_done:
 clr r1
 pop hlhs
 pop llhs
 pop temp8
 pop temp7
 pop temp6
 pop temp5
 pop temp4
 pop temp3
 pop temp2
 pop temp1
 ret

flash_led_true:
 ; Reuse the high register `col` as a temporary loop counter to flash LEDs.
 push col
 push temp7
 push temp8
 ldi col, LED_FLASH_COUNT        ; number of on/off blinks to execute
flash_led_loop:
 ldi temp7, 0xFF                 ; turn LEDs fully on
 out PORTC, temp7
 ldi temp7, low(LED_FLASH_DELAY) ; hold the on state
 ldi temp8, high(LED_FLASH_DELAY)
 delay
 clr temp7
 out PORTC, temp7                ; turn LEDs off
 ldi temp7, low(LED_FLASH_DELAY) ; hold the off state
 ldi temp8, high(LED_FLASH_DELAY)
 delay
 dec col                       ; countdown remaining flashes
 brne flash_led_loop
 pop temp8
 pop temp7
 pop col
 ret

end:
 rjmp end                      ; stay here forever
