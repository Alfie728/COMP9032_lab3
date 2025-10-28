.include "m2560def.inc"

.def temp1 = r2
.def temp2 = r3
.def temp3 = r4
.def temp4 = r5
.def temp5 = r6
.def temp6 = r7
.def temp7 = r16
.def temp8 = r17
.def aval = r8
.def bval = r9
.def cval = r10
.def input_state = r11
.def input_ready = r12
.def colmask = r19  ; column drive mask
.def col = r21
.def row = r22
.def data = r23
.def llhs = r24
.def hlhs = r25


.equ PORTLDIR = 0xF0 ; PE7-4: output, PE3-0, input
.equ PORTFDIR = 0xFF
.equ ROWMASK =0x0F
.equ INIT_COL_MASK = 0xEF
.equ INIT_ROW_MASK = 0x01
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

.macro power2
 mul hlhs, hlhs
 mov temp4, r0

 mov temp7, hlhs
 lsl temp7
 mul temp7, llhs
 mov temp5, r0
 add temp4, r1

 mul llhs, llhs
 mov temp6, r0
 add temp5, r1
 ldi temp7, 0
 adc temp4, temp7
.endmacro

; 1 mu second delay
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


start:
port_setting:
 ldi temp7, PORTFDIR
 out DDRF, temp7    ; set port F as output
 out DDRA, temp7    ; set port A as output
 ldi temp7, PORTLDIR
 sts DDRL, temp7    ; set PL7-4: output, PE3-0, input
 ser temp7
 out DDRC, temp7    ; set PC as output

open_lcd:

 ldi temp7, low(15000)
 ldi temp8, high(15000)
 delay

 ldi data, LCD_FUNC_SET | (1 << LCD_N)
 lcd_write_com

 ldi temp7, low(4100)
 ldi temp8, high(4100)
 delay

 lcd_write_com

 ldi temp7, low(100)
 ldi temp8, high(100)
 delay

 lcd_write_com
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_DISPLAY | (0 << LCD_D)
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_CLEAR
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_ENTRY_SET | (1 << LCD_ID) ;| (1 << LCD_S)
 lcd_write_com

 lcd_wait_busy
 ldi data, LCD_DISPLAY | (1 << LCD_D) | (1 << LCD_C)
 lcd_write_com


 clr aval
 clr bval
 clr cval
 clr input_state
 clr input_ready

KeyPad_loop:
 mov temp7, input_ready
 cpi temp7, 0
 breq KeyPad
 rcall display_values
 rjmp KeyPad_loop

KeyPad:
 ldi colmask, INIT_COL_MASK
 clr col
col_loop:
 cpi col, 4
 breq KeyPad
 sts PORTL, colmask

 ldi temp7, low(35000)
 ldi temp8, high(35000)
 delay

check_row:
 lds temp7, PINL
 andi temp7, ROWMASK
 cpi temp7, 0x0F
 breq next_col

 clr row

row_loop:
 sbrs temp7, 0
 rjmp convert
 inc row
 lsr temp7
 rjmp row_loop

next_col:
 lsl colmask
 inc col
 rjmp col_loop

convert:
 cpi col, 3
 breq letter

 cpi row, 3
 breq symbols

 mov data, row
 lsl data
 add data, row
 add data, col
 subi data, -49
 rjmp convert_end
letter:
 ldi data, 'A'
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
 ldi data, '*'
 rjmp convert_end
zero:
 ldi data, '0'

convert_end:

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
 rjmp key_ignore

key_hash:
 mov temp7, input_state
 cpi temp7, 3
 brge key_display_trampoline
 inc input_state
 mov temp7, input_state
 cpi temp7, 3
 brne key_display_trampoline
 ldi temp7, 1
 mov input_ready, temp7
 rjmp key_display

key_display_trampoline:
 rjmp key_display

key_star:
 clr aval
 clr bval
 clr cval
 clr input_state
 clr input_ready

 lcd_wait_busy
 ldi data, LCD_CLEAR
 lcd_write_com
 lcd_wait_busy
 rjmp key_post_delay

key_digit:
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
 lcd_wait_busy
 lcd_write_data
 rjmp key_post_delay

key_ignore:
 rjmp key_post_delay

key_post_delay:
 ldi temp7, low(55000)
 ldi temp8, high(55000)
 delay

 rjmp KeyPad_loop

display_values:
 push temp1
 push temp2
 push temp3
 push temp7
 push temp8
 push data

 lcd_wait_busy
 ldi data, LCD_CLEAR
 lcd_write_com
 lcd_wait_busy

 ldi data, LCD_SET_ADD | 0x00
 lcd_write_com

 ldi data, '('
 rcall lcd_put_char

 mov temp1, aval
 rcall print_decimal

 ldi data, ','
 rcall lcd_put_char
 ldi data, ' '
 rcall lcd_put_char

 mov temp1, bval
 rcall print_decimal

 ldi data, ','
 rcall lcd_put_char
 ldi data, ' '
 rcall lcd_put_char

 mov temp1, cval
 rcall print_decimal

 ldi data, ')'
 rcall lcd_put_char

 ldi data, LCD_SET_ADD | 0x40
 lcd_write_com

 rcall evaluate_result_char
 rcall lcd_put_char

 clr input_ready

 pop data
 pop temp8
 pop temp7
 pop temp3
 pop temp2
 pop temp1
 ret

print_decimal:
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
 cp temp2, temp7
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
 cp temp2, temp7
 brlo pd_tens_done
 sub temp2, temp7
 inc temp3
 rjmp pd_tens_loop
pd_tens_done:
 tst temp3
 brne pd_tens_nonzero
 tst temp5
 breq pd_ones
 ldi data, '0'
 rcall lcd_put_char
 rjmp pd_ones
pd_tens_nonzero:
 ldi data, '0'
 add data, temp3
 rcall lcd_put_char
 ldi temp7, 1
 mov temp5, temp7
pd_ones:
 ldi data, '0'
 add data, temp2
 rcall lcd_put_char

 pop temp8
 pop temp7
 pop temp5
 pop temp4
 pop temp3
 pop temp2
 ret

lcd_put_char:
 push temp7
 push temp8
 lcd_wait_busy
 lcd_write_data
 pop temp8
 pop temp7
 ret

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

 clr r1
 mov temp1, aval
 mov temp2, aval
 mul temp1, temp2
 mov llhs, r0
 mov hlhs, r1

 clr r1
 mov temp1, bval
 mov temp2, bval
 mul temp1, temp2
 mov temp3, r0
 mov temp4, r1

 add llhs, temp3
 adc hlhs, temp4

 clr r1
 mov temp1, cval
 mov temp2, cval
 mul temp1, temp2
 mov temp5, r0
 mov temp6, r1

 cp hlhs, temp6
 cpc llhs, temp5
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

blink_state_led:
 push temp7
 push temp8
 ldi temp7, 0xFF
 out PORTC, temp7
 ldi temp7, low(10000)
 ldi temp8, high(10000)
 delay
 clr temp7
 out PORTC, temp7
 pop temp8
 pop temp7
 ret

end:
 rjmp end
