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
.def bval = 9
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
 brlt key_ignore
 cpi data, ':'
 brge key_ignore
 rjmp key_digit

key_hash:
 cpi input_state, 3
 brge key_display
 inc input_state
 cpi input_state, 3
 brne key_display
 ldi input_ready, 1
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
 subi temp1, '0'
 cpi input_state, 0
 breq key_digit_a
 cpi input_state, 1
 breq key_digit_b
 cpi input_state, 2
 breq key_digit_c
 rjmp key_ignore

key_digit_a:
 mov temp2, aval
 ldi temp4, 10
 mul temp2, temp4
 mov temp2, r0
 clr r1
 add temp2, temp1
 mov aval, temp2
 rjmp key_display

key_digit_b:
 mov temp2, bval
 ldi temp4, 10
 mul temp2, temp4
 mov temp2, r0
 clr r1
 add temp2, temp1
 mov bval, temp2
 rjmp key_display

key_digit_c:
 mov temp2, cval
 ldi temp4, 10
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

end:
 rjmp end
