; Global vars.
; Use the following command to determine the correct dimensions: cat /sys/class/graphics/fb0/virtual_size.
SCREEN_X equ 1920
SCREEN_Y equ 1200
COLOUR_DEPTH equ 4 ; In bytes.
FB_SIZE equ SCREEN_X * SCREEN_Y * COLOUR_DEPTH ; In bytes.

; Syscall macros.
SYS_EXIT equ 60
SYS_READ equ 0
SYS_WRITE equ 1
SYS_OPEN equ 2
SYS_LSEEK equ 8
SYS_CLOSE equ 3
SYS_FCNTL equ 72
SYS_GETTIMEOFDAY equ 96

F_GETFL equ 3
F_SETFL equ 4

; Misc.
O_RDONLY equ 0
O_WRONLY equ 1
O_RDWR equ 2
O_NONBLOCK equ 0x800

; Keycodes (case insensitive)
KEY_CODE_Q equ 16
KEY_CODE_LEFT_ARROW equ 105
KEY_CODE_RIGHT_ARROW equ 106

section .data
    fb_device db "/dev/fb0", 0
	kb_device db "/dev/input/event3", 0 ; 3 for inbuilt, 22 for wireless.

	; Can be thought of as a cursor, refers to the pixel in the fb that is currently in focus.
	x dd 0
	y dd 0

    tiny dd 0x30000000 ; (0.5)^31 
    rand_offset_lower dd 0.025 ; The offset for when a collision happens. 
    rand_offset_upper dd 0.070
    rand_offset_sign dd 1.0

	screen_center_x dd 0
	screen_center_y dd 0
    
    circle_center_x dd 0
    circle_center_y dd 0
    circle_radius dd 0
    circle_colour dd 0x000000

	ball_pos_x dd 0.0
	ball_pos_y dd 0.0
	ball_vel_x dd 0.0
	ball_vel_y dd -2.5 
	ball_radius dd 10
    ball_colour dd 0xffffff

	bg_col dd 0xFF0000 ; Red.
    
    ; Common decimals for convinience.
    zero dd 0.0
    one dd 1.0
    minus_one dd -1.0

	boundary_radius dd 250
	boundary_thickness dd 3
    boundary_colour dd 0x000000 ; Black

    theta dd 0.0 ; Angle of paddle rotation.
	theta_offset dd 0.02 ; Baisically controls speed at which paddle turns.
	pi dd 3.14159265
	cos_pi_over_12 dd 0.9659258 ; cos(pi/12) roughly. Used to determine the width of the paddle.

	paddle_center_x dd 0
	paddle_center_y dd 0

    key_left_arrow_active dd 0
    key_right_arrow_active dd 0 
    
    delta_t dd 8333 ; 120 fps in microseconds. 
    
section .bss
    fb_fd resd 1
	kb_fd resd 1
	staging_buffer resb FB_SIZE ; Stores the framebuffer contents.
	kb_buffer resb 24 ; Buffer to hold integer string representation.
    time_interval_start resq 2 ; First 64 bits store time in microseconds. Require 2 x 64 bits to store the syscall response.
    time_interval_end resq 2 ; First 64 bits store time in microseconds.

section .text
    global _start

_start:
    ; Open framebuffer device.
    mov rax, SYS_OPEN              
    mov rdi, fb_device      
    mov rsi, O_RDWR              
    xor rdx, rdx ; Mode (not used).
    syscall                 
    mov dword [fb_fd], eax ; Store file descriptor.

	; Open keyboard device.
	mov rax, SYS_OPEN
	mov rdi, kb_device
	mov rsi, O_RDONLY | O_NONBLOCK
	syscall
	mov dword [kb_fd], eax ; Store file descriptor. 

    ; Init time
    call _getTime
    mov rax, qword [time_interval_end]
    mov qword [time_interval_start], rax
    
	call _computeScreenCenter
	jmp _mainLoop   
    
_mainLoop:
    ; Fixed time loop.
    call _getTime
    mov rax, qword [time_interval_end]
    sub rax, qword [time_interval_start]
    cmp eax, dword [delta_t]
    jl _mainLoop
    mov rax, qword [time_interval_end]
    mov qword [time_interval_start], rax

	; Update screen.
    call _clearScreen
    call _drawCircleBoundaryWithPaddle
    call _drawBall 
	call _flushScreenBuffer
	
    ; Read keyboard buffer.
    mov rax, SYS_READ
	mov edi, dword [kb_fd]
	mov rsi, kb_buffer
	mov rdx, 24 ; Number of bytes to read (input event size).
	syscall
	cmp rax, 24 ; Check if an event actually happened.	
	jne _mainLoop_noInput

	mov ax, word [kb_buffer + 16] ; Check for appropriate event type (1).
	cmp ax, 1
	jne _mainLoop_noInput
	call _processInput
_mainLoop_noInput:
	call _updateTheta
	call _updateBallPos	
	call _computePaddleCenter	
    call _handleCollision

	jmp _mainLoop

_getTime:
    push rax
    push rdi
    push rsi
    push rcx

    mov rax, SYS_GETTIMEOFDAY
    lea rdi, [time_interval_end]
    xor rsi, rsi
    syscall

    ; Convert seconds and microseconds to a single value in microseconds.
    mov rax, qword [time_interval_end] ; Seconds.
    mov rcx, 1000000
    imul rax, rcx ; Seconds * 1000000.
    add rax, [time_interval_end + 8] ; Add microseconds.
    mov qword [time_interval_end], rax 

    pop rcx
    pop rsi
    pop rdi
    pop rax
    ret

_processInput:	
	push rax
    push rbx

	mov eax, dword [kb_buffer + 20] ; Value (offset 20 in input_event structure).
	cmp eax, 0 ; Key release.
	je _processInput_update
	cmp eax, 1 ; Key press.
	je _processInput_update
	jmp _processInput_end
_processInput_update:
	mov bx, word [kb_buffer + 18] ; Keycode (offset 18 in input_event structure).
    cmp bx, KEY_CODE_LEFT_ARROW
    je _processInput_leftArrow
    cmp bx, KEY_CODE_RIGHT_ARROW
    je _processInput_rightArrow
    cmp bx, KEY_CODE_Q
    je _processInput_quit
    jmp _processInput_end
_processInput_leftArrow:
    mov dword [key_left_arrow_active], eax
    jmp _processInput_end 
_processInput_rightArrow:
    mov dword [key_right_arrow_active], eax
    jmp _processInput_end 
_processInput_end:
    pop rbx
	pop rax
	ret
_processInput_quit:	
	call _exit

_updateTheta:	
	push rax
	push rbp
	mov rbp, rsp

	sub rsp, 16

	movss dword [rsp + 0], xmm0
	movss dword [rsp + 4], xmm1
	movss dword [rsp + 8], xmm2

	mov eax, dword [key_left_arrow_active]	
	cmp eax, 1 ; Anti-clockwise.
	je _updateTheta_dec
    mov eax, dword [key_right_arrow_active] 
	cmp eax, 1 ; Clockwise.
	je _updateTheta_inc
	jmp _updateTheta_end
_updateTheta_inc:
	movss xmm0, dword [theta]
	movss xmm1, dword [theta_offset]
	addss xmm0, xmm1 ; Move theta. 	
	movss dword [theta], xmm0
	
	; Now do range check. If over 2pi, subtract 2pi.
	mov eax, 2
	cvtsi2ss xmm1, eax
	movss xmm2, dword [pi]
	mulss xmm2, xmm1 ; 2pi
	ucomiss xmm0, xmm2
	jb _updateTheta_end ; If within 2pi, we are fine.
	subss xmm0, xmm2 ; Subtract 2pi.
	movss dword [theta], xmm0
	jmp _updateTheta_end
_updateTheta_dec:
	movss xmm0, dword [theta]
	movss xmm1, dword [theta_offset]
	subss xmm0, xmm1 ; Move theta. 	
	movss dword [theta], xmm0
	
	; Now do range check. If less than 0, add 2pi.
	mov eax, 0
	cvtsi2ss xmm1, eax
	ucomiss xmm0, xmm1
	ja _updateTheta_end ; If greater than 0, we are fine. 
	mov eax, 2
	cvtsi2ss xmm1, eax
	movss xmm2, dword [pi]
	mulss xmm2, xmm1 ; 2pi
	addss xmm0, xmm2 ; Add 2pi.
	movss dword [theta], xmm0
	jmp _updateTheta_end
_updateTheta_end:

	movss xmm0, dword [rsp + 0]
	movss xmm1, dword [rsp + 4]
	movss xmm2, dword [rsp + 8]

	add rsp, 16

	pop rbp
	pop rax
	ret	

_updateBallPos:
	push rbp
	mov rbp, rsp

	sub rsp, 16

	movss dword [rsp + 0], xmm0
	movss dword [rsp + 44], xmm1

	; Update x.
	movss xmm0, dword [ball_pos_x]
	movss xmm1, dword [ball_vel_x]
	addss xmm0, xmm1
	movss dword [ball_pos_x], xmm0
	
	; Update y.
	movss xmm0, dword [ball_pos_y]
	movss xmm1, dword [ball_vel_y]
	addss xmm0, xmm1
	movss dword dword [ball_pos_y], xmm0

_updateBallPos_end:

	movss dword [rsp + 0], xmm0
	movss dword [rsp + 4], xmm1

	add rsp, 16

	pop rbp
	ret	    

; Approximates sine using taylor series with 3 terms.
; The approximation becomes shit after pi/2 so use symmetry.
;
; @param xmm0 - the angle in radians.
; @return xmm0 - sine of the angle.
_sine:	
	push rax
	push rbx
	sub rsp, 16
	movss dword [rsp + 0], xmm1
	movss dword [rsp + 4], xmm2
	movss dword [rsp + 8], xmm3
	movss dword [rsp + 12], xmm4

	mov ebx, 1 ; Final multiplier, used if theta greater than pi.

	movss xmm1, dword [pi]
	ucomiss xmm0, xmm1 ; Check if greater than pi.
	jb _sine_withinPi
	subss xmm0, xmm1
	mov ebx, -1
_sine_withinPi:
	mov eax, 2
	cvtsi2ss xmm2, eax
	divss xmm1, xmm2 ; pi/2.
	ucomiss xmm0, xmm1
	jb _sine_inRange	; If greater than pi/2, then do pi - theta.
	movss xmm1, dword [pi]
	subss xmm1, xmm0
	movss xmm0, xmm1
_sine_inRange:
	; Term 1: x
	movss xmm1, xmm0	

	mov eax, 6 
	cvtsi2ss xmm4, eax

	; Term 2: (x^3)/6
	movss xmm2, xmm0
	mulss xmm2, xmm2
	mulss xmm2, xmm0
	divss xmm2, xmm4

	mov eax, 120
	cvtsi2ss xmm4, eax

	; Term 3: (x^5)/120
	movss xmm3, xmm0
	mulss xmm3, xmm3
	mulss xmm3, xmm3
	mulss xmm3, xmm0
	divss xmm3, xmm4

	; Combine terms
	movss xmm0, xmm1
	subss xmm0, xmm2
	addss xmm0, xmm3
	
	; Adjust sign.
	cvtsi2ss xmm4, ebx
	mulss xmm0, xmm4

	movss xmm1, dword [rsp + 0]
	movss xmm2, dword [rsp + 4]
	movss xmm3, dword [rsp + 8]
	movss xmm4, dword [rsp + 12]
	add rsp, 16
	pop rbx
	pop rax
	ret

; Approximates cosine using taylor series with 3 terms.
; Similar story to sine, have to use symmetry after pi/2.
; Note this assumes theta is in between 0 and 2pi.
;
; @param xmm0 - the angle in radians.
; @return xmm0 - cosine of the angle.
_cosine:	
	push rax
	push rbx
	sub rsp, 16
	movss dword [rsp + 0], xmm1
	movss dword [rsp + 4], xmm2
	movss dword [rsp + 8], xmm3
	movss dword [rsp + 12], xmm4

	; Final sign multiplier (1 or -1).
	mov ebx, 1
	
	; Range manipulation magic.
	movss xmm1, dword [pi]
	mov eax, 2
	cvtsi2ss xmm2, eax
	divss xmm1, xmm2 ; pi/2.
	ucomiss xmm0, xmm1	
	jb _cosine_inRange
	mulss xmm1, xmm2 ; pi.
	ucomiss xmm0, xmm1
	jb _cosine_withinPi
	divss xmm1, xmm2 ; pi/2.
	mov eax, 3
	cvtsi2ss xmm2, eax
	mulss xmm1, xmm2 ; 3pi/2.
	ucomiss xmm0, xmm1
	jb _cosine_within3piover2
	; At this point, it is greater than 3pi/2.
	movss xmm1, dword [pi]
	mov eax, 2
	cvtsi2ss xmm2, eax
	mulss xmm1, xmm2 ; 2pi.
	subss xmm1, xmm0 ; 2pi - theta.
	movss xmm0, xmm1
	jmp _cosine_inRange	
_cosine_withinPi:
	divss xmm1, xmm2 ; pi/2.
	subss xmm1, xmm0 ; pi/2 - theta.
	mov ebx, -1
_cosine_within3piover2:
	movss xmm1, dword [pi]
	subss xmm0, xmm1
	mov ebx, -1
	jmp _cosine_inRange		
_cosine_inRange:	
	; Term 1: 1
	mov eax, 1 	
	cvtsi2ss xmm1, eax	

	mov eax, 2
	cvtsi2ss xmm4, eax	

	; Term 2: (x^2)/2
	movss xmm2, xmm0
	mulss xmm2, xmm2
	divss xmm2, xmm4

	mov eax, 24 
	cvtsi2ss xmm4, eax

	; Term 3: (x^4)/24
	movss xmm3, xmm0
	mulss xmm3, xmm3
	mulss xmm3, xmm3
	divss xmm3, xmm4

	; Combine terms
	movss xmm0, xmm1
	subss xmm0, xmm2
	addss xmm0, xmm3

	; Adjust sign.
	cvtsi2ss xmm4, ebx
	mulss xmm0, xmm4

	movss xmm1, dword [rsp + 0]
	movss xmm2, dword [rsp + 4]
	movss xmm3, dword [rsp + 8]
	movss xmm4, dword [rsp + 12]
	add rsp, 16
	pop rbx
	pop rax
	ret

; Computes distance squared between (x0,y0) and (x1,y1).	
; @param - Setup stack such that [rsp + 0] = x0, [rsp + 4] = y0, [rsp + 8] = x1, [rsp + 12] = y1.
;
; @return eax - distance.
_dist2:
	; Must move rsp since the function call itself pushes the return address onto the stack, along with the prolog.
	add rsp, 8

	; Find deltas.
	mov eax, dword [rsp + 8]
	sub dword dword [rsp + 0], eax
	mov eax, dword [rsp + 12]
	sub dword [rsp + 4], eax 

	; Square delatas.
	mov eax, dword [rsp + 0]
	imul eax, eax
	mov dword [rsp + 0], eax
	mov eax, [rsp + 4]
	imul eax, eax
	mov dword [rsp + 4], eax

	mov eax, dword [rsp + 0]
	add eax, dword [rsp + 4]

	sub rsp, 8

	ret

; Finds the distance squared between (x,y) and (screen_center_x, screen_center_y).
;
; @return eax - distance.
_distBetweenPointAndCenter2:
	push rbp
	mov rbp, rsp

	sub rsp, 16

	mov eax, [x]
	mov dword [rsp + 0], eax 
	mov eax, [y]
	mov dword [rsp + 4], eax
	mov eax, [screen_center_x]
	mov dword [rsp + 8], eax
	mov eax, [screen_center_y]
	mov dword [rsp + 12], eax

	call _dist2

	add rsp, 16

	pop rbp
	ret

; Finds the distance squared between (x,y) and (ball_pos_x, ball_pos_y).
;
; @return eax - distance.
_distBetweenPointAndBall2:
	push rbp
	mov rbp, rsp

	movss dword [rsp + 16], xmm0

	sub rsp, 32

	mov eax, dword [x]
	mov dword [rsp + 0], eax 
	mov eax, dword [y]
	mov dword [rsp + 4], eax
	movss xmm0, dword [ball_pos_x]
	cvttss2si eax, xmm0 ; Convert float to int with truncation.
	mov dword [rsp + 8], eax
	movss xmm0, dword [ball_pos_y]
	cvttss2si eax, xmm0
	mov dword [rsp + 12], eax

	call _dist2

	movss xmm0, dword [rsp + 16]

	add rsp, 32

	pop rbp
	ret


; Determines cos of the angle between (x,y), (screen_center_x, screen_center_y) and (paddle_center_x, paddle_center_y), at the center. Uses the dot prod formula.
;
; @return - result in xmm0.
_cosineOffset:
	push rax
	push rbx
	push rbp
	mov rbp, rsp

	; [rsp + 0] to [rsp + 12] used for setting up _dist2.
	; [rsp + 16] - Product of offset distance between center and point with offset between center and paddle.
	; [rsp + 20] - Dotprod of length between center and point with length between center and paddle.
	sub rsp, 32 ; 2 * 16.

	call _distBetweenPointAndCenter2	
	cvtsi2ss xmm0, rax   
	call _sqrt
	movss dword [rsp + 16], xmm0

	; Find dist between paddle and screen center.
	mov eax, dword [paddle_center_x] 
	mov dword [rsp + 0], eax 	
	mov eax, dword [paddle_center_y]
	mov dword [rsp + 4], eax
	mov eax, dword [screen_center_x]
	mov dword [rsp + 8], eax
	mov eax, dword [screen_center_y]
	mov dword [rsp + 12], eax
	call _dist2
	cvtsi2ss xmm0, rax   
	call _sqrt
	mulss xmm0, dword [rsp + 16]
	movss dword [rsp + 16], xmm0

	; Find dotprod of length between center and point with length between center and paddle.
	; Multiply x-components.
	mov eax, dword [x]
	sub eax, dword [screen_center_x]
	mov dword [rsp + 20], eax
	mov eax, dword [paddle_center_x]
	sub eax, dword [screen_center_x]
	imul eax, dword [rsp + 20]
	; Multiply y-components.
	mov ebx, dword [y]
	sub ebx, dword [screen_center_y]
	mov dword [rsp + 20], ebx
	mov ebx, dword [paddle_center_y]
	sub ebx, dword [screen_center_y]
	imul ebx, dword [rsp + 20]
	; Add components together.
	add eax, ebx	
	mov dword [rsp + 20], eax

	; Put it all together.
	cvtsi2ss xmm0, dword [rsp + 20]
	divss xmm0, dword [rsp + 16]	
		
	add rsp, 32

	pop rbp
	pop rbx
	pop rax
	ret

; Compute square root using 10 iterations of newton raphson.
; Formula for computing sqrt(a) is x_{n+1} = 0.5 * (x_n + (x_n / a)).
; 
; @param xmm0 - Number to square root. 
; @return xmm0 - Result.
_sqrt:
	push rax
	push rbx
	sub rsp, 16
	movss dword [rsp + 0], xmm1 ; a
	movss dword [rsp + 4], xmm2 ; scratch space.

	mov rax, 0
    mov ebx, 2 ; IEEE single precision for 2.0. 
	movss xmm1, xmm0 ; Store a.	
_sqrtLoop:
	inc rax
	movss xmm2, xmm1
	divss xmm2, xmm0 ; a / x_n.		
	addss xmm0, xmm2
	cvtsi2ss xmm2, ebx
	divss xmm0, xmm2 ; Divide by 2.
	
	cmp rax, 10
	jl _sqrtLoop
	
	movss xmm1, dword [rsp + 0]
	movss xmm2, dword [rsp + 4]
	add rsp, 16
	pop rbx
	pop rax
	ret

_computeScreenCenter:
	push rax
	push rbp
	mov rbp, rsp
	
	sub rsp, 16
	
	movss dword [rsp + 0], xmm0

	mov rax, SCREEN_X
	shr rax, 1
	mov dword [screen_center_x], eax
	cvtsi2ss xmm0, eax
	movss dword [ball_pos_x], xmm0 
	mov rax, SCREEN_Y
	shr rax, 1
	mov dword [screen_center_y], eax
	cvtsi2ss xmm0, eax
	movss dword [ball_pos_y], xmm0

	movss xmm0, dword [rsp + 0] 

	add rsp, 16

	pop rbp
	pop rax
	ret

; Calculates (paddle_center_x, paddle_center_y) based on (x,y), theta and boundary_radius.
; Note: this rounds position to an integer.
_computePaddleCenter:
	push rax
	sub rsp, 16
	movss dword [rsp + 0], xmm0
	movss dword [rsp + 4], xmm1
	movss dword [rsp + 8], xmm2

	cvtsi2ss xmm0, dword [boundary_radius] ; paddle x.
	cvtsi2ss xmm1, dword [boundary_radius] ; paddle y.	

	; Calc sin(theta)
	movss dword [rsp + 12], xmm0 ; Stash xmm0.
	movss xmm0, dword [theta]
	call _sine ; Result in xmm0.
	movss xmm2, xmm0
	movss xmm0, dword [rsp + 12]
	
	; Find paddle_center_x
	mulss xmm0, xmm2 ; rcos(theta).
	cvtsi2ss xmm2, dword [screen_center_x]
	addss xmm0, xmm2 ; Offset.
	cvttss2si rax, xmm0
	mov dword [paddle_center_x], eax


	; Store cos(theta) in xmm2.
	movss xmm0, dword [theta]
	call _cosine ; Result in xmm0.
	movss xmm2, xmm0

	; Find paddle_center_y
	mulss xmm1, xmm2 ; rsin(theta).
	cvtsi2ss xmm2, dword [screen_center_y]
	subss xmm2, xmm1 ; Offset.
	cvttss2si rax, xmm2
	mov dword [paddle_center_y], eax

	movss xmm0, dword [rsp + 0]
	movss xmm1, dword [rsp + 4]
	movss xmm2, dword [rsp + 8]
	add rsp, 16
	pop rax
	ret
	
_flushScreenBuffer:
	; Seek to the beginning of the framebuffer.
    mov rax, SYS_LSEEK              
    mov edi, dword [fb_fd]        
    xor rsi, rsi            ; Offset (beginning of file).
    xor rdx, rdx            ; Whence (SEEK_SET).
    syscall                 

	; Flush.
	mov rax, SYS_WRITE              
    mov edi, dword [fb_fd]        
    mov rsi, staging_buffer         
    mov rdx, FB_SIZE        
    syscall                 

	ret    

; Draws a circle to the staging buffer.
;
; @param - [circle_center_x]
; @param - [circle_center_y]
; @param - [circle_radius]
_drawCircle:
    push rax
    push rbx
    push rcx
    push rdx
    push r8
    push r9
    push r10
    push r11

	; Load parameters into registers
    mov eax, dword [circle_center_x]           
    mov ebx, dword [circle_center_y]            
    mov ecx, dword [circle_radius]            
    mov edx, dword [circle_colour]

    ; Initialize circle drawing variables
    mov r8, rcx
    xor r9, r9               
    mov r10, 1               
    sub r10, rcx
    

_drawCircle_loop:

    ; Draw the 8 symmetric points
    call _drawCircle_plotPoints

    inc r9 
    cmp r10, 0
    jle _drawCircle_loop_decisionNegative 
    
    dec r8
    sub r9, r8
    shl r9, 1
    add r10, r9
    shr r9, 1
    add r9, r8
    add r10, 1 

    jmp _drawCircle_nextPixel
_drawCircle_loop_decisionNegative:
    shl r9, 1
    add r10, r9
    shr r9, 1
    add r10, 1
_drawCircle_nextPixel:
    ; Repeat until x > y 
    cmp r8, r9
    jge _drawCircle_loop

    pop r11
    pop r10
    pop r9
    pop r8
    pop rdx
    pop rcx
    pop rbx
    pop rax

    ret

_drawCircle_plotPoints:
    ; (cx + x, cy + y)
    push rax
    push rbx
    add rax, r8
    add rbx, r9
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx - x, cy + y)
    push rax
    push rbx
    sub rax, r8
    add rbx, r9
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx + x, cy - y)
    push rax
    push rbx
    add rax, r8
    sub rbx, r9
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx - x, cy - y)
    push rax
    push rbx
    sub rax, r8
    sub rbx, r9
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx + y, cy + x)
    push rax
    push rbx
    add rax, r9
    add rbx, r8
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx - y, cy + x)
    push rax
    push rbx
    sub rax, r9
    add rbx, r8
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx + y, cy - x)
    push rax
    push rbx
    add rax, r9
    sub rbx, r8
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ; (cx - y, cy - x)
    push rax
    push rbx
    sub rax, r9
    sub rbx, r8
    call _drawCircle_plotPixel
    pop rbx
    pop rax

    ret

_drawCircle_plotPixel:
    ; Calculate the framebuffer address for the pixel
    ; Address = framebuffer + (y * screen_width + x) * 4
    mov r11, SCREEN_X
    imul r11, rbx
    add r11, rax
    shl r11, 2

    mov dword [x], eax
    mov dword [y], ebx    
    call _cosineOffset ; Result in xmm0.
	movss xmm1, dword [cos_pi_over_12]
	ucomiss xmm0, xmm1
	ja _drawCircle_plotPixel_skip ; Jump less than.

    ; Store the color value at the calculated address
    mov dword [staging_buffer + r11], edx
    ret
_drawCircle_plotPixel_skip:
    mov dword [staging_buffer + r11], 0xffffff    
    ret

_clearScreen:
    push rax
    push rbx

    mov eax, dword [bg_col]
    mov rbx, 0
_clearScreen_loop:
    mov dword [staging_buffer + rbx], eax
    add rbx, 4
    cmp rbx, FB_SIZE
    jne _clearScreen_loop

    pop rbx
    pop rax
    ret

_drawCircleBoundaryWithPaddle:
    push rax
    push rbx

    mov ebx, 0
_drawCircleBoundaryWithPaddle_loop:
    mov eax, dword [screen_center_x]
    mov dword [circle_center_x], eax
    mov eax, dword [screen_center_y]
    mov dword [circle_center_y], eax
    mov eax, dword [boundary_radius]
    add eax, ebx
    mov dword [circle_radius], eax
    mov eax, dword [boundary_colour]
    mov dword [circle_colour], eax
    call _drawCircle
 
    inc ebx
    cmp ebx, dword [boundary_thickness]
    jl _drawCircleBoundaryWithPaddle_loop

    pop rbx
    pop rax
    ret

_drawBall:
    push rax
    push rbp
    mov rbp, rsp

    sub rsp, 16
    movss dword [rsp + 0], xmm0

    movss xmm0, dword [ball_pos_x]
    cvttss2si eax, xmm0
    mov dword [circle_center_x], eax
    movss xmm0, dword [ball_pos_y]
    cvttss2si eax, xmm0
    mov dword [circle_center_y], eax
    mov eax, dword [ball_radius]
    mov dword [circle_radius], eax
    mov eax, dword [ball_colour]
    mov dword [circle_colour], eax 
    call _drawCircle

    movss xmm0, dword [rsp + 0]
    add rsp, 16

    pop rbp
    pop rax
    ret

_handleCollision:
    push rax
    push rbp
    mov rbp, rsp

    sub rsp, 32
    movss dword [rsp + 0], xmm0 ; Dir of ball from center {x}.
    movss dword [rsp + 4], xmm1 ; Dir of ball from center {y}.
    movss dword [rsp + 8], xmm2 ; Aux.
    movss dword [rsp + 12], xmm3 ; Aux.
    movss dword [rsp + 16], xmm4 ; Aux.
    movss dword [rsp + 20], xmm5 ; Aux.

    ; Calc dir of ball from center.
    movss xmm0, dword [ball_pos_x]
    movss xmm1, dword [ball_pos_y]

    mov eax, dword [screen_center_x]
    cvtsi2ss xmm2, eax
    mov eax, dword [screen_center_y] 
    cvtsi2ss xmm3, eax
    subss xmm0, xmm2
    subss xmm1, xmm3

    ; Calc mag of dir of ball from center. 
    movss xmm2, xmm0
    mulss xmm2, xmm2
    movss xmm3, xmm1
    mulss xmm3, xmm3,
    addss xmm2, xmm3 ; Mag squared.  
    movss xmm3, xmm0 ; Stash dir of ball from center {x}.
    movss xmm0, xmm2
    mov eax, [boundary_radius] 
    cvtsi2ss xmm2, eax
    mulss xmm2, xmm2 ; Boundary radius squared.  
    comiss xmm0, xmm2 ; Check if ball is out of range.
    ja _handleCollision_outsideBoundary
    jmp _handleCollision_end ; If we are still within the boundary, there is no collision to handle.
_handleCollision_outsideBoundary:
    movss xmm2, dword [ball_pos_x] 
    cvttss2si eax, xmm2
    mov dword [x], eax
    movss xmm2, dword [ball_pos_y]
    cvttss2si eax, xmm2
    mov dword [y], eax
    movss xmm2, xmm0 ; Stash mag squared.
    call _cosineOffset
    comiss xmm0, dword [cos_pi_over_12]
    ja _handleCollision_paddleContact ; If greater it means angle is smaller so there is contact.
    call _exit ; Process just ends (for now) when we lose. 
_handleCollision_paddleContact:
    movss xmm0, xmm2 ; Restore mag squared.
    call _sqrt
    movss xmm2, xmm0 ; Store mag.
    movss xmm0, xmm3 ; Restore dir from center {x}.

    ; Normalise dir of ball from center. 
    divss xmm0, xmm2
    divss xmm1, xmm2 
    
    ; Dotprod of velocity vector with dir of ball from center.
    movss xmm2, dword [ball_vel_x]
    movss xmm3, dword [ball_vel_y]
    mulss xmm2, xmm0
    mulss xmm3, xmm1
    addss xmm2, xmm3 ; Store dotprod

    ; Ensure dotprod is non-negative.
    mov eax, -1
    cvtsi2ss xmm3, eax  
    comiss xmm2, dword [zero]
    ja _handleCollision_magPositive
    mulss xmm2, xmm3 
_handleCollision_magPositive:   
    ; Require to move ball towards center by offset of radius to prevent 'bunny hopping' where collision handler triggers more than once, increasing speed of ball.
    movss xmm4, xmm0
    movss xmm5, xmm1
    mov eax, dword [ball_radius]
    cvtsi2ss xmm3, eax
    mulss xmm4, xmm3
    mulss xmm5, xmm3 
    mulss xmm4, dword [minus_one]
    mulss xmm5, dword [minus_one]
    addss xmm4, dword [ball_pos_x]
    addss xmm5, dword [ball_pos_y]
    movss dword [ball_pos_x], xmm4
    movss dword [ball_pos_y], xmm5 

    ; Perform a cross product to determine the sign of the random angle. This gives player some control over direction.
    mov eax, dword [paddle_center_x]
    sub eax, dword [screen_center_x]
    cvtsi2ss xmm4, eax
    mov eax, dword [paddle_center_y]
    sub eax, dword [screen_center_y]
    cvtsi2ss xmm5, eax
    mulss xmm4, xmm1
    mulss xmm5, xmm0
    subss xmm4, xmm5
    movss xmm5, dword [one]
    comiss xmm4, dword [zero]
    ja _handleCollision_anglePositive
    movss xmm5, dword [minus_one]
_handleCollision_anglePositive:
    movss dword [rand_offset_sign], xmm5    

    ; Scale dir of ball from center.
    mulss xmm0, xmm2
    mulss xmm1, xmm2

    ; Apply velocity reflection.
    movss xmm2, dword [ball_vel_x]
    movss xmm3, dword [ball_vel_y]
    subss xmm2, xmm0
    subss xmm2, xmm0
    subss xmm3, xmm1
    subss xmm3, xmm1

    ; Multiply ball veclocity vector by small random rotation matrix.
    movss xmm4, xmm2
    movss xmm5, xmm3
    call _randRotMat
    mulss xmm0, xmm4
    mulss xmm1, xmm5
    addss xmm0, xmm1 ; Reflected and randomised [ball_vel_x].
    mulss xmm2, xmm4
    mulss xmm3, xmm5
    addss xmm2, xmm3 ; Reflected and randomised [ball_vel_y].

    ; Save. 
    movss dword [ball_vel_x], xmm0
    movss dword [ball_vel_y], xmm2

_handleCollision_end:
    movss xmm0, dword [rsp + 0]
    movss xmm1, dword [rsp + 4]
    movss xmm2, dword [rsp + 8]
    movss xmm3, dword [rsp + 12]
    movss xmm4, dword [rsp + 16]
    movss xmm5, dword [rsp + 20]
    add rsp, 32
    
    pop rbp
    pop rax
    ret

; Returns random anticlockwise rotation matrix.
; @param - [random_offset_upper]
; @param - [random_offset_lower]
; @return - [[xmm0, xmm1], [xmm2, xmm3]] corrensponding to [[cos(x), -sin(x)], [sin(x), cos(x)]].
_randRotMat:
    movss xmm0, dword [rand_offset_lower]
    movss xmm1, dword [rand_offset_upper]
    mulss xmm0, dword [rand_offset_sign]
    mulss xmm1, dword [rand_offset_sign]
    call _randFloat
    movss xmm3, xmm0 ; Stash random int in xmm3.
    call _sine
    movss xmm1, xmm0
    movss xmm0, xmm3 ; Reload xmm0 with the random angle.
    call _cosine
    movss xmm3, xmm0
    movss xmm2, xmm1
    mulss xmm1, dword [minus_one]
    
    ret  

; Generates a random float in a given range.
; 
; @param - xmm1 - lower bound.
; @param - xmm2 - upper bound.
; @return - xmm0 - random float in range.
_randFloat:
    push rax
    push rbp
    mov rbp, rsp

    sub rsp, 16

    movss dword [rsp + 0], xmm1
    movss dword [rsp + 4], xmm2 

_randFloat_loop: ; According to docs, hardware can take time to generate rand so requires polling
    rdrand eax
    jnc _randFloat_loop  
    shr eax, 1 ; Ensures positive. 
    subss xmm1, xmm0 ; Store range. 
	cvtsi2ss xmm2, eax
    mulss xmm2, [tiny] ; Range proportion (betwee 0 and 1).
    
    mulss xmm1, xmm2 ; Multiply range by random proportion.
    addss xmm0, xmm1 ; Add min.
    
    movss xmm1, dword [rsp + 0]
    movss xmm2, dword [rsp + 4]

    add rsp, 16

    pop rbp
    pop rax
    ret

_exit:
	; Close framebuffer device.
    mov rax, SYS_CLOSE              
    mov edi, dword [fb_fd]        
    syscall                 

	; Close keyboard device.
	mov rax, SYS_CLOSE
	mov edi, dword [kb_fd]
	syscall

	mov rax, SYS_EXIT
	xor rdi, rdi
	syscall
