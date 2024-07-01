; Global vars.
; Use the following command to determine the correct dimensions.
; cat /sys/class/graphics/fb0/virtual_size
%define SCREEN_X 1920
%define SCREEN_Y 1200
%define COLOUR_DEPTH 4 ; In bytes.
%define FB_SIZE SCREEN_X * SCREEN_Y * COLOUR_DEPTH ; In bytes.

; Syscall macros.
%define SYS_EXIT 60
%define SYS_READ 0
%define SYS_WRITE 1
%define SYS_OPEN 2
%define SYS_LSEEK 8
%define SYS_CLOSE 3

; Misc.
%define O_RDONLY 0
%define O_WRONLY 1
%define O_RDWR 2

; Keycodes (case insensitive)
%define KEY_CODE_Q 16
%define KEY_CODE_LEFT_ARROW 105
%define KEY_CODE_RIGHT_ARROW 106

section .data
    fb_device db "/dev/fb0", 0
	kb_device db "/dev/input/event3", 0 ; 3 for inbuilt, 22 for wireless.

	bg_col dd 0xFF0000 ; Red.

	; Can be thought of as a cursor, refers to the pixel in the fb that is currently in focus.
	x dd 0
	y dd 0

	screen_center_x dd 0
	screen_center_y dd 0

	ball_pos_x dd 0
	ball_pos_y dd 0
	ball_vel_x dd 0
	ball_vel_y dd 0
	ball_radius dd 100

	boundary_radius dd 250
	boundary_thickness dd 25

	theta dd 0.0 ; Determines position of paddle relative to north, in radians. 
	theta_offset dd 0.05 ; Baisically controls speed at which paddle turns.
	pi dd 3.14159265
	cos_pi_over_12 dd 0.9659258 ; cos(pi/12) roughly. Used to determine the width of the paddle.

	paddle_center_x dd 0
	paddle_center_y dd 0

	move_state dd 0 ; 0 (still), 1 (anti-clockwise), 2 (clockwise).

section .bss
    fb_fd resq 1
	kb_fd resq 1
	staging_buffer resb FB_SIZE
	kb_buffer resb 24 ; Buffer to hold integer string representation.

section .text
    global _start

_start:
	; Testing area.
	;mov ebx, 6
	;cvtsi2ss xmm1, ebx
	;movss xmm0, [theta]   
	;divss xmm0, xmm1
	;mov ebx, 11
	;cvtsi2ss xmm1, ebx
	;mulss xmm0, xmm1
	;call _cosine
;_finish:
	;mov rax, SYS_EXIT
	;xor rdi, rdi
	;syscall

    ; Open framebuffer device.
    mov rax, SYS_OPEN              
    mov rdi, fb_device      
    mov rsi, O_RDWR              
    xor rdx, rdx ; Mode (not used).
    syscall                 
    mov [fb_fd], rax ; Store file descriptor.

	; Open keyboard device.
	mov rax, SYS_OPEN
	mov rdi, kb_device
	mov rsi, O_RDONLY
	xor rdx, rdx ; Mode (not used).
	syscall
	mov [kb_fd], rax ; Store file descriptor.

	call _computeScreenCenter
	call _mainLoop   
    
_mainLoop:
	; Update screen.
	; call _clearScreen
	call _drawCircle
	call _flushScreenBuffer

	; Read input event.
	mov rax, SYS_READ
	mov rdi, [kb_fd]
	mov rsi, kb_buffer
	mov rdx, 24 ; Number of bytes to read (input event size).
	syscall

	cmp rax, 24	
	jne _mainLoop_noInput
	mov ax, [kb_buffer + 16]
	cmp ax, 1
	jne _mainLoop_noInput
	call _processInput
_mainLoop_noInput:
	call _updateTheta
	call _computePaddleCenter
	jmp _mainLoop

_updateTheta:	
	mov eax, [move_state]	
	cmp eax, 2 ; Clockwise.
	je _updateTheta_inc
	cmp eax, 1 ; Anti-clockwise.
	je _updateTheta_dec
	ret
_updateTheta_inc:
	movss xmm0, [theta]
	movss xmm1, [theta_offset]
	addss xmm0, xmm1 ; Move theta. 	
	movss dword [theta], xmm0
	
	; Now do range check. If over 2pi, subtract 2pi.
	mov eax, 2
	cvtsi2ss xmm1, eax
	movss xmm2, [pi]
	mulss xmm2, xmm1 ; 2pi
	ucomiss xmm0, xmm2
	jb _updateTheta_end ; If within 2pi, we are fine.
	subss xmm0, xmm2 ; Subtract 2pi.
	movss dword [theta], xmm0
	ret
_updateTheta_dec:
	movss xmm0, [theta]
	movss xmm1, [theta_offset]
	subss xmm0, xmm1 ; Move theta. 	
	movss dword [theta], xmm0
	
	; Now do range check. If less than 0, add 2pi.
	mov eax, 0
	cvtsi2ss xmm1, eax
	ucomiss xmm0, xmm1
	ja _updateTheta_end ; If greater than 0, we are fine. 
	mov eax, 2
	cvtsi2ss xmm1, eax
	movss xmm2, [pi]
	mulss xmm2, xmm1 ; 2pi
	addss xmm0, xmm2 ; Add 2pi.
	movss dword [theta], xmm0
	ret
_updateTheta_end:
	ret	

_processInput:	
	mov eax, [kb_buffer + 20] ; Value (offset 20 in input_event structure).
	cmp eax, 0 ; Key release.
	je _processInput_release
	cmp eax, 1 ; Key press.
	jge _processInput_press
	ret
_processInput_release:
	mov dword [move_state], 0
	ret
_processInput_press:	
	mov ax, [kb_buffer + 18] ; Keycode (offset 18 in input_event structure).
	cmp ax, KEY_CODE_LEFT_ARROW
	je _processInput_moveAntiClockwise	
	cmp ax, KEY_CODE_RIGHT_ARROW
	je _processInput_moveClockwise
	cmp ax, KEY_CODE_Q
	je _processInput_quit
	ret
_processInput_moveAntiClockwise:
	mov dword [move_state], 1	
	ret
_processInput_moveClockwise:
	mov dword [move_state], 2 
	ret
_processInput_quit:
	; Close framebuffer device.
    mov rax, SYS_CLOSE              
    mov rdi, [fb_fd]        
    syscall                 

	; Close keyboard device.
	mov rax, SYS_CLOSE
	mov rdi, [kb_fd]
	syscall

	call _exit

; Sets entire framebuffer to one colour.
_clearScreen:
	push rax
	push rdx
	
	mov rax, 0
	mov edx, [bg_col]
_clearScreenLoop:
	mov dword [staging_buffer + rax], edx ; rax is the offset.
	add rax, 4
	cmp rax, FB_SIZE
	jne _clearScreenLoop

	pop rdx
	pop rax
	ret

_drawCircle:	
	push rax
	push rcx
	push rbp
	mov rbp, rsp

	mov rbx, 0 ; Framebuffer offset.
_drawCircleLoop:		
	; Find distance from screen center.
	call _distBetweenPointAndCenter2 ; Result in eax.
	mov ecx, [boundary_radius]
	imul ecx, ecx ; Square radius.
	sub eax, ecx
	cmp eax, 0 ; Make sure computed distance is positive.
	jge _positive
	neg eax
_positive:
	; Set correct color of pixel.
	mov dword [staging_buffer + rbx], 0xFF0000 ; Red.
	mov ecx, [boundary_thickness]
	imul ecx, ecx ; Square thickness.
	cmp eax, ecx
	jg _nextPixel
	mov dword [staging_buffer + rbx], 0x000000 ; Black.	

	; Check if part of the paddle.	
	; WARNING: Will probably render multiple paddles due to periodicity of cos and squaring.
	call _cosineOffset ; Result in xmm0.
	movss xmm1, [cos_pi_over_12]
	ucomiss xmm0, xmm1
	jb _nextPixel ; Jump less than.
	mov dword [staging_buffer + rbx], 0xFFFFFF ; White.	
_nextPixel:
	; Move to next pixel.
	add rbx, 4

	; Determine next x and y coords.
	mov rax, rbx
	shr rax, 2 ; Divide by 4 since pixel moves in bytes of 4.
	mov rdx, 0 ; Clear rdx for unsigned division.
	mov rcx, SCREEN_X
	idiv rcx
	mov dword [y], eax ; Quotient.
	mov dword [x], edx ; Remainder.

	cmp rbx, FB_SIZE
	jne _drawCircleLoop 	
	
	pop rbp
	pop rcx
	pop rax

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

	movss xmm1, [pi]
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
	movss xmm1, [pi]
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

	movss xmm1, [rsp + 0]
	movss xmm2, [rsp + 4]
	movss xmm3, [rsp + 8]
	movss xmm4, [rsp + 12]
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
	movss xmm1, [pi]
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
	movss xmm1, [pi]
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
	movss xmm1, [pi]
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

	movss xmm1, [rsp + 0]
	movss xmm2, [rsp + 4]
	movss xmm3, [rsp + 8]
	movss xmm4, [rsp + 12]
	add rsp, 16
	pop rbx
	pop rax
	ret

; Computes distance squared between (x0,y0) and (x1,y1).	
; @param - Setup stack such that [rsp + 0] = x0, [rsp + 4] = y0, [rsp + 8] = x1, [rsp + 12] = y1.
;
; @return eax - distance.
_dist2:
	; Must move rsp since the function call itself pushes the return address onto the stack.
	add rsp, 8

	; Find deltas.
	mov eax, [rsp + 8]
	sub dword [rsp + 0], eax
	mov eax, [rsp + 12]
	sub dword [rsp + 4], eax 

	; Square delatas.
	mov eax, [rsp + 0]
	imul eax, eax
	mov dword [rsp + 0], eax
	mov eax, [rsp + 4]
	imul eax, eax
	mov dword [rsp + 4], eax

	mov eax, [rsp + 0]
	add eax, [rsp + 4]

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
	mov eax, [paddle_center_x] 
	mov dword [rsp + 0], eax 	
	mov eax, [paddle_center_y]
	mov dword [rsp + 4], eax
	mov eax, [screen_center_x]
	mov dword [rsp + 8], eax
	mov eax, [screen_center_y]
	mov dword [rsp + 12], eax
	call _dist2
	cvtsi2ss xmm0, rax   
	call _sqrt
	mulss xmm0, [rsp + 16]
	movss dword [rsp + 16], xmm0

	; Find dotprod of length between center and point with length between center and paddle.
	; Multiply x-components.
	mov eax, [x]
	sub eax, [screen_center_x]
	mov dword [rsp + 20], eax
	mov eax, [paddle_center_x]
	sub eax, [screen_center_x]
	imul eax, [rsp + 20]
	; Multiply y-components.
	mov ebx, [y]
	sub ebx, [screen_center_y]
	mov dword [rsp + 20], ebx
	mov ebx, [paddle_center_y]
	sub ebx, [screen_center_y]
	imul ebx, [rsp + 20]
	; Add components together.
	add eax, ebx	
	mov dword [rsp + 20], eax

	; Put it all together.
	cvtsi2ss xmm0, [rsp + 20]
	divss xmm0, [rsp + 16]	
		
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
	
	movss xmm1, [rsp + 0]
	movss xmm2, [rsp + 4]
	add rsp, 16
	pop rbx
	pop rax

_computeScreenCenter:
	push rax

	mov rax, SCREEN_X
	shr rax, 1
	mov dword [screen_center_x], eax
	mov rax, SCREEN_Y
	shr rax, 1
	mov dword [screen_center_y], eax

	pop rax

; Calculates (paddle_center_x, paddle_center_y) based on (x,y), theta and boundary_radius.
; Note: this rounds position to an integer.
_computePaddleCenter:
	push rax
	sub rsp, 16
	movss dword [rsp + 0], xmm0
	movss dword [rsp + 4], xmm1
	movss dword [rsp + 8], xmm2

	cvtsi2ss xmm0, [boundary_radius] ; paddle x.
	cvtsi2ss xmm1, [boundary_radius] ; paddle y.	

	; Calc sin(theta)
	movss dword [rsp + 12], xmm0 ; Stash xmm0.
	movss xmm0, [theta]
	call _sine ; Result in xmm0.
	movss xmm2, xmm0
_break:
	movss xmm0, [rsp + 12]
	
	; Find paddle_center_x
	mulss xmm0, xmm2 ; rcos(theta).
	cvtsi2ss xmm2, [screen_center_x]
	addss xmm0, xmm2 ; Offset.
	cvttss2si rax, xmm0
	mov dword [paddle_center_x], eax


	; Store cos(theta) in xmm2.
	movss xmm0, [theta]
	call _cosine ; Result in xmm0.
	movss xmm2, xmm0

	; Find paddle_center_y
	mulss xmm1, xmm2 ; rsin(theta).
	cvtsi2ss xmm2, [screen_center_y]
	subss xmm2, xmm1 ; Offset.
	cvttss2si rax, xmm2
	mov dword [paddle_center_y], eax

	movss xmm0, [rsp + 0]
	movss xmm1, [rsp + 4]
	movss xmm2, [rsp + 8]
	add rsp, 16
	pop rax
	ret
	
_flushScreenBuffer:
	; Seek to the beginning of the framebuffer.
    mov rax, SYS_LSEEK              
    mov rdi, [fb_fd]        
    xor rsi, rsi            ; Offset (beginning of file).
    xor rdx, rdx            ; Whence (SEEK_SET).
    syscall                 

	; Flush.
	mov rax, SYS_WRITE              
    mov rdi, [fb_fd]        
    mov rsi, staging_buffer         
    mov rdx, FB_SIZE        
    syscall                 

	ret

_exit:
	mov rax, SYS_EXIT
	xor rdi, rdi
	syscall
