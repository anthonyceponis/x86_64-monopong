section .data
    fb_path     db "/dev/fb0", 0
    screensize  equ 1920 * 1080 * 4  ; Assuming 1920x1080 resolution, 32 bits per pixel (4 bytes)

section .bss
    fb_fd       resq 1
    fb_ptr      resq 1
    vinfo       resb 128  ; sizeof(struct fb_var_screeninfo) is usually less than 128 bytes

section .text
    global _start

_start:
    ; Open framebuffer device
    mov rdi, fb_path
    mov rsi, O_RDWR
    call sys_open
    mov qword [fb_fd], rax  ; Save file descriptor

    ; Get variable screen information
    mov rdi, qword [fb_fd]
    lea rsi, [vinfo]
    mov rdx, 128  ; sizeof(struct fb_var_screeninfo)
    mov rax, FBIOGET_VSCREENINFO
    syscall
    test rax, rax
    js error_exit

    ; Calculate screensize
    mov rax, 1920
    imul rax, 1080
    imul rax, 4  ; 32 bits per pixel

    ; Map framebuffer to memory
    mov rdi, 0
    mov rsi, rax  ; screensize
    mov rdx, PROT_READ | PROT_WRITE
    mov r10, MAP_SHARED
    mov r8, qword [fb_fd]
    mov r9, 0
    mov rax, SYS_mmap
    syscall
    test rax, rax
    js error_exit
    mov qword [fb_ptr], rax  ; Save framebuffer pointer

    ; Fill framebuffer with white color
    mov rdi, qword [fb_ptr]
    mov rcx, rax  ; screensize
    xor rax, rax  ; Clear RAX for memset
    mov al, 0xFF  ; White color (assuming little-endian)

    cld  ; Clear direction flag for stosb
    rep stosb

    ; Unmap framebuffer memory
    mov rdi, qword [fb_ptr]
    mov rsi, rax  ; screensize
    mov rax, SYS_munmap
    syscall

    ; Close framebuffer device
    mov rdi, qword [fb_fd]
    mov rax, SYS_close
    syscall

    ; Exit program
    mov rax, 60  ; syscall: exit
    xor rdi, rdi  ; status 0
    syscall

error_exit:
    mov rdi, errmsg
    call print_string
    mov rax, 60  ; syscall: exit
    xor rdi, rdi  ; status 0
    syscall

section .data
    errmsg  db "Error: Unable to perform framebuffer operation", 0

section .text
print_string:
    ; Function to print null-terminated string in RDI
    push rdi
    mov rax, 1  ; syscall: write
    mov rsi, rsp  ; Address of the string
    mov rdx, 0xFFFFFFFF  ; Assume a long enough length
    syscall
    pop rdi
    ret

