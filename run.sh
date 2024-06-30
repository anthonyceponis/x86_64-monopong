nasm -f elf64 -g main.asm -o main.o
ld main.o -o main
sudo ./main
echo $?
