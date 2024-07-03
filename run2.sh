nasm -f elf64 -g main2.asm -o main2.o
ld main2.o -o main2
sudo ./main2
echo $?
