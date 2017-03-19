mknod "/dev/kyouko3" c 500 127
rmmod mymod.ko
make
insmod mymod.ko
gcc spherenew.c -lm


