
// Header File

#include<linux/types.h>
/* Kernel device IDs */
#define PCI_VENDOR_ID_CCORSI 0x1234
#define PCI_DEVICE_ID_CCORSI_KYOUKO3 0x1113
#define KYOUKO_CONTROL_SIZE 65536
#define DEVICE_RAM 0x0020
#define PAGESHIFT 2
#define KYOUKO3_FRAMEBUFFER_SIZE (1024 * 764 *4)
#define VERTEX_EMIT  0x3004

/*DMA Buffer Attributes */
#define DMA_BUFFER_SIZE (1024*124)
#define DMA_NUM_BUFS    (8)

/* FIFO parameters */
#define FIFO_START 0x1020
#define FIFO_END 0x1024
#define FIFO_ENTRIES 1024
#define FIFO_HEAD 0x4010
#define FIFO_TAIL 0x4014

/* Frame parameters */
#define FRAME_COLUMNS 0X8000
#define FRAME_ROWS 0X8004
#define FRAME_ROWPITCH 0X8008
#define FRAME_PIXELFORMAT 0X800C
#define FRAME_STARTADDRESS 0X8010

/* Encoder parameters */
#define ENCODER_WIDTH 0X9000
#define ENCODER_HEIGHT 0X9004
#define ENCODER_BITX 0X9008
#define ENCODER_BITY 0X900C
#define ENCODER_FRAME 0X9010
#define ACCELERATION_BITMASK 0X1010
#define CONFIG_MODESET 0X1008

/* Color parameters */
#define RED 0x5018
#define GREEN 0x5014
#define BLUE 0x5010
#define ALPHA 0x501C

/* Raster parameters */
#define RASTER_CLEAR 0x3008
#define RASTER_FLUSH 0x3FFC
#define RASTER_PRIMITIVE 0x3000

/* IOWR parameters */
#define VMODE _IOW(0xCC, 0, unsigned long)
#define FIFO_QUEUE _IOWR(0xCC, 3, unsigned long)
#define FIFO_FLUSH _IO(0XCC, 4)
#define BIND_DMA _IOW(0XCC, 1, unsigned long)
#define START_DMA _IOWR(0XCC, 2 , unsigned long)
#define UNBIND_DMA _IOW(0XCC, 5 , unsigned long)

/* Graphics parameters */
#define GRAPHICS_ON 1
#define GRAPHICS_OFF 0

/* Coordinate paramters */
#define X_COORDINATE 0X5000
#define Y_COORDINATE 0X5004
#define Z_COORDINATE 0X5008
#define W_COORDINATE 0X500C

/*Buffer information */
#define NUM_BUFS 8

/* Stream Control-BufferA.Address and BufferA.Config */
#define BUFFERA_ADDRESS 0X2000
#define BUFFERA_CONFIG 0X2008

/* Interrupt handler parameters */
#define INTERUPT_STATUS 0x4008
#define INPT_SET 0x100C



/* Structures */

/* User device structure */
struct u_kyouko_device 
{
  unsigned int *u_control_base;
  unsigned int *u_frame_buffer;
}u_kyouko3;

struct kyouko3_dma_hdr
{
  unsigned int address:14;
  unsigned int count:10;
  unsigned int opcode:8;
}hdr;

/* Fifo structures */
struct fifo_entry
{
  unsigned int command;
  unsigned int value;
};


