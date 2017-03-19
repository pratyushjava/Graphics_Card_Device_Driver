
// User Code


#include<fcntl.h>
#include<stdio.h>
#include<errno.h>
#include<string.h>
#include<math.h>
#include<sys/mman.h>
#include<linux/kernel.h>
#include<linux/ioctl.h>
#include <linux/sched.h>
#include<stdlib.h>

// number of rows and column
#define TOTAL 30

float RADIUS = 0.7;
float HPI = 1.57079632679489661923;
float PI = 3.14159265358979323846;

struct vertex{
float x,y,z;
};

// size is one more than total rows and column, to avoid indexoutofbound 
struct vertex vertces[TOTAL+1][TOTAL+1];

/* User defined header files */
#include "kyouko3.h"

/* Function to write register at the user level */
void U_WRITE_REG(unsigned int rgister, unsigned int value)
{
	*(u_kyouko3.u_frame_buffer+(rgister))=value;
}

/* Function to read register at the user level */
unsigned int U_READ_REG(unsigned int rgister)
{
	return(*(u_kyouko3.u_control_base+(rgister>>2)));
}

// generate random color between 0 to 1
float generate_color()
{
	return (float)rand() / (float)RAND_MAX ;
}

// convert index to theta for longitude range is -180 to 180 degree
// for latitude it is -90 to 90 degree
float map_range(float index, float indexst, float indexed, float expecst, float expeced) 
{
	return expecst + (expeced - expecst) * ((index - indexst) / (indexed - indexst));
}


// computing all coordinates on sphere before
void compute_vertices()
{
	int i = 0 ,j = 0;
	for(i = 0 ; i < TOTAL+1 ; i++)
	{
	// latitude range of sphere is -90 to 90
	float latitude = map_range(i , 0 , TOTAL , -HPI , HPI); 

	for(j = 0 ; j < TOTAL+1 ; j++)
	{
	  // longitude range of sphere is -180 to 180
	  float longitude = map_range(j , 0 , TOTAL , -PI , PI);

	  // computing coordinates
	  vertces[i][j].x = RADIUS * sin(longitude) * cos(latitude);
	  vertces[i][j].y = RADIUS * sin(longitude) * sin(latitude);
	  vertces[i][j].z = RADIUS * cos(longitude);
	}
	}
}


// fill dma buffer
// considering vertices of triangle to be two adjacent vertices and one below them
void draw(int i,int j, unsigned int* dma_base, int* count)
{
	int total_count = *count;
	float zero = 0.0;
	int k = 0;

	// fill random color
	for (k = 0; k < 3; k++)
	{
	total_count++;
	float color = generate_color(); 
	*dma_base++ =  *(unsigned int*)&color;
	}
	*dma_base++ =  *(unsigned int*)&vertces[i][j].x;
	*dma_base++ =  *(unsigned int*)&vertces[i][j].y;
	*dma_base++ =  *(unsigned int*)&vertces[i][j].z;
	total_count +=3;

	for (k = 0; k < 3; k++)
	{
	total_count++;
	float color = generate_color(); 
	*dma_base++ =  *(unsigned int*)&color;
	}
	*dma_base++ =  *(unsigned int*)&vertces[i+1][j].x;
	*dma_base++ =  *(unsigned int*)&vertces[i+1][j].y;
	*dma_base++ =  *(unsigned int*)&vertces[i+1][j].z;
	total_count +=3;

	for (k = 0; k < 3; k++)
	{
	total_count++;
	float color = generate_color();
	*dma_base++ =  *(unsigned int*)&color;
	}
	*dma_base++ =  *(unsigned int*)&vertces[i][j+1].x;
	*dma_base++ =  *(unsigned int*)&vertces[i][j+1].y;
	*dma_base++ =  *(unsigned int*)&vertces[i][j+1].z;
	total_count +=3;
	*count = total_count;
}

void fifo_triangle(int fd)
{
	  int i = 0;
	  float a1 = 0.0;
	  float b1 = -0.5;
	  float a2 = -0.5;
	  float b2 = 0.5;
	  float a3 = 0.5;
	  float b3 = 0.5;
	  float a4 = -1.0;
	  float b4 = 0.25;
	  float a5 = -0.75; 
	  float zero=0.0;
	  float one=1.0;
	  float a= 1.0;
	  float b=0.0;
	  float c=0.0;
	  float d = 0.0;
	  float e = 1.0;
	  float f = 0.0;
	  float g = 0.0;
	  float h = 0.0;
	  float it = 1.0;
	  unsigned int zero_zs_int = *(unsigned int*)&zero;
	  unsigned int one_us_int = *(unsigned int*)&one;
	  unsigned int a1_int = *(unsigned int*)&a1;
	  unsigned int b1_int = *(unsigned int*)&b1;
	  unsigned int a2_int = *(unsigned int*)&a2;
	  unsigned int b2_int = *(unsigned int*)&b2;
	  unsigned int a3_int = *(unsigned int*)&a3;
	  unsigned int b3_int = *(unsigned int*)&b3;
	  unsigned int a4_int = *(unsigned int*)&a4;
	  unsigned int b4_int = *(unsigned int*)&b4;
	  unsigned int aa = *(unsigned int*)&a;
	  unsigned int bb = *(unsigned int*)&b;
	  unsigned int cc = *(unsigned int*)&c;
	  unsigned int dd = *(unsigned int*)&d;
	  unsigned int ee = *(unsigned int*)&e;
	  unsigned int ff = *(unsigned int*)&f;
	  unsigned int gg = *(unsigned int*)&g;
	  unsigned int hh = *(unsigned int*)&h;
	  unsigned int ii = *(unsigned int*)&it;


	struct fifo_entry entry[24] = {{RASTER_PRIMITIVE,1}, 
						{X_COORDINATE, a1_int},
						{Y_COORDINATE, b1_int},
						{Z_COORDINATE, zero_zs_int},
						{W_COORDINATE, one_us_int},
						{X_COORDINATE, a2_int},
						{Y_COORDINATE, b2_int},
						{Z_COORDINATE, zero_zs_int},
						{X_COORDINATE, a3_int},
						{Y_COORDINATE, b3_int},
						{Z_COORDINATE, zero_zs_int},   
						{BLUE, aa},
						{GREEN, bb},
						{RED, cc},
						{ALPHA, zero_zs_int},
						{VERTEX_EMIT,0},
						{RASTER_PRIMITIVE,0},
						{RASTER_FLUSH,0},
						{BLUE, dd},
						{GREEN, ee},
						{RED, ff},
						{BLUE, gg},
						{GREEN, hh},
						{RED, ii},
	  };

  
for(i=0;i<18;i++)
{
	ioctl(fd, FIFO_QUEUE, &entry[i]);
}

ioctl(fd,FIFO_FLUSH);
}

unsigned int ftoui(float *zz)
{
	return *(unsigned int*)zz;
}

void dma_triangles(int fd)
{

	struct fifo_entry d_entry[1]={{RASTER_FLUSH,0}};

	unsigned long dmabase = 0;
	unsigned int* tempbase;

	ioctl(fd,BIND_DMA,&dmabase);
  
	tempbase = (unsigned int*) dmabase;
	compute_vertices();
	hdr.address = 0x1045;
	hdr.opcode = 0x0014;
	hdr.count = 3;

	int i = 0;
	int count = 0;
	int j =0;
	for(i = 0 ; i < TOTAL+1 ; i++){
		for(j = 0 ; j < TOTAL+1 ; j++){
			*tempbase++ = *(unsigned int *)&hdr;
			count++;
			draw(i,j,tempbase,&count);
			dmabase = count * 4;
			ioctl(fd, START_DMA, &dmabase);
			tempbase = (unsigned int *)dmabase;
			ioctl(fd,FIFO_QUEUE,&d_entry[0]);
			count = 0;
		}
	}
	sleep(5);
	ioctl(fd,FIFO_FLUSH,0);
	ioctl(fd, UNBIND_DMA, 0);  
}
	
/* Main function */
int main(void)
{
	int choice;
	int fd;
	int result;
	unsigned long dmabase = 0;
	unsigned int* buf;

	 /* Open the driver */
	fd= open("/dev/kyouko3", O_RDWR);
	printf("Enter your choice:\n 0: FIFO Triangle\n 1: DMA Triangles\n");
	scanf("%d",&choice);
	ioctl(fd,VMODE,GRAPHICS_ON);
	switch(choice)
	{
		case 0: fifo_triangle(fd);break;
		case 1: dma_triangles(fd);break;
		default: dma_triangles(fd);break;
	}
	sleep(5);
	ioctl(fd,VMODE,GRAPHICS_OFF);
	close(fd);
	return 0;
}


