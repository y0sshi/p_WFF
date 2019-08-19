#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PENALTY 7
#define DATA_WIDTH 72
#define EX_WIDTH 14
#define WV_WIDTH 10
#define CF_WIDTH 10
#define PN_WIDTH 4

// initial begin
//   BRAM_FIFO_blk[i].FIFO.RAM.RAM[j] <= {72bit data};
// end

typedef struct DATA {
  int ex;
  int wv;
  int cf_9;
  int cf_7;
  int cf_5;
  int cf_3;
  int cf_2;
  int cf_1;
} DATA;

int printb(int value, int bit_width);
int fprintb(FILE *fp, int value, int bit_width);
int bit_array(char *array, int value, int bit_width);

int main(int argc, char *argv[]) {
  int i, j;
  int X,Y,Z, XY;
  int NODE_NUM;
  FILE *fp;
  DATA *RAM;

  // input X Y Z
  if (argc != 4) {
    printf("input X Y Z\n");

    printf("X = ");
    scanf("%d",&X);
    printf("Y = ");
    scanf("%d",&Y);
    printf("Z = ");
    scanf("%d",&Z);
  }
  else {
    X = strtol(argv[1],NULL,10);
    Y = strtol(argv[2],NULL,10);
    Z = strtol(argv[3],NULL,10);
  }
  printf("(X,Y,Z) = (%d,%d,%d)\n",X,Y,Z);
  NODE_NUM = X*Y*Z;
  XY = X*Y;

  char cf_en9, cf_en2, cf_en1, ex_source;

  RAM = (DATA *)malloc(sizeof(DATA)*NODE_NUM);
  for (i=0;i<NODE_NUM;i++) {
    cf_en9 = ((i%XY) < (XY-X));
    cf_en2 = (i < (NODE_NUM-XY));
    cf_en1 = ((i%X) < (X-1));
    ex_source = (i<XY);

    // cf_en2 : is this node connected sink ?
    if (!cf_en2) {
      RAM[i].ex = 16379; 
    }
    else if (ex_source) {
      RAM[i].ex = 10;
    }
    else {
      RAM[i].ex = 0;
    }
    RAM[i].cf_2 = cf_en2 ? i : 0;

    RAM[i].wv = 0;
    RAM[i].cf_9 = cf_en9 ? PENALTY : 0;
    RAM[i].cf_7 = 0;
    RAM[i].cf_5 = 0;
    RAM[i].cf_3 = 0;
    RAM[i].cf_1 = cf_en1 ? PENALTY : 0;
  }

  // write init file
  int g;
  char filename[] = "./init.dat";

  fp = fopen(filename,"w");
  printf("// initial begin\n");
  for (i=0;i<Z;i++){
    for (j=0;j<XY;j++){
      g = j + i*XY;
      fprintf(fp,"BRAM_FIFO_blk[%d].FIFO.RAM.RAM[%d] <= 72'b",i,j);
      fprintb(fp,RAM[g].ex,EX_WIDTH);
      fprintb(fp,RAM[g].wv,WV_WIDTH);
      fprintb(fp,RAM[g].cf_9,PN_WIDTH);
      fprintb(fp,RAM[g].cf_7,CF_WIDTH);
      fprintb(fp,RAM[g].cf_5,CF_WIDTH);
      fprintb(fp,RAM[g].cf_3,CF_WIDTH);
      fprintb(fp,RAM[g].cf_2,CF_WIDTH);
      fprintb(fp,RAM[g].cf_1,PN_WIDTH);
      fprintf(fp,";\n");
    }
  }
  printf("// end\n");

  //printb(RAM[NODE_NUM-1].ex,EX_WIDTH);
  //printf("\n");

  free(RAM);
  return 0;
}

int printb(int value, int bit_width){
  int i;
  char bit[bit_width];

  bit_array(bit,value,bit_width);

  for(i=0;i<bit_width;i++){
    printf("%d",bit[i]);
  }

  return 0;
}

int fprintb(FILE *fp, int value, int bit_width){
  int i;
  char bit[bit_width];

  bit_array(bit,value,bit_width);

  for(i=0;i<bit_width;i++){
    fprintf(fp,"%d",bit[i]);
  }

  return 0;
}

int bit_array(char *array, int value, int bit_width){
  int i, temp;
  int b;

  for (i=bit_width-1;i>=0;i--){
    array[i] = value % 2;
    value = value >> 1;
  }

  return 0;
}
