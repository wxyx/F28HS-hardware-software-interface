#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MAX_MSG_LENGTH 100

//定义了一个表示像素的结构体，包含一个像素的rgb值
struct Pixel {
    int red;
    int green;
    int blue;
};

struct Pixel6
{
    /* data */
    unsigned char red;
    unsigned char green;
    unsigned char blue;
};

//定义了一个ppm文件类型的结构体，包含了ppm文件中的所有内容
//分别为图片的宽方向上像素值的数量，高方向上像素值的数量，和每个像素值中rgb分量最大为多少
//还有一个是Pixel类型的二维数组，用于记录每个像素
struct PPM {
    int type;
    int width, height, max;
    struct Pixel **pixels;
    struct Pixel6 ** pixel6s;
};



//这个函数是通过一个ppm格式的文件获取一个PPM的对象
//也就是把一个ppm图像的信息用上面那个结构体记录下来
struct PPM *getPPM(FILE *f) {
    char line[70];  //缓冲区
    struct PPM *img = malloc(sizeof(struct PPM)); //给ppm结构体分配内存空间
    fgets(line, 70, f);  //读取文件一行的内容，且最多只能读取70个字符，将读取的字符冲入缓冲区

    //ppm文件有两种类型，p3编码的类型和p6编码的类型，ppm文件第一行记录了该ppm文件使用的是哪种编码方式
    //这个程序中p3和p6类型都是允许的
    if (strcmp(line, "P3\n") != 0 && strcmp(line, "P6\n") != 0) { //strcmp就是比较字符串的函数，如果这个字符串在主串里出现过，会返回其位置
        fprintf(stderr, "\nError: File Not in PPM (P3 or P6) Format\n");
        exit(0);
    }

    img->type = line[1] - '0';

    fgets(line, 70, f);

    while (line[0] == '#') {  //有可能文件会有乱码#，所以必须把#读掉，不让其干扰我们存储像素
        fgets(line, 70, f);
    }

    int imageWidth, imageHeight;
    sscanf(line, "%d %d", &imageWidth, &imageHeight);
    img->width = imageWidth;
    img->height = imageHeight;  //ppm文件的第二行是ppm文件的宽和高，读取并放入结构体中

    if (img->type == 6) {  //如果是p6类型的编码的话，像素的值是用byte来存的，并不是数字，需要用fread来读取byte类型的变量
        fgets(line, 70, f);
        sscanf(line, "%d", &img->max);
        fgetc(f); // Skip the newline character before pixel data

        img->pixel6s = malloc(sizeof(struct Pixel *) * img->height);

        for (int j = 0; j < img->height; j++) {
            img->pixel6s[j] = malloc(sizeof(struct Pixel6) * img->width);
            fread(img->pixel6s[j], sizeof(struct Pixel6), img->width, f);
        }
    } else {  //如果是用p3类型的编码的话，像素是用int类型来存的，直接是用fscanf来读取既可
        fscanf(f, "%d", &img->max);

        img->pixels = malloc(sizeof(struct Pixel *) * img->height);

        for (int j = 0; j < img->height; j++) {
            img->pixels[j] = malloc(sizeof(struct Pixel) * img->width);

            for (int k = 0; k < img->width; k++) {
                fscanf(f, "%d", &img->pixels[j][k].red);
                fscanf(f, "%d", &img->pixels[j][k].green);
                fscanf(f, "%d", &img->pixels[j][k].blue);
            }
        }
    }

    return img; //最后返回值就是存取了一个ppm文件的结构体的指针
}

void showPPM(const struct PPM *img) {
//    printf("P%d\n", (img->max > 255) ? 6 : 3);
//    if(img->max > 255)
//    {
//        printf("P6\n");
//    }else{
//        printf("P3\n");
//    }
    //这里就判断是那种编码方式的PPM文件
    if(img->type == 6)
    {
        printf("P6\n");
    }else{
        printf("P3\n");
    }
    printf("%d %d\n", img->width, img->height);
    printf("%d\n", img->max);

    if (img->type == 6) {
        //第一种情况是P6方式编码的PPM文件，应该用fwrite输出，因为P6中存储的是byte类型的变量
        for (int j = 0; j < img->height; j++) {
            fwrite(img->pixel6s[j], sizeof(struct Pixel6), img->width, stdout);
        }
    } else {
        //如果是P3编码方式，直接用printf输出即可，里面存的是int类型的变量
        for (int j = 0; j < img->height; j++) {
            for (int k = 0; k < img->width; k++) {
                struct Pixel p = img->pixels[j][k];
                printf("%d %d %d\n", p.red, p.green, p.blue);
            }
        }
    }
}

struct PPM *readPPM(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (f == NULL) {
        fprintf(stderr, "File %s could not be opened.\n", filename); //stderr是个标准的错误流，如果没有这个文件的话就输出错误流中的话，并返回空指针
        return NULL;
    }

    struct PPM *img = getPPM(f);

    fclose(f); //关闭输入流，节约空间

    if (img == NULL) {
        fprintf(stderr, "File %s could not be read.\n", filename);
        return NULL;
    }

    return img;  //返回指针，我感觉这个函数主要是为了防止读取失败或者文件不存在的情况，其基本用法基本和getPPM相同
}

struct PPM *encode(char *text, struct PPM *img) {
    srand(time(NULL));
    int j, acc, random;
    acc = 0;

    int type = img->type; //这个是判断图片的编码方式，两种编码方式有不同的编码和解码，所以我们事先判断编码的方式

    //判断我们要加密的内容是否超过规定长度，超过就终止
    if (strlen(text) > MAX_MSG_LENGTH) {
        printf("\nMessage exceeds max message length.\n");
        exit(0);
    }

    if(type == 3)
    {
        //逐一获取加密代码中的字符
        for (j = 0; j < strlen(text); j++) {
            random = (rand() % 100);  //获取一个随机数
            acc = acc + random;  //使acc随机化

            int row, column;
            row = acc / img->width;
            column = acc - (row * img->width);  //随机选取一个像素，用来存储加密的信息

            struct Pixel *p = &(img->pixels[row][column]);
            //p->red = text[j];  //用这个随机选取的像素的红色偏移量来存储我们的加密信息，我觉得换成其他颜色也行
            if(j % 3 == 0){
                p->red = text[j];
            }else if(j % 3 == 1){
                p->green = text[j];
            }else{
                p->blue = text[j];
            }
        }
    }else
    {
        for (j = 0; j < strlen(text); j++) {
            random = (rand() % 100);
            acc = acc + random;

            int row, column;
            row = acc / img->width;
            column = acc - (row * img->width);

            struct Pixel6 *p = &(img->pixel6s[row][column]);
            p->red = text[j];
        }
    }

    return img;
}
//以下是解码程序
char *decode(struct PPM *oldimg, struct PPM *newimg) {
    char *buffer = malloc(sizeof(char) * MAX_MSG_LENGTH); //定义缓冲区，存储解码的内容

    //大小不相同的话直接不用判断了
    if (oldimg->height != newimg->height || oldimg->width != newimg->width) {
        printf("Files differ in Height or Width\n");
        exit(0);
    }

    int type = oldimg->type; //正如之前所说由于不同的编码方式，加密的方法有些不同，我们解码的方式也要对应

    if(type == 3)
    {
        int l = 0;  //这里表示的是解码第几个字符，注意字符串开始第一个字符的下标为0

        for (int c = 0; c < newimg->height; c++) {
            for (int r = 0; r < newimg->width; r++) {
                if ((l % 3 == 0) && (newimg->pixels[c][r].red != oldimg->pixels[c][r].red)) {
                    buffer[l] = newimg->pixels[c][r].red;
                    printf("%c", buffer[l]);
                    l = l + 1;
                }else if((l % 3 == 1) && (newimg->pixels[c][r].green != oldimg->pixels[c][r].green)){
                    buffer[l] = newimg->pixels[c][r].green;
                    printf("%c", buffer[l]);
                    l = l + 1;
                }else if((l % 3 == 2) && (newimg->pixels[c][r].blue != oldimg->pixels[c][r].blue)){
                    buffer[l] = newimg->pixels[c][r].blue;
                    printf("%c", buffer[l]);
                    l = l + 1;
                }
            }
        }

        buffer[l] = '\0'; //在c语言中，字符串的结尾必是'\0'
        char *str = malloc(sizeof(char) * (l + 1));
        strcpy(str, buffer);  //将解码的内容从缓冲区复制到字符串，完成解码
        free(buffer);
        return str;
    }else
    {
        int l = 0;

        for (int c = 0; c < newimg->height; c++) {
            for (int r = 0; r < newimg->width; r++) {
                if (newimg->pixel6s[c][r].red != oldimg->pixel6s[c][r].red) {
                    buffer[l] = newimg->pixel6s[c][r].red;
                    printf("%c", buffer[l]);
                    l = l + 1;
                }
            }
        }

        buffer[l] = '\0';
        char *str = malloc(sizeof(char) * (l + 1));
        strcpy(str, buffer);
        free(buffer);
        return str;
    }

    return "nothing";
}

int main(int argc, char *argv[]) {
    //main函数基本不用变
    srand(time(NULL));

    int length = 0;

    if (argc == 3 && strcmp(argv[1], "t") == 0) {
        struct PPM *img = readPPM(argv[2]);
        showPPM(img);
    } else if (argc == 3 && strcmp(argv[1], "e") == 0) {
        char txt[MAX_MSG_LENGTH];
        FILE *i1 = fopen(argv[2], "r");
        struct PPM *oldimg = getPPM(i1);

        fprintf(stderr, "Print message to Encode -> ");
        fgets(txt, MAX_MSG_LENGTH, stdin);
        length = strlen(txt) - 1;
        int j = strlen(txt) - 1;
        if (txt[j] == '\n')
            txt[j] = '\0';
        encode(txt, oldimg);

        FILE *pfile = fopen("newfile.ppm", "wb");
        // fprintf(pfile, "P%d\n%d %d\n%d\n", (oldimg->max > 255) ? 6 : 3, oldimg->width, oldimg->height, oldimg->max);
        if(oldimg->type == 3)
        {
            fprintf(pfile, "P3\n%d %d\n%d\n", oldimg->width, oldimg->height, oldimg->max);
        }else
        {
            fprintf(pfile, "P6\n%d %d\n%d\n", oldimg->width, oldimg->height, oldimg->max); 
        }

        if (oldimg->type == 6) {
            for (int x = 0; x < oldimg->height; x++) {
                fwrite(oldimg->pixel6s[x], sizeof(struct Pixel6), oldimg->width, pfile);
            }
        } else {
            for (int x = 0; x < oldimg->height; x++) {
                for (int y = 0; y < oldimg->width; y++) {
                    fwrite(&(oldimg->pixels[x][y]), sizeof(struct Pixel), 1, pfile);
                }
            }
        }

        fclose(pfile);
        showPPM(oldimg);
    } else if (argc == 4 && strcmp(argv[1], "d") == 0) {
        FILE *i1 = fopen(argv[2], "r");
        FILE *i2 = fopen(argv[3], "r");
        struct PPM *oldimg = getPPM(i1);
        struct PPM *newimg = getPPM(i2);
        char *message = decode(oldimg, newimg);
        printf("%.*s\n", length, message);
    } else {
        fprintf(stderr, "Unrecognized or incomplete command line.\n");
        return 1;
    }

    return 0;
}
