#include "mbed.h"

BufferedSerial uart0(P1_7, P1_6,9600);
SPI spi(P0_9, P0_8, P0_6);    //mosi, miso, sclk
I2C i2c(P0_5,P0_4); //SDA,SCL P0_5,P0_4
DigitalOut cs1(P0_3);   //dds1. spi.format(16,2);
DigitalOut cs2(P0_2);   //dds2. spi.format(16,2);
DigitalOut cs3(P0_7);   //adc. spi.format(8,1);
DigitalOut sel(P1_0);   //VI sel. 0=V, 1=I
DigitalOut a0(P1_2);
DigitalOut a1(P1_1);
DigitalOut iv0(P1_4);
DigitalOut iv1(P1_5);
DigitalIn drdy(P0_11);

//DDS control
const uint32_t mclk=25000000;   //Hz order
const uint64_t res=268435456;   //2^28
void waveset(uint32_t freq, uint8_t pha);    //dde set func
uint32_t freq=10000;
uint8_t pha;

//adc control
const uint8_t rst=0b0110;
const uint8_t wreg=0b0100;
const uint8_t start=0b1000;
float adc_read(uint8_t avg);  //adc read funv

//amp control
void vg_set();
void ig_set();

//calc
float im_f[2],vm_f[2],den,z_re,z_im,vg,ig,temp;
uint8_t i_gain=0,v_gain=0, avg=10;
int16_t v_real, i_real, v_img, i_img;
uint32_t dut_r,dut_l,dut_c;

//lcd
const uint8_t contrast=40;  //lcd contrast
const uint8_t lcd_addr=0x7C;   //lcd i2c addr 0x7C
void lcd_init(uint8_t addr, uint8_t contrast);     //lcd init func
void char_disp(uint8_t addr, uint8_t position, char data);
void val_disp(uint8_t addr, uint8_t position, uint8_t digit,uint16_t val);

int main(){
    spi.frequency(1000000); //SPI clk 5MHz
    i2c.frequency(400000);  //I2C clk 400kHz
    cs1=1;
    cs2=1;
    cs3=1;
    sel=0;
    a0=0;
    a1=0;
    iv0=0;
    iv1=0;

    //init LCD
    thread_sleep_for(100);  //wait for LCD power on
    lcd_init(lcd_addr, contrast);
    thread_sleep_for(100);  //wait for LCD power on

    //adc init
    spi.format(8,1);
    cs3=0;
    spi.write(rst);
    cs3=1;
    thread_sleep_for(10);   //NEED!!
    cs3=0;
    spi.write((wreg<<4)+(0x01<<2)+0);    //write addr 0x01, 1byte
    spi.write((0b010<<5)+(0b10<<3)+(0b1<<2));       //180sps, turbo mode, cc mode
    cs3=1;
    cs3=0;
    spi.write((wreg<<4));       //write addr 0x00, 1byte
    spi.write((0b0110<<4)+1);   //ch1 ch0 bipolar mux, pga disable h81
    cs3=1;

    char_disp(lcd_addr,0,'R');
    char_disp(lcd_addr,1,'=');
    char_disp(lcd_addr,7,'.');
    char_disp(lcd_addr,12,'k');
    char_disp(lcd_addr,13,0x1e);

    char_disp(lcd_addr,0x40+0,'X');
    char_disp(lcd_addr,0x40+1,'=');
    char_disp(lcd_addr,0x40+7,'.');
    char_disp(lcd_addr,0x40+12,'u');

    while (true){
        //v read
        sel=0;  //select V.
        vg_set();

        waveset(freq,0);
        spi.format(8,1);
        thread_sleep_for(10);
        vm_f[0]=adc_read(avg)/vg;

        waveset(freq,1);
        spi.format(8,1);
        thread_sleep_for(10);
        vm_f[1]=adc_read(avg)/vg;

        //i read
        sel=1;  //select I.
        ig_set();

        waveset(freq,0);
        spi.format(8,1);
        thread_sleep_for(10);
        im_f[0]=adc_read(avg)/ig*-1;

        waveset(freq,1);
        spi.format(8,1);
        thread_sleep_for(10);
        im_f[1]=adc_read(avg)/ig*-1;

        //calc
        den=im_f[0]*im_f[0]+im_f[1]*im_f[1];    //Re(im)^2+Im(im)^2
        z_re=(vm_f[0]*im_f[0]+vm_f[1]*im_f[1])/den;
        z_im=(vm_f[1]*im_f[0]-vm_f[0]*im_f[1])/den;

        //disp
        dut_r=(uint32_t)z_re;
        val_disp(lcd_addr,3,4,dut_r/1000);
        val_disp(lcd_addr,8,3,dut_r%1000);

        if(z_im>=0){    //inductance
            temp=z_im/(6.28*freq)*1000000000;  //nH order
            dut_l=(uint32_t)temp;
            val_disp(lcd_addr,0x40+3,4,dut_l/1000);
            val_disp(lcd_addr,0x40+8,3,dut_l%1000);
            char_disp(lcd_addr,0x40+13,'H');
        }else{  //capacitance
            temp=1/(6.28*freq*z_im*-1)*1000000000;  //nF order
            dut_c=(uint32_t)temp;
            val_disp(lcd_addr,0x40+3,4,dut_c/1000);
            val_disp(lcd_addr,0x40+8,3,dut_c%1000);
            char_disp(lcd_addr,0x40+13,'F');
        }
        
        //debug
        int z_ree,z_imm;
        z_ree=(int)z_re;
        z_imm=(int)z_im;
        printf("Freq=%d, Avg=%d, I_gain=%d, V_gain=%d, Re(Z)=%d, Im(Z)=%d\n\r",freq, avg, i_gain, v_gain, z_ree,z_imm);
    }
}

//dds set
void waveset(uint32_t freq, uint8_t pha){
    uint16_t buf;
    const uint16_t base_pha=0;
    uint32_t reg=freq*res/mclk;
    if(freq>1000000)freq=1000000;
    spi.format(16,2);

    cs1=0;
    cs2=0;
    spi.write(0x2100);
    buf=(reg&0x3FFF)+0x4000;
    spi.write(buf);
    buf=(reg>>14)+0x4000;
    spi.write(buf);
    buf=(4096*base_pha/360)+0xC000;    //0deg
    spi.write(buf);
    cs1=1;
    cs2=1;
    
    if(pha!=0){
        cs2=0;
        buf=(4096*(90)/360)+0xC000;    //90deg
        spi.write(buf);
        cs2=1;
    }else{
        cs2=0;
        buf=(4096*(base_pha)/360)+0xC000;    //0deg
        spi.write(buf);
        cs2=1;
    }

    cs1=0;
    spi.write(0x2000);      //accum. reset
    cs1=1;
    cs2=0;
    spi.write(0x2028);      //accum. reset
    cs2=1;
}

//adc read func.
float adc_read(uint8_t avg){
    uint8_t i;
    uint8_t buf[2];     //spi receive buf
    int16_t raw_val;
    float total=0;
    //cs3=0;
    //spi.write((wreg<<4));       //write addr 0x00, 1byte
    //spi.write((0b0110<<4)+1);   //ch1 ch0 bipolar mux, pga disable h81
    //cs3=1;
    for (i=0;i<avg;++i){
        //drdy wait
        while(true){
            if(drdy==0) break;
        }
        
        //read start
        cs3=0;
        buf[1]=spi.write(0x00);
        buf[0]=spi.write(0x00);
        cs3=1;
        raw_val=(buf[1]<<8)+buf[0];
        total=total+(float)raw_val;
    }
    return total/avg;
}

void vg_set(){
    switch (v_gain) {
        case 0:
            a0=0;
            a1=0;
            vg=1;
        break;
        case 1:
            a0=1;
            a1=0;
            vg=2;
        break;
        case 2:
            a0=0;
            a1=1;
            vg=4;
        break;
        case 3:
            a0=1;
            a1=1;
            vg=8;
        break;
    }
}

void ig_set(){
    switch (i_gain) {
        case 0:
            iv0=1;
            iv1=0;
            ig=152;
        break;
        case 1:
            iv0=1;
            iv1=1;
            ig=213;
        break;
        case 2:
            iv0=0;
            iv1=0;
            ig=1000;
        break;
    }
}

//disp char func
void char_disp(uint8_t addr, uint8_t position, char data){
    char buf[2];
    buf[0]=0x0;
    buf[1]=0x80+position;   //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    buf[0]=0x40;            //write command
    buf[1]=data;
    i2c.write(addr,buf,2);
}

//disp val func
void val_disp(uint8_t addr, uint8_t position, uint8_t digit, uint16_t val){
    char buf[2],data[4];
    uint8_t i;
    buf[0]=0x0;
    buf[1]=0x80+position;       //set cusor position (0x80 means cursor set cmd)
    i2c.write(addr,buf,2);
    data[3]=0x30+val%10;        //1
    data[2]=0x30+(val/10)%10;   //10
    data[1]=0x30+(val/100)%10;  //100
    data[0]=0x30+(val/1000)%10; //1000
    buf[0]=0x40;                //write command
    for(i=0;i<digit;++i){
        if(i==0&&data[0]==0x30&&digit==4) buf[1]=0x20;
        else buf[1]=data[i+4-digit];
        i2c.write(addr,buf,2);
    }
}

//LCD init func
void lcd_init(uint8_t addr, uint8_t contrast){
    char lcd_data[2];
    lcd_data[0]=0x0;
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x39;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x14;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x70|(contrast&0b1111);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x56|((contrast&0b00110000)>>4);
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x6C;
    i2c.write(addr,lcd_data,2);
    thread_sleep_for(200);
    lcd_data[1]=0x38;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x0C;
    i2c.write(addr,lcd_data,2);
    lcd_data[1]=0x01;
    i2c.write(addr,lcd_data,2);
}
