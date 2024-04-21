#include "hm01b0_func.h"


void hm01b0_init_fixed_rom_qvga_fixed(void){

//    hm_i2c_write(REG_SW_RESET, 0x00); //Software reset, reset all serial interface registers to its default values
    hm_i2c_write( REG_MODE_SELECT, 0x00);//go to stand by mode
    hm_i2c_write( REG_ANA_REGISTER_17, 0x00);//register to change the clk source(osc:1 mclk:0), if no mclk it goes to osc by default
    hm_i2c_write( REG_TEST_PATTERN_MODE, TEST_PATTERN);//Enable the test pattern, set it to walking 1

    hm_i2c_write( REG_BIN_MODE, 0x00);//VERTICAL BIN MODE
    hm_i2c_write( REG_QVGA_WIN_EN, 0x01);//Set line length LSB to QQVGA => enabled: makes the image 160(row)*240(col)
//    disable: image 160*320 //In test pattern mode, enabling this does not have any effect

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);

    //100*100 optimization
    hm_i2c_write( REG_BIN_RDOUT_X, 0x01);//Horizontal Binning enable
    hm_i2c_write( REG_BIN_RDOUT_Y, 0x01);//vertical Binning enable => this register should be always 0x03 because we never go more than 160 for the height
        //frame timing control
    hm_i2c_write(REG_FRAME_LENGTH_PCK_H,0x01);
    hm_i2c_write(REG_FRAME_LENGTH_PCK_L,0x78);//changed by Ali

    hm_i2c_write(REG_FRAME_LENGTH_LINES_H,0x02);//changed by Ali
    hm_i2c_write(REG_FRAME_LENGTH_LINES_L,0x12);//changed by Ali

    /*looking at lattice cfg setting*/
    //hm_i2c_write(0x0103,0x00);


    hm_i2c_write(0x3044,0x0A);
    hm_i2c_write(0x3045,0x00);
    hm_i2c_write(0x3047,0x0A);
    hm_i2c_write(0x3050,0xC0);
    hm_i2c_write(0x3051,0x42);
//    hm_i2c_write(0x3052,0x50);
    hm_i2c_write(0x3053,0x00);
    hm_i2c_write(0x3054,0x03);
    hm_i2c_write(0x3055,0xF7);
    hm_i2c_write(0x3056,0xF8);
    hm_i2c_write(0x3057,0x29);
    hm_i2c_write(0x3058,0x1F);
//    hm_i2c_write(0x3059,0x1E);//bit control
    hm_i2c_write(0x3064,0x00);
    hm_i2c_write(0x3065,0x04);

    //black level control
    hm_i2c_write(0x1000,0x43);
    hm_i2c_write(0x1001,0x40);
    hm_i2c_write(0x1002,0x32);
    hm_i2c_write(0x1003,0x08);//default from lattice 0x08
    hm_i2c_write(0x1006,0x01);
    hm_i2c_write(0x1007,0x08);//default from lattice 0x08

    hm_i2c_write(0x0350,0x7F);

    //Sensor reserved
    hm_i2c_write(0x1008,0x00);
    hm_i2c_write(0x1009,0xA0);
    hm_i2c_write(0x100A,0x60);
    hm_i2c_write(0x100B,0x90);//default from lattice 0x90
    hm_i2c_write(0x100C,0x40);//default from lattice 0x40

    //Vsync, hsync and pixel shift register
//    hm_i2c_write(0x1012,0x07);//changed by Ali
    hm_i2c_write(0x1012,0x00);//lattice value

    //Statistic control and read only
    hm_i2c_write(0x2000,0x07);
    hm_i2c_write(0x2003,0x00);
    hm_i2c_write(0x2004,0x1C);
    hm_i2c_write(0x2007,0x00);
    hm_i2c_write(0x2008,0x58);
    hm_i2c_write(0x200B,0x00);
    hm_i2c_write(0x200C,0x7A);
    hm_i2c_write(0x200F,0x00);
    hm_i2c_write(0x2010,0xB8);
    hm_i2c_write(0x2013,0x00);
    hm_i2c_write(0x2014,0x58);
    hm_i2c_write(0x2017,0x00);
    hm_i2c_write(0x2018,0x9B);

    //Automatic exposure gain control
    hm_i2c_write(0x2100,0x01);
    hm_i2c_write(0x2101,0x70);//0x70);//lattice 0xA0
    hm_i2c_write(0x2102,0x01);//lattice 0x06
    hm_i2c_write(0x2104,0x07);
    hm_i2c_write(0x2105,0x03);
    hm_i2c_write(0x2106,0xA4);
    hm_i2c_write(0x2108,0x33);
    hm_i2c_write(0x210A,0x00);
    //hm_i2c_write(0x210C,0x04);
    hm_i2c_write(0x210B,0x80);
    hm_i2c_write(0x210F,0x00);
    hm_i2c_write(0x2110,0xE9);
    hm_i2c_write(0x2111,0x01);
    hm_i2c_write(0x2112,0x17);
    hm_i2c_write(0x2150,0x03);

    //Sensor exposure gain
    hm_i2c_write(0x0205,0x05);//Vikram
    hm_i2c_write(0x020E,0x01);//Vikram
    hm_i2c_write(0x020F,0x00);//Vikram
    hm_i2c_write(0x0202,0x01);//Vikram
    hm_i2c_write(0x0203,0x08);//Vikram

    hm_i2c_write( REG_OSC_CLK_DIV, 0x30);//This is effective when we use external clk, Use the camera in the gated clock mode to make the clock zero when there is no data

    hm_i2c_write( REG_BIT_CONTROL, 0x20);//Set the output to send 1 bit serial

    hm_i2c_write( REG_PMU_PROGRAMMABLE_FRAMECNT, 0x01);//set the number of frames to be sent out, it sends N frames

}
