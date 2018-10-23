#include "app_display.h"
#include "ugui.h"
#include "nrf_gfx.h"

UG_WINDOW window_1;
char m_summary_string1[64];
char m_summary_string2[64];

extern const nrf_lcd_t nrf_lcd_ili9341;
static const nrf_lcd_t * p_lcd = &nrf_lcd_ili9341;

UG_GUI gui;
UG_TEXTBOX textbox_toggle_phy;
UG_TEXTBOX textbox_toggle_power;
UG_TEXTBOX textbox_toggle_adv_type; //advertising type (non-connectable / connectable)
UG_TEXTBOX textbox_rssi_label;
UG_TEXTBOX textbox_per_label;
UG_TEXTBOX textbox_rssi_number;
UG_TEXTBOX textbox_per_number;
UG_TEXTBOX textbox_dbm_label; 
UG_TEXTBOX textbox_percentage_label;
UG_TEXTBOX textbox_link_indicator;
UG_BUTTON phy_button;
UG_BUTTON power_button;
UG_BUTTON adv_button;
#define MAX_OBJECTS 15
UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];

void window_1_callback (UG_MESSAGE *msg)
{
    UNUSED_PARAMETER(msg);    
}

void app_display_init(void)
{
    UG_Init(&gui, 240, 320, p_lcd);
}

void app_display_draw_main_screen(void)
{
    /* Create the window */
    UG_WindowCreate ( &window_1 , obj_buff_wnd_1 , MAX_OBJECTS, window_1_callback ) ;
    /* Modify the window t i t l e */
    UG_WindowSetTitleText (&window_1 , "Nordic Range Demo") ;
    UG_WindowSetTitleTextFont (&window_1 , &FONT_10X16) ;
    UG_WindowSetTitleTextAlignment(&window_1, ALIGN_CENTER);

    /* Create "Toggle PHY" textbox (TXB_ID_0) */
    UG_TextboxCreate(&window_1, &textbox_toggle_phy, TXB_ID_0, TXT_ID_0_X_LOCATION, TXT_ID_0_Y_LOCATION, TXT_ID_0_X_LOCATION+TXT_ID_0_WIDTH, TXT_ID_0_Y_LOCATION+TXT_ID_0_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_0, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_0 , "Button 1: PHY") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_0 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_0 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_0 , ALIGN_CENTER );

    /* Create "PHY" selection button (BTN_ID_0) */
    UG_ButtonCreate(&window_1, &phy_button, BTN_ID_0, BTN_ID_0_X_LOCATION, BTN_ID_0_Y_LOCATION, BTN_ID_0_X_LOCATION+BTN_ID_0_WIDTH, BTN_ID_0_Y_LOCATION+BTN_ID_0_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_0, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_0, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_0, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_0, &FONT_10X16);
    UG_ButtonSetText(&window_1, BTN_ID_0, "1 Mbps");    

    /* Create "Toggle output power" textbox (TXB_ID_1) */
    UG_TextboxCreate(&window_1, &textbox_toggle_power, TXB_ID_1, TXT_ID_1_X_LOCATION, TXT_ID_1_Y_LOCATION, TXT_ID_1_X_LOCATION+TXT_ID_1_WIDTH, TXT_ID_1_Y_LOCATION+TXT_ID_1_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_1, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_1 , "Button 2: Power") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_1, C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_1, C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_1, ALIGN_CENTER );  

    /* Create "Power" selection button (BTN_ID_1) */
    UG_ButtonCreate(&window_1, &power_button, BTN_ID_1, BTN_ID_1_X_LOCATION, BTN_ID_1_Y_LOCATION, BTN_ID_1_X_LOCATION+BTN_ID_1_WIDTH, BTN_ID_1_Y_LOCATION+BTN_ID_1_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_1, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_1, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_1, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_1, &FONT_10X16);
    UG_ButtonSetText(&window_1, BTN_ID_1, "0 dBm"); 

    /* Create "Toggle output power" textbox (TXB_ID_2) */
    UG_TextboxCreate(&window_1, &textbox_toggle_adv_type, TXB_ID_2, TXT_ID_2_X_LOCATION, TXT_ID_2_Y_LOCATION, TXT_ID_2_X_LOCATION+TXT_ID_2_WIDTH, TXT_ID_2_Y_LOCATION+TXT_ID_2_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_2, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_2 , "Button 3: Adv. type") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_2, C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_2, C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_2, ALIGN_CENTER );  

    /* Create "Advertising type" selection button (BTN_ID_2) */
    UG_ButtonCreate(&window_1, &adv_button, BTN_ID_2, BTN_ID_2_X_LOCATION, BTN_ID_2_Y_LOCATION, BTN_ID_2_X_LOCATION+BTN_ID_2_WIDTH, BTN_ID_2_Y_LOCATION+BTN_ID_2_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_2, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_2, C_YELLOW);
    UG_ButtonSetBackColor(&window_1, BTN_ID_2, C_MEDIUM_BLUE);  
    UG_ButtonSetFont(&window_1, BTN_ID_2, &FONT_10X16);
    UG_ButtonSetText(&window_1, BTN_ID_2, "Connectable"); 

    /* Create "RSSI" textbox (TXB_ID_3) */
    UG_TextboxCreate(&window_1, &textbox_rssi_label, TXB_ID_3, TXT_ID_3_X_LOCATION, TXT_ID_3_Y_LOCATION, TXT_ID_3_X_LOCATION+TXT_ID_3_WIDTH, TXT_ID_3_Y_LOCATION+TXT_ID_3_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_3, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_3 , "RSSI: ") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_3 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_3 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_3 , ALIGN_CENTER );

    /* Create "PER" textbox (TXB_ID_4) */
    UG_TextboxCreate(&window_1, &textbox_per_label, TXB_ID_4, TXT_ID_4_X_LOCATION, TXT_ID_4_Y_LOCATION, TXT_ID_4_X_LOCATION+TXT_ID_4_WIDTH, TXT_ID_4_Y_LOCATION+TXT_ID_4_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_4, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_4 , "PER: ") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_4 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_4 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_4 , ALIGN_CENTER );

    /* Create "RSSI" textbox for the numbers (TXB_ID_5) */
    UG_TextboxCreate(&window_1, &textbox_rssi_number, TXB_ID_5, TXT_ID_5_X_LOCATION, TXT_ID_5_Y_LOCATION, TXT_ID_5_X_LOCATION+TXT_ID_5_WIDTH, TXT_ID_5_Y_LOCATION+TXT_ID_5_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_5, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_5 , "###") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_5 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_5 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_5 , ALIGN_CENTER );

    /* Create "PER" textbox for the numbers (TXB_ID_6) */
    UG_TextboxCreate(&window_1, &textbox_per_number, TXB_ID_6, TXT_ID_6_X_LOCATION, TXT_ID_6_Y_LOCATION, TXT_ID_6_X_LOCATION+TXT_ID_6_WIDTH, TXT_ID_6_Y_LOCATION+TXT_ID_6_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_6, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_6 , "###") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_6 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_6 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_6 , ALIGN_CENTER );

    /* Create "dBm" textbox for the RSSI metric (TXB_ID_7) */
    UG_TextboxCreate(&window_1, &textbox_dbm_label, TXB_ID_7, TXT_ID_7_X_LOCATION, TXT_ID_7_Y_LOCATION, TXT_ID_7_X_LOCATION+TXT_ID_7_WIDTH, TXT_ID_7_Y_LOCATION+TXT_ID_7_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_7, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_7 , "dBm") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_7 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_7 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_7 , ALIGN_CENTER );

    /* Create "%" textbox for the PER metric (TXB_ID_8) */
    UG_TextboxCreate(&window_1, &textbox_percentage_label, TXB_ID_8, TXT_ID_8_X_LOCATION, TXT_ID_8_Y_LOCATION, TXT_ID_8_X_LOCATION+TXT_ID_8_WIDTH, TXT_ID_8_Y_LOCATION+TXT_ID_8_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_8, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_8 , "%") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_8 , C_MAROON ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_8 , C_DODGER_BLUE ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_8 , ALIGN_CENTER );

    /* Create Link textbox for the Link indicator (TXB_ID_9) */
    UG_TextboxCreate(&window_1, &textbox_link_indicator, TXB_ID_9, TXT_ID_9_X_LOCATION, TXT_ID_9_Y_LOCATION, TXT_ID_9_X_LOCATION+TXT_ID_9_WIDTH, TXT_ID_9_Y_LOCATION+TXT_ID_9_HEIGHT);      
    UG_TextboxSetBackColor (&window_1 , TXB_ID_9 , C_DARK_RED );   

    /* Finally , show the window */
    UG_WindowShow( &window_1 ) ;
}


void app_display_update(void)
{
    UG_Update();
}
