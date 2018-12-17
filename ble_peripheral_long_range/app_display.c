#include "app_display.h"
#include "ugui.h"
#include "nrf_gfx_ext.h"
#include "images.h"

UG_WINDOW window_1;
char m_summary_string1[64];
char m_summary_string2[64];

extern const nrf_lcd_t nrf_lcd_ili9341;
static const nrf_lcd_t * p_lcd = &nrf_lcd_ili9341;

char *display_string_phy[] = {"Coded", "1 Mbps", "2 Mbps", "MultiPhy"};
char *display_string_tx_power[] = {"0 dBm", "4 dBm", "8 dBm"}; 
char *display_string_app_state[] = {"Idle", "Advertising", "Connected", "Disconnected"};
char *display_string_led_state[] = {"Off", "On"};
char *display_string_button_state[] = {"Off", "On"};
const UG_COLOR display_app_state_button_color[] = APP_STATE_COLORS;
const UG_COLOR display_app_state_button_font_color[] = APP_STATE_FONT_COLORS;
const UG_COLOR display_on_off_color[] = ON_OFF_COLORS;
const UG_COLOR display_on_off_font_color[] = ON_OFF_FONT_COLORS;

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
UG_TEXTBOX textbox_led_label;
UG_TEXTBOX textbox_button_label;
UG_BUTTON phy_button;
UG_BUTTON power_button;
UG_BUTTON adv_button;
UG_BUTTON led_button;
UG_BUTTON button_button;
UG_IMAGE  image_1;

#define MAX_OBJECTS 20

UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];

static app_display_content_t content_previous = {0};

void window_1_callback (UG_MESSAGE *msg)
{
    UNUSED_PARAMETER(msg);    
}

void app_display_init(app_display_content_t *initial_state)
{
    content_previous = *initial_state;
    UG_Init(&gui, 240, 320, p_lcd);
}

void app_display_create_main_screen(app_display_content_t *content)
{
    /* Create the window */
    UG_WindowCreate ( &window_1 , obj_buff_wnd_1 , MAX_OBJECTS, window_1_callback ) ;
    /* Modify the window t i t l e */
    UG_WindowSetTitleText (&window_1 , content->main_title) ;
    UG_WindowSetTitleTextFont (&window_1 , &FONT_10X16) ;
    UG_WindowSetTitleTextAlignment(&window_1, ALIGN_CENTER);

    /* Create "Toggle PHY" textbox (TXB_ID_0) */
    UG_TextboxCreate(&window_1, &textbox_toggle_phy, TXB_ID_0, TXT_ID_0_X_LOCATION, TXT_ID_0_Y_LOCATION, TXT_ID_0_X_LOCATION+TXT_ID_0_WIDTH, TXT_ID_0_Y_LOCATION+TXT_ID_0_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_0, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_0 , "Btn 1: PHY") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_0 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_0 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_0 , ALIGN_CENTER );

    /* Create "PHY" selection button (BTN_ID_0) */
    UG_ButtonCreate(&window_1, &phy_button, BTN_ID_0, BTN_ID_0_X_LOCATION, BTN_ID_0_Y_LOCATION, BTN_ID_0_X_LOCATION+BTN_ID_0_WIDTH, BTN_ID_0_Y_LOCATION+BTN_ID_0_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_0, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_0, FONT_COLOR_BUTTON);
    UG_ButtonSetBackColor(&window_1, BTN_ID_0, FILL_COLOR_BUTTON);  
    UG_ButtonSetFont(&window_1, BTN_ID_0, &FONT_10X16);

    /* Create "Toggle output power" textbox (TXB_ID_1) */
    UG_TextboxCreate(&window_1, &textbox_toggle_power, TXB_ID_1, TXT_ID_1_X_LOCATION, TXT_ID_1_Y_LOCATION, TXT_ID_1_X_LOCATION+TXT_ID_1_WIDTH, TXT_ID_1_Y_LOCATION+TXT_ID_1_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_1, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_1 , "Btn 2: Power") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_1, FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_1, FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_1, ALIGN_CENTER );  

    /* Create "Power" selection button (BTN_ID_1) */
    UG_ButtonCreate(&window_1, &power_button, BTN_ID_1, BTN_ID_1_X_LOCATION, BTN_ID_1_Y_LOCATION, BTN_ID_1_X_LOCATION+BTN_ID_1_WIDTH, BTN_ID_1_Y_LOCATION+BTN_ID_1_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_1, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetForeColor(&window_1, BTN_ID_1, FONT_COLOR_BUTTON);
    UG_ButtonSetBackColor(&window_1, BTN_ID_1, FILL_COLOR_BUTTON);  
    UG_ButtonSetFont(&window_1, BTN_ID_1, &FONT_10X16);
    //UG_ButtonSetText(&window_1, BTN_ID_1, "0 dBm"); 

    /* Create "Toggle output power" textbox (TXB_ID_2) */
    UG_TextboxCreate(&window_1, &textbox_toggle_adv_type, TXB_ID_2, TXT_ID_2_X_LOCATION, TXT_ID_2_Y_LOCATION, TXT_ID_2_X_LOCATION+TXT_ID_2_WIDTH, TXT_ID_2_Y_LOCATION+TXT_ID_2_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_2, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_2 , "Btn 3: App state") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_2, FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_2, FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_2, ALIGN_CENTER );  

    /* Create "App State" selection button (BTN_ID_2) */
    UG_ButtonCreate(&window_1, &adv_button, BTN_ID_2, BTN_ID_2_X_LOCATION, BTN_ID_2_Y_LOCATION, BTN_ID_2_X_LOCATION+BTN_ID_2_WIDTH, BTN_ID_2_Y_LOCATION+BTN_ID_2_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_2, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetFont(&window_1, BTN_ID_2, &FONT_10X16);
    
    /* Create LED status textbox (TXB_ID_10) */
    UG_TextboxCreate(&window_1, &textbox_led_label, TXB_ID_10, TXT_ID_10_X_LOCATION, TXT_ID_10_Y_LOCATION, TXT_ID_10_X_LOCATION+TXT_ID_10_WIDTH, TXT_ID_10_Y_LOCATION+TXT_ID_10_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_10, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_10 , "LED") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_10, FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_10, FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_10, ALIGN_CENTER );  

    /* Create LED button (BTN_ID_3) */
    UG_ButtonCreate(&window_1, &led_button, BTN_ID_3, BTN_ID_3_X_LOCATION, BTN_ID_3_Y_LOCATION, BTN_ID_3_X_LOCATION+BTN_ID_3_WIDTH, BTN_ID_3_Y_LOCATION+BTN_ID_3_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_3, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetFont(&window_1, BTN_ID_3, &FONT_10X16);
    
    /* Create button status textbox (TXB_ID_10) */
    UG_TextboxCreate(&window_1, &textbox_button_label, TXB_ID_11, TXT_ID_11_X_LOCATION, TXT_ID_11_Y_LOCATION, TXT_ID_11_X_LOCATION+TXT_ID_11_WIDTH, TXT_ID_11_Y_LOCATION+TXT_ID_11_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_11, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_11 , "Button 4") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_11, FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_11, FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_11, ALIGN_CENTER );  

    /* Create button button (BTN_ID_3) */
    UG_ButtonCreate(&window_1, &button_button, BTN_ID_4, BTN_ID_4_X_LOCATION, BTN_ID_4_Y_LOCATION, BTN_ID_4_X_LOCATION+BTN_ID_4_WIDTH, BTN_ID_4_Y_LOCATION+BTN_ID_4_HEIGHT);
    UG_ButtonSetStyle(&window_1, BTN_ID_4, BTN_STYLE_3D|BTN_STYLE_USE_ALTERNATE_COLORS);
    UG_ButtonSetFont(&window_1, BTN_ID_4, &FONT_10X16);

    /* Create "RSSI" textbox (TXB_ID_3) */
    UG_TextboxCreate(&window_1, &textbox_rssi_label, TXB_ID_3, TXT_ID_3_X_LOCATION, TXT_ID_3_Y_LOCATION, TXT_ID_3_X_LOCATION+TXT_ID_3_WIDTH, TXT_ID_3_Y_LOCATION+TXT_ID_3_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_3, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_3 , "RSSI:") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_3 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_3 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_3 , ALIGN_CENTER );

    /* Create "RSSI" textbox for the numbers (TXB_ID_5) */
    UG_TextboxCreate(&window_1, &textbox_rssi_number, TXB_ID_5, TXT_ID_5_X_LOCATION, TXT_ID_5_Y_LOCATION, TXT_ID_5_X_LOCATION+TXT_ID_5_WIDTH, TXT_ID_5_Y_LOCATION+TXT_ID_5_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_5, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_5 , "-") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_5 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_5 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_5 , ALIGN_CENTER );

    /* Create "dBm" textbox for the RSSI metric (TXB_ID_7) */
    UG_TextboxCreate(&window_1, &textbox_dbm_label, TXB_ID_7, TXT_ID_7_X_LOCATION, TXT_ID_7_Y_LOCATION, TXT_ID_7_X_LOCATION+TXT_ID_7_WIDTH, TXT_ID_7_Y_LOCATION+TXT_ID_7_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_7, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_7 , "dBm") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_7 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_7 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_7 , ALIGN_CENTER );

#if 0
    /* Create "PER" textbox (TXB_ID_4) */
    UG_TextboxCreate(&window_1, &textbox_per_label, TXB_ID_4, TXT_ID_4_X_LOCATION, TXT_ID_4_Y_LOCATION, TXT_ID_4_X_LOCATION+TXT_ID_4_WIDTH, TXT_ID_4_Y_LOCATION+TXT_ID_4_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_4, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_4 , "PER: ") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_4 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_4 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_4 , ALIGN_CENTER );

    /* Create "PER" textbox for the numbers (TXB_ID_6) */
    UG_TextboxCreate(&window_1, &textbox_per_number, TXB_ID_6, TXT_ID_6_X_LOCATION, TXT_ID_6_Y_LOCATION, TXT_ID_6_X_LOCATION+TXT_ID_6_WIDTH, TXT_ID_6_Y_LOCATION+TXT_ID_6_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_6, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_6 , "###") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_6 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_6 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_6 , ALIGN_CENTER );
    
    /* Create "%" textbox for the PER metric (TXB_ID_8) */
    UG_TextboxCreate(&window_1, &textbox_percentage_label, TXB_ID_8, TXT_ID_8_X_LOCATION, TXT_ID_8_Y_LOCATION, TXT_ID_8_X_LOCATION+TXT_ID_8_WIDTH, TXT_ID_8_Y_LOCATION+TXT_ID_8_HEIGHT);  
    UG_TextboxSetFont(&window_1, TXB_ID_8, &FONT_8X12);
    UG_TextboxSetText(&window_1 , TXB_ID_8 , "%") ;
    UG_TextboxSetForeColor (&window_1 , TXB_ID_8 , FONT_COLOR_TEXT ) ;
    UG_TextboxSetBackColor (&window_1 , TXB_ID_8 , FILL_COLOR_TEXT ); 
    UG_TextboxSetAlignment (&window_1 , TXB_ID_8 , ALIGN_CENTER );
#endif 

    UG_ImageCreate(&window_1, &image_1, IMG_ID_0, 0, 228, 30, 250);
    UG_ImageSetBMP(&window_1, IMG_ID_0, &bmp_nordicsemi);

    /* Update the dynamic elements of the screen */
    app_display_update_main_screen(content);

    /* Finally , show the window */
    UG_WindowShow( &window_1 ) ;
}

static char sprintf_buf[64];

void app_display_update_main_screen(app_display_content_t *content)
{
    static bool first_update = true;
    if(first_update || content->phy != content_previous.phy)
    {
        UG_ButtonSetText(&window_1, BTN_ID_0, display_string_phy[content->phy]);    
    }
    if(first_update || content->tx_power != content_previous.tx_power)
    {
        UG_ButtonSetText(&window_1, BTN_ID_1, display_string_tx_power[content->tx_power]); 
    }
    if(first_update || content->app_state != content_previous.app_state)
    {
        UG_ButtonSetBackColor(&window_1, BTN_ID_2, display_app_state_button_color[content->app_state]); 
        UG_ButtonSetForeColor(&window_1, BTN_ID_2, display_app_state_button_font_color[content->app_state]);
        UG_ButtonSetText(&window_1, BTN_ID_2, display_string_app_state[content->app_state]); 
        if(content->app_state == APP_STATE_CONNECTED)
        {
            UG_TextboxShow(&window_1, TXB_ID_3);
            UG_TextboxShow(&window_1, TXB_ID_5);
            UG_TextboxShow(&window_1, TXB_ID_7);
        }
        else
        {
            UG_TextboxHide(&window_1, TXB_ID_3);
            UG_TextboxHide(&window_1, TXB_ID_5);
            UG_TextboxHide(&window_1, TXB_ID_7);            
        }
    }
    if(first_update || content->led_on != content_previous.led_on)
    {
        UG_ButtonSetBackColor(&window_1, BTN_ID_3, display_on_off_color[content->led_on ? 1 : 0]); 
        UG_ButtonSetForeColor(&window_1, BTN_ID_3, display_on_off_font_color[content->led_on ? 1 : 0]);
        UG_ButtonSetText(&window_1, BTN_ID_3, display_string_led_state[content->led_on ? 1 : 0]); 
    }
    if(first_update || content->button_pressed != content_previous.button_pressed)
    {
        UG_ButtonSetBackColor(&window_1, BTN_ID_4, display_on_off_color[content->button_pressed ? 1 : 0]); 
        UG_ButtonSetForeColor(&window_1, BTN_ID_4, display_on_off_font_color[content->button_pressed ? 1 : 0]);
        UG_ButtonSetText(&window_1, BTN_ID_4, display_string_led_state[content->button_pressed ? 1 : 0]); 
    }
    if(content->rssi != content_previous.rssi)
    {
        sprintf(sprintf_buf, "%i", (int)content->rssi);
        UG_TextboxSetText(&window_1 , TXB_ID_5 , content->rssi != 0 ? sprintf_buf : "-") ;
    }
    first_update = false;
    content_previous = *content;
}

void app_display_update(void)
{
    UG_Update();
}
