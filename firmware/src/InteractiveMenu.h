#ifndef _INTERACTIVEMENU_H_
#define _INTERACTIVEMENU_H_

#include <Arduino.h>
#include "MenuWindow.h"

class InteractiveMenu{

    private:
        int current_menu;

    public:
        MenuWindow *config_menus;

        InteractiveMenu(int no_of_menu_windows);
        ~InteractiveMenu();
        void clear_screen();
        void show_current_menu();
        bool option_is_valid(int option);
        bool set_option(int option);

};

#endif