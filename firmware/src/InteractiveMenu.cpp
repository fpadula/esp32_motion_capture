#include "InteractiveMenu.h"

InteractiveMenu::InteractiveMenu(int no_of_menu_windows){
    this->current_menu = 0;
    this->config_menus = new MenuWindow[no_of_menu_windows];
}

InteractiveMenu::~InteractiveMenu(){
    delete this->config_menus;
}

void InteractiveMenu::clear_screen(){
    for(int i = 0; i < 127; i++)
        Serial.print('\n');                
}

void InteractiveMenu::show_current_menu(){
    this->config_menus[this->current_menu].print();
}

bool InteractiveMenu::option_is_valid(int option){
    return this->config_menus[this->current_menu].is_option_valid(option);
}

bool InteractiveMenu::set_option(int option){
    if(this->option_is_valid(option)){
        if (this->config_menus[this->current_menu].option_has_callback(option)){
            this->config_menus[this->current_menu].exec_callback(option);
        }
        this->current_menu = this->config_menus[this->current_menu].get_option_value(option);
        return true;
    }
    return false;
}